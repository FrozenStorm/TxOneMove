#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UART Waveform Interpreter
Dekodiert UART Telegramme aus Oszilloskop CSV Dateien
mit Unterstützung für CSRF Protokoll
"""

import csv
import struct
from dataclasses import dataclass
from typing import List, Tuple, Optional
import argparse


@dataclass
class UArtConfig:
    """UART Konfigurationsparameter"""
    baud_rate: int = 420000
    data_bits: int = 8
    stop_bits: int = 1
    parity: str = 'N'  # N=None, E=Even, O=Odd
    lsb_first: bool = True
    idle_voltage: float = 3.2  # High State (3.3V oder 3.2V)
    low_voltage: float = 0.0   # Low State
    voltage_threshold: float = 1.6  # Middle point for detection


@dataclass
class UArtBit:
    """Einzelnes UART Bit"""
    time_start: float
    time_end: float
    value: int  # 0 oder 1
    duration: float
    bit_type: str  # 'START', 'DATA', 'PARITY', 'STOP'


@dataclass
class UArtByte:
    """Dekodiertes UART Byte"""
    value: int
    bits: List[UArtBit]
    timestamp: float
    parity_ok: bool = True
    error_msg: str = ""


class UArtDecoder:
    """UART Waveform Decoder"""
    
    def __init__(self, config: UArtConfig = None):
        self.config = config or UArtConfig()
        self.bit_duration = 1.0 / self.config.baud_rate
        
    def load_csv(self, filename: str) -> Tuple[List[float], List[float]]:
        """Lädt CSV Datei und gibt Zeit- und Spannungsvektoren zurück"""
        timestamps = []
        voltages = []
        
        with open(filename, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            in_data = False
            
            for row_idx, row in enumerate(reader):
                # Überspringe Header bis zur Datensektion
                if len(row) >= 2:
                    try:
                        time_val = float(row[0])
                        volt_val = float(row[1])
                        timestamps.append(time_val)
                        voltages.append(volt_val)
                        in_data = True
                    except (ValueError, IndexError):
                        if in_data:
                            break
        
        return timestamps, voltages
    
    def detect_bits(self, timestamps: List[float], voltages: List[float]) -> List[UArtBit]:
        """Erkennt einzelne Bits aus der Waveform"""
        bits = []
        
        # Vereinfachte Edge Detection
        threshold = self.config.voltage_threshold
        
        bit_start = None
        current_bit = None
        edge_times = []
        
        for i, (time, volt) in enumerate(zip(timestamps, voltages)):
            if i == 0:
                current_bit = 1 if volt > threshold else 0
                bit_start = time
                continue
            
            # Spannungswechsel erkannt
            new_bit = 1 if volt > threshold else 0
            
            if new_bit != current_bit:
                edge_times.append(time)
                duration = time - bit_start
                
                # Filtere zu kurze Pulses (Rauschen)
                if duration > self.bit_duration * 0.3:
                    bits.append(UArtBit(
                        time_start=bit_start,
                        time_end=time,
                        value=current_bit,
                        duration=duration,
                        bit_type='DATA'
                    ))
                    bit_start = time
                    current_bit = new_bit
        
        return bits
    
    def extract_bytes(self, bits: List[UArtBit]) -> List[UArtByte]:
        """Extrahiert Bytes aus Bits basierend auf Bit-Dauer"""
        bytes_decoded = []
        
        if not bits:
            return bytes_decoded
        
        # Gruppiere Bits in Bytes basierend auf Timing
        current_byte_bits = []
        byte_start_time = bits[0].time_start
        
        for bit in bits:
            # Berechne erwartete Bitdauer
            bits_since_start = len(current_byte_bits)
            expected_bit_position = bits_since_start * self.bit_duration
            
            # Prüfe ob dieses Bit Teil desselben Bytes ist
            time_since_start = bit.time_start - byte_start_time
            
            if time_since_start < (self.config.data_bits + 2) * self.bit_duration * 1.5:
                current_byte_bits.append(bit)
            else:
                # Neues Byte beginnt
                if current_byte_bits:
                    byte_obj = self._decode_byte(current_byte_bits, byte_start_time)
                    bytes_decoded.append(byte_obj)
                
                current_byte_bits = [bit]
                byte_start_time = bit.time_start
        
        # Letztes Byte
        if current_byte_bits:
            byte_obj = self._decode_byte(current_byte_bits, byte_start_time)
            bytes_decoded.append(byte_obj)
        
        return bytes_decoded
    
    def _decode_byte(self, bits: List[UArtBit], timestamp: float) -> UArtByte:
        """Dekodiert ein einzelnes Byte"""
        value = 0
        bit_count = 0
        parity_ok = True
        error_msg = ""
        
        # Erwartetes Format: START (0) + DATA (8) + PARITY (optional) + STOP (1)
        
        if not bits:
            return UArtByte(0, [], timestamp, False, "Keine Bits")
        
        # START Bit (sollte 0 sein)
        if bits[0].value != 0:
            error_msg += "START Bit ist 1 (sollte 0 sein); "
        
        # DATA Bits
        data_bits = bits[1:1+self.config.data_bits]
        
        for i, bit in enumerate(data_bits):
            if self.config.lsb_first:
                value |= (bit.value << i)
            else:
                value |= (bit.value << (self.config.data_bits - 1 - i))
            bit.bit_type = 'DATA'
            bit_count += 1
        
        # Parity Check (falls aktiviert)
        if self.config.parity != 'N' and len(bits) > self.config.data_bits + 1:
            parity_bit_idx = self.config.data_bits + 1
            if parity_bit_idx < len(bits):
                parity_ok = self._check_parity(value, bits[parity_bit_idx], self.config.parity)
                bits[parity_bit_idx].bit_type = 'PARITY'
                if not parity_ok:
                    error_msg += "Parity Fehler; "
        
        # STOP Bits
        if len(bits) > self.config.data_bits + (2 if self.config.parity != 'N' else 1):
            stop_bit_idx = self.config.data_bits + (2 if self.config.parity != 'N' else 1)
            if stop_bit_idx < len(bits):
                bits[stop_bit_idx].bit_type = 'STOP'
                if bits[stop_bit_idx].value != 1:
                    error_msg += "STOP Bit ist 0 (sollte 1 sein); "
        
        return UArtByte(value, bits, timestamp, parity_ok, error_msg)
    
    def _check_parity(self, value: int, parity_bit: UArtBit, parity_type: str) -> bool:
        """Prüft Parität"""
        ones_count = bin(value).count('1')
        
        if parity_type == 'E':  # Even
            expected_parity = ones_count % 2
        elif parity_type == 'O':  # Odd
            expected_parity = 1 - (ones_count % 2)
        else:
            return True
        
        return parity_bit.value == expected_parity


class CSRFDecoder:
    """CSRF Protokoll Decoder (Character Serial Frame Format)"""
    
    def __init__(self):
        self.frame_header = 0xAA
        self.frame_esc = 0xBB
        self.frame_end = 0xCC
    
    def decode_frame(self, data: bytes) -> dict:
        """Dekodiert ein CSRF Frame"""
        frame = {
            'valid': False,
            'header': None,
            'length': None,
            'payload': b'',
            'checksum': None,
            'checksum_ok': False,
            'escaped_data': False,
            'error': ''
        }
        
        if len(data) < 4:
            frame['error'] = 'Zu kurz für CSRF Frame'
            return frame
        
        idx = 0
        
        # Suche Frame Header
        if data[idx] == self.frame_header:
            frame['header'] = data[idx]
            idx += 1
        else:
            frame['error'] = f'Ungültiger Header: {hex(data[idx])}'
            return frame
        
        # Länge
        if idx < len(data):
            frame['length'] = data[idx]
            idx += 1
        else:
            frame['error'] = 'Keine Länge vorhanden'
            return frame
        
        # Payload mit Escape-Sequenz-Handling
        payload = b''
        while idx < len(data) and len(payload) < frame['length']:
            if data[idx] == self.frame_esc:
                frame['escaped_data'] = True
                idx += 1
                if idx < len(data):
                    payload += bytes([data[idx] ^ 0xFF])  # XOR Dekodierung
                    idx += 1
            elif data[idx] == self.frame_end:
                break
            else:
                payload += bytes([data[idx]])
                idx += 1
        
        frame['payload'] = payload
        
        # Checksum
        if idx < len(data) and data[idx] == self.frame_end:
            idx += 1
            if idx < len(data):
                frame['checksum'] = data[idx]
                # Berechne Checksum
                calc_checksum = self._calculate_checksum(payload)
                frame['checksum_ok'] = (calc_checksum == frame['checksum'])
                if not frame['checksum_ok']:
                    frame['error'] = f'Checksum Fehler: erwartet {hex(calc_checksum)}, erhalten {hex(frame["checksum"])}'
        
        frame['valid'] = len(payload) == frame['length'] and frame['checksum_ok']
        return frame
    
    def _calculate_checksum(self, data: bytes) -> int:
        """Berechnet Checksum (einfaches Modulo)"""
        return sum(data) & 0xFF
    
    def encode_frame(self, payload: bytes) -> bytes:
        """Enkodiert ein CSRF Frame"""
        frame = bytes([self.frame_header])
        frame += bytes([len(payload)])
        
        # Escape Handling
        for byte in payload:
            if byte in [self.frame_header, self.frame_esc, self.frame_end]:
                frame += bytes([self.frame_esc, byte ^ 0xFF])
            else:
                frame += bytes([byte])
        
        frame += bytes([self.frame_end])
        frame += bytes([self._calculate_checksum(payload)])
        
        return frame


class UArtAnalyzer:
    """Hauptanalyse-Klasse"""
    
    def __init__(self, csv_file: str, baud_rate: int = 9600):
        self.uart_config = UArtConfig(baud_rate=baud_rate)
        self.decoder = UArtDecoder(self.uart_config)
        self.csrf_decoder = CSRFDecoder()
        self.csv_file = csv_file
        self.bytes_data = []
        self.raw_data = b''
        
    def analyze(self) -> None:
        """Führt komplette Analyse durch"""
        print("=" * 80)
        print("UART Waveform Interpreter - CSRF Protokoll Decoder")
        print("=" * 80)
        
        # Lade CSV
        print(f"\n[1] Lade CSV Datei: {self.csv_file}")
        timestamps, voltages = self.decoder.load_csv(self.csv_file)
        print(f"    Zeitstempel: {len(timestamps)}, Spannungswerte: {len(voltages)}")
        print(f"    Zeit Range: {timestamps[0]:.6e} - {timestamps[-1]:.6e} s")
        
        # Erkenne Bits
        print(f"\n[2] Erkenne Bits aus Waveform")
        bits = self.decoder.detect_bits(timestamps, voltages)
        print(f"    Erkannte Bits: {len(bits)}")
        
        # Extrahiere Bytes
        print(f"\n[3] Extrahiere Bytes")
        self.bytes_data = self.decoder.extract_bytes(bits)
        print(f"    Dekodierte Bytes: {len(self.bytes_data)}")
        
        # Zeige Raw Daten
        self._print_raw_data()
        
        # Dekodiere CSRF Frames
        print(f"\n[4] Dekodiere CSRF Frames")
        self._decode_csrf_frames()
        
    def _print_raw_data(self) -> None:
        """Gibt dekodierte Raw-Daten aus"""
        print(f"\n[RAW DATA - {len(self.bytes_data)} Bytes]")
        print("-" * 80)
        
        hex_line = ""
        ascii_line = ""
        
        for i, byte_obj in enumerate(self.bytes_data):
            value = byte_obj.value
            self.raw_data += bytes([value])
            
            # Hex Ausgabe
            hex_str = f"{value:02X}"
            hex_line += hex_str + " "
            
            # ASCII Ausgabe
            if 32 <= value <= 126:
                ascii_line += chr(value)
            else:
                ascii_line += "."
            
            # 16 Bytes pro Zeile
            if (i + 1) % 16 == 0:
                print(f"{i-15:04d}: {hex_line:48s} | {ascii_line}")
                hex_line = ""
                ascii_line = ""
            
            # Zeige Fehler
            if byte_obj.error_msg:
                print(f"       ⚠️  Byte {i}: {byte_obj.error_msg}")
        
        # Letzte Zeile
        if hex_line:
            print(f"{len(self.bytes_data)-len(ascii_line):04d}: {hex_line:48s} | {ascii_line}")
        
        print(f"\nHex String: {self.raw_data.hex().upper()}")
    
    def _decode_csrf_frames(self) -> None:
        """Dekodiert CSRF Frames aus Raw-Daten"""
        if not self.raw_data:
            print("    Keine Raw-Daten vorhanden")
            return
        
        frame_num = 0
        idx = 0
        
        while idx < len(self.raw_data):
            # Suche Frame Header (0xAA)
            if self.raw_data[idx] != 0xAA:
                idx += 1
                continue
            
            # Extrahiere potenzielles Frame
            frame_data = self.raw_data[idx:]
            if len(frame_data) < 4:
                break
            
            # Dekodiere Frame
            frame = self.csrf_decoder.decode_frame(frame_data)
            
            frame_num += 1
            print(f"\n    Frame {frame_num}:")
            print(f"      Gültig: {'✓' if frame['valid'] else '✗'}")
            print(f"      Header: {hex(frame['header']) if frame['header'] else 'N/A'}")
            print(f"      Länge: {frame['length']}")
            print(f"      Payload: {frame['payload'].hex().upper()} ({len(frame['payload'])} Bytes)")
            print(f"      Payload ASCII: {frame['payload']}")
            print(f"      Checksum: {hex(frame['checksum']) if frame['checksum'] else 'N/A'} - {'OK' if frame['checksum_ok'] else 'FEHLER'}")
            
            if frame['error']:
                print(f"      ⚠️  Fehler: {frame['error']}")
            
            # Bestimme Frame-Ende
            frame_end_marker_idx = frame_data.find(bytes([0xCC]))
            if frame_end_marker_idx >= 0:
                idx += frame_end_marker_idx + 2 + (1 if len(frame_data) > frame_end_marker_idx + 1 else 0)
            else:
                idx += len(frame['payload']) + 4
            
            if idx >= len(self.raw_data):
                break


def main():
    parser = argparse.ArgumentParser(
        description='UART Waveform Interpreter mit CSRF Protokoll Support'
    )
    parser.add_argument('csv_file', type=str, default=".\measurement\WA000001.CS", help='CSV Datei vom Oszilloskop')
    parser.add_argument('--baud', type=int, default=9600, help='Baud Rate (Standard: 9600)')
    parser.add_argument('--data-bits', type=int, default=8, help='Datenbits (Standard: 8)')
    parser.add_argument('--stop-bits', type=int, default=1, help='Stoppbits (Standard: 1)')
    parser.add_argument('--parity', choices=['N', 'E', 'O'], default='N', help='Parität (Standard: N=None)')
    
    args = parser.parse_args()
    
    analyzer = UArtAnalyzer(args.csv_file, args.baud)
    analyzer.uart_config.data_bits = args.data_bits
    analyzer.uart_config.stop_bits = args.stop_bits
    analyzer.uart_config.parity = args.parity
    
    analyzer.analyze()
    
    # Speichere Raw-Daten
    if analyzer.raw_data:
        output_file = args.csv_file.replace('.csv', '_decoded.bin')
        with open(output_file, 'wb') as f:
            f.write(analyzer.raw_data)
        print(f"\n✓ Raw-Daten gespeichert: {output_file}")


if __name__ == '__main__':
    main()
