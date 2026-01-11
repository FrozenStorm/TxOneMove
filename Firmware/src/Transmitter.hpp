#pragma once

#include "RadioClass.hpp"
#include "driver/uart.h"

#define PIN_TX_MODULE_RTX     7

#define CRSF_ADDRESS_TRANSMITTER_MODULE 0xEE
#define CRSF_ADDRESS_REMOTE_CONTROL 0xEA
#define CRSF_FRAME_RC_CHANNELS_PACKED 0x16
#define CRSF_BATTERY_TYPE 0x08 
#define CRSF_FRAME_SIZE_MAX 64
#define BITS_PER_CHANNEL 11
#define CHANNEL_DATA_LENGTH (CHANNEL_COUNT * BITS_PER_CHANNEL / 8)  // 22 Bytes
#define FRAME_LENGTH (CHANNEL_DATA_LENGTH + 2) // Type + Data + CRC
#define CRC_LENGTH (CHANNEL_DATA_LENGTH + 1) // Type + Data
#define TOTAL_LENGTH (FRAME_LENGTH + 2) // Sync + Length + Frame

class Transmitter : public RadioClass
{
private:
    uint8_t txData[CRSF_FRAME_SIZE_MAX];
    uint8_t rxData[CRSF_FRAME_SIZE_MAX];
    uint8_t rxPos = 0;
    uint8_t rxState = 0;   // 0: sync wait, 1: length, 2: type, 3: payload+CRC
    unsigned char crc8tab[256] = {
                                    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
                                    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
                                    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
                                    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
                                    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
                                    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
                                    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
                                    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
                                    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
                                    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
                                    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
                                    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
                                    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
                                    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
                                    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
                                    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};
    uint8_t crc8(const uint8_t * ptr, uint8_t length);
    void packChannels(uint8_t *output);
    bool sendTx(void);
    bool receiveRx(void);
public:
    Transmitter(RadioData& newRadioData): RadioClass(newRadioData) {}
    void begin();
    void doFunction();
};

void Transmitter::begin()
{
    Serial1.begin(400000, SERIAL_8N1, PIN_TX_MODULE_RTX, 37);
    pinMode(PIN_TX_MODULE_RTX, INPUT_PULLUP);
}

void Transmitter::doFunction()
{
    receiveRx();
    sendTx();
}

uint8_t Transmitter::crc8(const uint8_t * ptr, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<length; i++)
        crc = crc8tab[crc ^ *ptr++];
    return crc;
}

void Transmitter::packChannels(uint8_t *output) {
  for(int i = 0; i < CHANNEL_COUNT*BITS_PER_CHANNEL; i++){
    if(radioData.channelData.channel[i/BITS_PER_CHANNEL] & (0x01 << (i % BITS_PER_CHANNEL))){
        output[i/8] |= (0x01 << (i % 8));
    }
    else{
        output[i/8] &= ~(0x01 << (i % 8));
    }
  }
}

bool Transmitter::sendTx()
{
    // Frame aufbauen
    txData[0] = CRSF_ADDRESS_TRANSMITTER_MODULE;
    txData[1] = FRAME_LENGTH;
    txData[2] = CRSF_FRAME_RC_CHANNELS_PACKED;
    
    // Channels packen
    packChannels(&txData[3]);
    
    // CRC über Addr+Type+Len+Data
    uint8_t crc = crc8(&txData[2], CRC_LENGTH);
    txData[TOTAL_LENGTH - 1] = crc;
    
    // Senden
    Serial1.begin(400000, SERIAL_8N1, 37, PIN_TX_MODULE_RTX);
    Serial1.write(txData, TOTAL_LENGTH);
    Serial1.flush();
    Serial1.begin(400000, SERIAL_8N1, PIN_TX_MODULE_RTX, 37);
    pinMode(PIN_TX_MODULE_RTX, INPUT_PULLUP);

    return true;
}

bool Transmitter::receiveRx()
{
  while (Serial1.available()) {
    uint8_t byte = Serial1.read();
    switch (rxState) {
      case 0:  // Sync Byte warten
        if (byte == CRSF_ADDRESS_REMOTE_CONTROL) {
          // Serial.println("CRSF: Sync Byte empfangen");
          rxState = 1;
          rxPos = 0;
          rxData[rxPos++] = byte;
        }
        break;
        
      case 1:  // Length
        rxData[rxPos++] = byte;
        rxState = 2;
        break;
        
      case 2:  // Type (Adressfeld)
        rxData[rxPos++] = byte;
        if (byte == CRSF_BATTERY_TYPE) {  // Battery Sensor?
          rxState = 3;
        } else {
          rxState = 0;  // Nur Battery verarbeiten
        }
        break;
        
      case 3:  // Payload + CRC sammeln
        // Serial.printf("CRSF: Battery Byte empfangen %x\n", byte );
        rxData[rxPos++] = byte;
        uint8_t len = rxData[1];
        if (rxPos >= len + 2) {  // Vollständig: DeviceAddr(1) + Type(1) + Payload(len-4) + CRC(1) + Len(1)? Warte, Standard: Sync+Len+Type+Payload+(Len-3)+CRC
          // CRC prüfen (über Len+Type+Payload)
          uint8_t calc_crc = crc8(&rxData[2], len-1);
          if (calc_crc == rxData[rxPos - 1]) {
            // Battery Daten extrahieren (Payload start bei Index 3)
            uint16_t voltage = (rxData[3] << 8) | rxData[4];      // mV
            uint16_t current = (rxData[5] << 8) | rxData[6];      // mA
            uint16_t consumption = (rxData[7] << 8) | rxData[8];  // mAh
            radioData.transmitterData.receiverBatteryVoltage = voltage / 10.0;  // 0.01V steps -> Volt
            float a = current / 10.0;  // 0.01A steps -> Amp
          }
          rxState = 0;
          rxPos = 0;
        }
        break;
    }
  }
  return true;
}
