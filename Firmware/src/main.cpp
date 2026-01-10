#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>



// FrSky Tx Version 3.4.3 41daca CE_LBT
// IrangeX 3.3.2 2d9891 ISM2G4
// Mateksys 3.3.2 2d9891 ISM2G4

// ----------------- Pins -----------------
#define ThrottleSensorPin 1
#define ArmSwitchPin 9
#define TxModuleUartTxPin 7
#define TxModuleUartRxPin 8
#define GpsUartRxPin 4      // GPS TX → ESP32 RX
#define GpsUartTxPin 3      // GPS RX → ESP32 TX (optional)
#define GpsUartBaud 9600
#define VibratorPin 44
#define LedPin 43
#define VbatPin 2
#define MotionSensorSdaPin 5
#define MotionSensorSclPin 6
#define MotionSensorAddress 0x29

// ----------------- Sensoren -----------------
TwoWire I2CBNO = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, MotionSensorAddress, &I2CBNO);
HardwareSerial GPS_Serial(2);  // UART3
TinyGPSPlus gps;               // ✅ TinyGPS++

// ----------------- CRSF -----------------
// CRSF Konstanten
#define CRSF_ADDRESS_TRANSMITTER_MODULE 0xEE
#define CRSF_ADDRESS_REMOTE_CONTROL 0xEA
#define CRSF_FRAME_RC_CHANNELS_PACKED 0x16
#define CRSF_BATTERY_TYPE 0x08 
#define CRSF_FRAME_SIZE_MAX 64
#define CHANNEL_COUNT 16
#define BITS_PER_CHANNEL 11
#define CHANNEL_DATA_LENGTH (CHANNEL_COUNT * BITS_PER_CHANNEL / 8)  // 22 Bytes

// HardwareSerial crsfSerial(1);  // UART1

uint8_t crsfFrame[CRSF_FRAME_SIZE_MAX];

// Beispiel Channel-Werte (172-1811, neutral ~992)
uint16_t channels[CHANNEL_COUNT] = {
  938, 998, 173, 992, 992, 992, 992, 992,
  992, 992, 992, 992, 992, 992, 992, 992
};


// ----------------- Timing -----------------
unsigned long lastBNO = 0, lastMagnet = 0, lastHeartbeat = 0, lastCalib = 0, lastCRSF = 0;
const unsigned long BNO_INTERVAL = 100;     // 10Hz
const unsigned long MAG_INTERVAL = 500;     // 2Hz  
const unsigned long HEARTBEAT_INTERVAL = 500;
const unsigned long CALIB_INTERVAL = 5000;
const unsigned long CRSF_INTERVAL = 5;    // 100Hz

void initGps() {
  GPS_Serial.begin(GpsUartBaud, SERIAL_8N1, GpsUartRxPin, GpsUartTxPin);
  delay(2000);
  
  Serial.println("=== ATGM336H GPS + TinyGPS++ ===");
  Serial.printf("GPS UART3: RX=%d, TX=%d, Baud=%d\n", GpsUartRxPin, GpsUartTxPin, GpsUartBaud);
  
  // TinyGPS++ PMTK-Konfig (optional)
  GPS_Serial.println("$PMTK220,1000*1F");  // 1Hz
  GPS_Serial.println("$PMTK314,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");  // GGA+RMC
  Serial.println("✓ GPS konfiguriert (TinyGPS++ ready)");
}

void initBNO055() {
  if (!I2CBNO.begin(MotionSensorSdaPin, MotionSensorSclPin)) {
    Serial.println("❌ I2C-Bus Fehler BNO055!");
    while (1) { digitalWrite(LED_BUILTIN, millis() % 200 < 100); delay(50); }
  }
  
  if (!bno.begin()) {
    Serial.println("❌ Kein BNO055 gefunden!");
    while (1) { digitalWrite(LED_BUILTIN, millis() % 200 < 100); delay(50); }
  }
  
  Serial.println("✅ BNO055 gefunden & initialisiert");
  
  // Fixed Kalibrierung (Ihre Werte)
  adafruit_bno055_offsets_t fixedCalib = {
    0, 4, -8,     // Accel X,Y,Z
    29, 366, 245, // Mag X,Y,Z  
    0, -3, -1,    // Gyro X,Y,Z
    1000, 602     // Accel/Mag Radius
  };
  
  bno.setExtCrystalUse(true);
  bno.setSensorOffsets(fixedCalib);
  bno.setMode(OPERATION_MODE_NDOF);
  
  Serial.println("✅ BNO055 Kalibrierung geladen");
  
  // Initial Status
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.printf("Initial Kalib: SYS:%d G:%d A:%d M:%d\n", sys, gyro, accel, mag);
}

void initIo() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(VibratorPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, LOW);    // Aus  
  pinMode(ArmSwitchPin, INPUT_PULLUP);
  digitalWrite(VibratorPin, LOW);  // Aus
}

void initCRSF() {
  Serial.println("=== CRSFforArduino Initialisierung ===");
  Serial.printf("CRSF UART1: RX=%d, TX=%d, Baud=400000\n", TxModuleUartRxPin, TxModuleUartTxPin);
  
  // UART1 mit Pins 7(TX), 8(RX) initialisieren: 400kbaud, 8E2
  Serial1.begin(400000, SERIAL_8N1, TxModuleUartTxPin, 37);
  pinMode(TxModuleUartTxPin, INPUT);
  }

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  delay(5000);  // Stabilisieren

  initIo();

  initBNO055();
  initGps();

  initCRSF();

  analogReadResolution(12);  // 12-Bit Auflösung (0-4095)
  
  Serial.println("🚀 Multi-Sensor System READY!");
}

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

uint8_t crc8(const uint8_t * ptr, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<length; i++)
        crc = crc8tab[crc ^ *ptr++];
    return crc;
}

void packChannels(uint8_t *output) {
  for(int i = 0; i < 16*11; i++){
    if(channels[i/11] & (0x01 << (i % 11))){
        output[i/8] |= (0x01 << (i % 8));
    }
    else{
        output[i/8] &= ~(0x01 << (i % 8));
    }
  }
}

void sendCrsfFrame() {
  // Frame aufbauen
  uint8_t frameLen = 2 + CHANNEL_DATA_LENGTH;  // (Sync + Len).... Type + Data + CRC
  crsfFrame[0] = CRSF_ADDRESS_TRANSMITTER_MODULE;
  crsfFrame[1] = frameLen;
  crsfFrame[2] = CRSF_FRAME_RC_CHANNELS_PACKED;
  
  // Channels packen
  packChannels(&crsfFrame[3]);
  
  // CRC über Addr+Type+Len+Data
  uint8_t crc = crc8(&crsfFrame[2], CHANNEL_DATA_LENGTH + 1);
  crsfFrame[frameLen + 2 - 1] = crc;
  
  // Senden
  Serial1.begin(400000, SERIAL_8N1, 37, TxModuleUartTxPin);
  Serial1.write(crsfFrame, frameLen+2);
  Serial1.flush();
  Serial1.begin(400000, SERIAL_8N1, TxModuleUartTxPin, 37);
  pinMode(TxModuleUartTxPin, INPUT);
}

void readMagnetSensor() {
  int raw = analogRead(ThrottleSensorPin);
  float voltage = raw * (3.3 / 4095.0);
  Serial.printf("ADC/Throttle[V]: %.2f\n", voltage);
  // ch1Value = map(raw, 0, 4095, 988, 2012);  // CRSF_RC_MIN=988, CRSF_RC_MAX=2012
  float b = 1.66;
  float a = (2.19 - b) / (100*100);
  float throttlePercent = sqrt((voltage - b) / a);
  Serial.printf("Channel/Throttle[%%]: %.1f\n", throttlePercent);
}

void readBatteryVoltage() {
  int raw = analogRead(VbatPin);
  float voltage = 2 * raw * (3.3 / 4095.0);
  Serial.printf("ADC/Voltage[V]: %.2f\n", voltage);
}

void readBNO055Sensor() {
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  Serial.printf("Gravity/BNO055_X[raw]: %.3f\n", gravity.x());
  Serial.printf("Gravity/BNO055_Y[raw]: %.3f\n", gravity.y());
  Serial.printf("Gravity/BNO055_Z[raw]: %.3f\n", gravity.z());
}

void readGPSTinyGPS() {
  // ✅ TinyGPS++: Einfach & Robust!
  while (GPS_Serial.available() > 0) {
    char read = GPS_Serial.read();
    // Serial.write(read);  // Optional: Rohdaten ausgeben
    if (gps.encode(read)) {
      // Neue GPS-Daten verfügbar!
      if (gps.location.isValid()) {
        // Serial.println();
        Serial.printf("GPS/TinyGPS: %.6f, %.6f | Speed: %.1f km/h | Alt: %.1f m\n",
                     gps.location.lat(), gps.location.lng(),
                     gps.speed.kmph(), gps.altitude.meters());
        Serial.printf("GPS/Sats: %d | HDOP: %.1f | FixAge: %lu\n", 
                     gps.satellites.value(), gps.hdop.hdop(), gps.location.age());
      }
    }
  }
}

void updateCRSF() {
    // channels[0] = 992 + (sin(millis() / 1000.0) * 400);  // Simuliere Sinus-Welle
    sendCrsfFrame();
}

uint8_t crsfBuffer[64];  // Max Telegram-Größe
uint8_t crsfPos = 0;
uint8_t crsfState = 0;   // 0: sync wait, 1: length, 2: type, 3: payload+CRC

void readCRSF() {
  while (Serial1.available()) {
    uint8_t byte = Serial1.read();
    switch (crsfState) {
      case 0:  // Sync Byte warten
        if (byte == CRSF_ADDRESS_REMOTE_CONTROL) {
          // Serial.println("CRSF: Sync Byte empfangen");
          crsfState = 1;
          crsfPos = 0;
          crsfBuffer[crsfPos++] = byte;
        }
        break;
        
      case 1:  // Length
        // Serial.printf("CRSF: Length Byte empfangen %d\n", byte );
        crsfBuffer[crsfPos++] = byte;
        crsfState = 2;
        break;
        
      case 2:  // Type (Adressfeld)
      // if(byte != 0x3A) Serial.printf("CRSF: Type Byte empfangen %x\n", byte );
        crsfBuffer[crsfPos++] = byte;
        if (byte == CRSF_BATTERY_TYPE) {  // Battery Sensor?
          crsfState = 3;
        } else {
          crsfState = 0;  // Nur Battery verarbeiten
        }
        break;
        
      case 3:  // Payload + CRC sammeln
        // Serial.printf("CRSF: Battery Byte empfangen %x\n", byte );
        crsfBuffer[crsfPos++] = byte;
        uint8_t len = crsfBuffer[1];
        if (crsfPos >= len + 2) {  // Vollständig: DeviceAddr(1) + Type(1) + Payload(len-4) + CRC(1) + Len(1)? Warte, Standard: Sync+Len+Type+Payload+(Len-3)+CRC
          // CRC prüfen (über Len+Type+Payload)
          // print buffer for debug as hex bytes
          for (int i = 0; i < crsfPos; i++) {
            Serial.printf("%02X ", crsfBuffer[i]);
          }
          Serial.println();
          uint8_t calc_crc = crc8(&crsfBuffer[2], len-1);
          Serial.printf("CRSF: Len: %d\n", len);
          Serial.printf("CRSF: Calculated CRC: %02X, Received CRC: %02X\n", calc_crc, crsfBuffer[crsfPos - 1]);
          if (calc_crc == crsfBuffer[crsfPos - 1]) {
            // Battery Daten extrahieren (Payload start bei Index 3)
            uint16_t voltage = (crsfBuffer[3] << 8) | crsfBuffer[4];      // mV
            uint16_t current = (crsfBuffer[5] << 8) | crsfBuffer[6];      // mA
            uint16_t consumption = (crsfBuffer[7] << 8) | crsfBuffer[8];  // mAh
            float v = voltage / 10.0;  // 0.01V steps -> Volt
            float a = current / 10.0;  // 0.01A steps -> Amp
            
            // Ausgabe über Serial
            Serial.print("Telemetry/Battery[V]: ");
            Serial.println(v, 2);
            Serial.print("Telemetry/Battery[A]: ");
            Serial.println(a, 2);
            Serial.print("Telemetry/Battery[mAh]: ");
            Serial.println(consumption);
          }
          crsfState = 0;
          crsfPos = 0;
        }
        break;
    }
  }
}

void updateIo()
{
  // Taster, Vibrator und LED testen
  if(digitalRead(ArmSwitchPin) == HIGH) {
    // Deaktiviert
    digitalWrite(VibratorPin, LOW);
    digitalWrite(LedPin, LOW);
  }
  else {
    // Aktiviert
    digitalWrite(VibratorPin, HIGH);
    digitalWrite(LedPin, HIGH);
  }
}

void loop() {
  updateIo();
  readCRSF();

  unsigned long now = millis();
  
  // // BNO055 Gravity (10Hz)
  // if (now - lastBNO >= BNO_INTERVAL) {
  //   readBNO055Sensor();
  //   lastBNO = now;
  // }

  // if (now - lastCalib > CALIB_INTERVAL) {
  //   // BNO Kalibrierung
  //   uint8_t sys, gyro, accel, mag;
  //   bno.getCalibration(&sys, &gyro, &accel, &mag);
  //   Serial.printf("Status: SYS:%d G:%d A:%d M:%d\n", sys, gyro, accel, mag);
  //   lastCalib = now;
  // }
  
  // // TinyGPS++ GPS (kontinuierlich, non-blocking!)
  // readGPSTinyGPS();
  
  // // ADC Sensor (2Hz)
  // if (now - lastMagnet >= MAG_INTERVAL) {
  //   readMagnetSensor();
  //   readBatteryVoltage();
  //   lastMagnet = now;
  // }
  
  // LED Heartbeat (2Hz)
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastHeartbeat = now;
  }

  // CRSF Update (100Hz)
  if(now - lastCRSF >= CRSF_INTERVAL) {
    updateCRSF();
    lastCRSF = now;
  }
}
