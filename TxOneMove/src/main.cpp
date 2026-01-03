#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>




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
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xEE
#define CRSF_FRAME_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAME_SIZE_MAX 64
#define CHANNEL_COUNT 16
#define BITS_PER_CHANNEL 11
#define CHANNEL_DATA_LENGTH (CHANNEL_COUNT * BITS_PER_CHANNEL / 8)  // 22 Bytes

HardwareSerial crsfSerial(1);  // UART1

uint8_t crsfFrame[CRSF_FRAME_SIZE_MAX];

// Beispiel Channel-Werte (172-1811, neutral ~992)
uint16_t channels[CHANNEL_COUNT] = {
  992, 992, 992, 992, 992, 992, 992, 992,
  992, 992, 992, 992, 992, 992, 992, 992
};


// ----------------- Timing -----------------
unsigned long lastBNO = 0, lastMagnet = 0, lastHeartbeat = 0, lastCalib = 0, lastCRSF = 0;
const unsigned long BNO_INTERVAL = 100;     // 10Hz
const unsigned long MAG_INTERVAL = 500;     // 2Hz  
const unsigned long HEARTBEAT_INTERVAL = 500;
const unsigned long CALIB_INTERVAL = 5000;
const unsigned long CRSF_INTERVAL = 20;    // 50Hz

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
  crsfSerial.begin(400000, SERIAL_8E2, TxModuleUartRxPin, TxModuleUartTxPin);
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
  Serial.println("Gravity/BNO055_X/Y/Z + GPS/TinyGPS++ + Magnet");
}

uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i) {
    uint8_t currByte = data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (((crc ^ currByte) & 0x80) != 0) {
        crc = (crc << 1) ^ 0xD5;  // CRC-8 poly=0xD5
      } else {
        crc = (crc << 1);
      }
      currByte <<= 1;
    }
  }
  return crc;
}

void packChannels(uint8_t *output) {
  uint32_t bitBuffer = 0;
  uint8_t bitCount = 0;
  uint8_t bytePos = 0;

  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ++ch) {
    uint16_t channelValue = constrain(channels[ch], 172, 1811);
    bitBuffer |= ((uint32_t)channelValue << bitCount);
    
    bitCount += BITS_PER_CHANNEL;
    while (bitCount >= 8) {
      output[bytePos++] = (bitBuffer & 0xFF);
      bitBuffer >>= 8;
      bitCount -= 8;
    }
  }
  
  // Verbleibende Bits flushen
  if (bitCount > 0) {
    output[bytePos++] = (bitBuffer & 0xFF);
  }
}

void sendCrsfFrame() {
  // Frame aufbauen
  uint8_t frameLen = 4 + CHANNEL_DATA_LENGTH;  // Sync + Addr + Type + Len + Data
  crsfFrame[0] = CRSF_SYNC_BYTE;
  crsfFrame[1] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  crsfFrame[2] = CRSF_FRAME_RC_CHANNELS_PACKED;
  crsfFrame[3] = CHANNEL_DATA_LENGTH;
  
  // Channels packen
  packChannels(&crsfFrame[4]);
  
  // CRC über Addr+Type+Len+Data
  uint8_t crc = crc8(&crsfFrame[1], frameLen - 2);
  crsfFrame[frameLen - 1] = crc;
  
  // Senden
  crsfSerial.write(crsfFrame, frameLen);
}

void readMagnetSensor() {
  int raw = analogRead(ThrottleSensorPin);
  float voltage = raw * (3.3 / 4095.0);
  Serial.printf("ADC/Throttle[V]: %.2f\n", voltage);
  // ch1Value = map(raw, 0, 4095, 988, 2012);  // CRSF_RC_MIN=988, CRSF_RC_MAX=2012
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
    channels[0] = 992 + (sin(millis() / 1000.0) * 400);  // Simuliere Sinus-Welle
    sendCrsfFrame();
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

  unsigned long now = millis();
  
  // BNO055 Gravity (10Hz)
  if (now - lastBNO >= BNO_INTERVAL) {
    readBNO055Sensor();
    lastBNO = now;
  }

  if (now - lastCalib > CALIB_INTERVAL) {
    // BNO Kalibrierung
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.printf("Status: SYS:%d G:%d A:%d M:%d\n", sys, gyro, accel, mag);
    lastCalib = now;
  }
  
  // TinyGPS++ GPS (kontinuierlich, non-blocking!)
  readGPSTinyGPS();
  
  // ADC Sensor (2Hz)
  if (now - lastMagnet >= MAG_INTERVAL) {
    readMagnetSensor();
    readBatteryVoltage();
    lastMagnet = now;
  }
  
  // LED Heartbeat (2Hz)
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastHeartbeat = now;
  }

  // CRSF Update (50Hz)
  if(now - lastCRSF >= CRSF_INTERVAL) {
    updateCRSF();
    lastCRSF = now;
  }
}
