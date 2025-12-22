#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// ----------------- Pins -----------------
#define ThrottleSensorPin A0
#define GpsUartRxPin 4      // GPS TX → ESP32 RX
#define GpsUartTxPin 3      // GPS RX → ESP32 TX (optional)
#define GpsUartBaud 9600
#define BuzzerPin 9
#define MotionSensorSdaPin 5
#define MotionSensorSclPin 6
#define MotionSensorAddress 0x29

// ----------------- Sensoren -----------------
TwoWire I2CBNO = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, MotionSensorAddress, &I2CBNO);
HardwareSerial GPS_Serial(2);  // UART3
TinyGPSPlus gps;               // ✅ TinyGPS++

// ----------------- Timing -----------------
unsigned long lastBNO = 0, lastMagnet = 0, lastHeartbeat = 0, lastCalib = 0;
const unsigned long BNO_INTERVAL = 100;     // 10Hz
const unsigned long MAG_INTERVAL = 500;     // 2Hz  
const unsigned long HEARTBEAT_INTERVAL = 500;
const unsigned long CALIB_INTERVAL = 5000;

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

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(5000);  // Stabilisieren
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  digitalWrite(BuzzerPin, HIGH);  // Aus
  
  initBNO055();
  initGps();
  
  Serial.println("🚀 Multi-Sensor System READY!");
  Serial.println("Gravity/BNO055_X/Y/Z + GPS/TinyGPS++ + Magnet");
}

void readMagnetSensor() {
  int raw = analogRead(ThrottleSensorPin);
  float voltage = raw * (3.3 / 4095.0);
  Serial.printf("Magnet/Throttle[V]: %.2f\n", voltage);
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

void loop() {
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
  
  // Magnet Sensor (2Hz)
  if (now - lastMagnet >= MAG_INTERVAL) {
    readMagnetSensor();
    lastMagnet = now;
  }
  
  // LED Heartbeat (2Hz)
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastHeartbeat = now;
  }
}
