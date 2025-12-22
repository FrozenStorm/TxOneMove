#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <HardwareSerial.h>

#define ThrottleSensorPin A0 // Analog pin connected to the throttle sensor
#define ExpressLRSUartTxPin 7 // UART TX pin for ExpressLRS module
#define ExpressLRSUartRxPin 8 // UART RX pin for ExpressLRS module
#define GpsUartTxPin 3 // UART TX pin for GPS module
#define GpsUartRxPin 4 // UART RX pin for GPS module
#define GpsUartBaud 9600
#define BuzzerPin 9 // Pin connected to the buzzer
#define MotionSensorSdaPin 5 // I2C SDA pin for motion sensor
#define MotionSensorSclPin 6 // I2C SCL pin for motion sensor
#define MotionSensorAddress 0x29 // I2C address for the motion sensor

TwoWire I2CBNO = TwoWire(0);      // eigenen I2C-Bus anlegen (Bus 0 oder 1)
Adafruit_BNO055 bno = Adafruit_BNO055(55, MotionSensorAddress, &I2CBNO);

HardwareSerial GPS_Serial(2);  // UART3 = Serial2 auf ESP32-S3
static String gpsBuffer = "";
static bool inSentence = false;


// Globale Variablen für Non-Blocking
unsigned long lastBNO = 0;
unsigned long lastGPS = 0;
unsigned long lastMagnet = 0;
unsigned long lastHeartbeat = 0;
const unsigned long BNO_INTERVAL = 100;    // 10Hz BNO055
const unsigned long GPS_INTERVAL = 50;     // 20Hz GPS Check
const unsigned long MAG_INTERVAL = 500;    // 2Hz Magnet
const unsigned long HEARTBEAT_INTERVAL = 500; // 2Hz LED Heartbeat

void initGps(){
  GPS_Serial.begin(GpsUartBaud, SERIAL_8N1, GpsUartRxPin, GpsUartTxPin);
  delay(2000);
  Serial.println("=== ATGM336H GPS auf ESP32-S3 UART3 ===");
  Serial.printf("GPS UART3: RX=%d, TX=%d, Baud=%d\n", GpsUartRxPin, GpsUartTxPin, GpsUartBaud);
  // PMTK-Kommandos über Hardware UART3
  GPS_Serial.println("$PMTK101*32");  // Cold Start
  delay(100);
  GPS_Serial.println("$PMTK220,1000*1F");  // 1Hz Update
  delay(100);
  GPS_Serial.println("$PMTK314,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");  // GGA+RMC
  delay(100);
  GPS_Serial.println("$PMTK301,2*2E");  // Position Mode
  Serial.println("✓ GPS konfiguriert (UART3)");
  Serial.println("GPS bereit - 1Hz Positionen...");
}

void initBNO055() {
  if (!bno.begin(OPERATION_MODE_CONFIG)) {
    Serial.println("Kein BNO055 gefunden, Verkabelung/Adresse prüfen!");
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(150);
    }
  }
  Serial.println("BNO055 gefunden.");

  bno.setMode(OPERATION_MODE_CONFIG);
  delay(100);
  bno.setExtCrystalUse(true);
  
  delay(100);

  // Kalibrierung fixieren (Werte von vorheriger Kalibrierung eintragen)
  adafruit_bno055_offsets_t fixedCalib;
  fixedCalib.accel_offset_x = 0;
  fixedCalib.accel_offset_y = 4;
  fixedCalib.accel_offset_z = -8;
  fixedCalib.mag_offset_x = 29;
  fixedCalib.mag_offset_y = 366;
  fixedCalib.mag_offset_z = 245;
  fixedCalib.gyro_offset_x = 0;
  fixedCalib.gyro_offset_y = -3;
  fixedCalib.gyro_offset_z = -1;
  fixedCalib.accel_radius = 1000;
  fixedCalib.mag_radius = 602;
  bno.setSensorOffsets(fixedCalib);
  Serial.println("Set Offsets:");
  Serial.print("Accel X: "); Serial.println(fixedCalib.accel_offset_x);
  Serial.print("Accel Y: "); Serial.println(fixedCalib.accel_offset_y);
  Serial.print("Accel Z: "); Serial.println(fixedCalib.accel_offset_z);
  Serial.print("Mag X: "); Serial.println(fixedCalib.mag_offset_x);
  Serial.print("Mag Y: "); Serial.println(fixedCalib.mag_offset_y);
  Serial.print("Mag Z: "); Serial.println(fixedCalib.mag_offset_z);
  Serial.print("Gyro X: "); Serial.println(fixedCalib.gyro_offset_x);
  Serial.print("Gyro Y: "); Serial.println(fixedCalib.gyro_offset_y);
  Serial.print("Gyro Z: "); Serial.println(fixedCalib.gyro_offset_z);
  Serial.print("Accel Radius: "); Serial.println(fixedCalib.accel_radius);
  Serial.print("Mag Radius: "); Serial.println(fixedCalib.mag_radius);

  delay(100);

  bno.setMode(OPERATION_MODE_NDOF);

  delay(100);

  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  Serial.println("Get Offsets:");
  Serial.print("Accel X: "); Serial.println(newCalib.accel_offset_x);
  Serial.print("Accel Y: "); Serial.println(newCalib.accel_offset_y);
  Serial.print("Accel Z: "); Serial.println(newCalib.accel_offset_z);
  Serial.print("Mag X: "); Serial.println(newCalib.mag_offset_x);
  Serial.print("Mag Y: "); Serial.println(newCalib.mag_offset_y);
  Serial.print("Mag Z: "); Serial.println(newCalib.mag_offset_z);
  Serial.print("Gyro X: "); Serial.println(newCalib.gyro_offset_x);
  Serial.print("Gyro Y: "); Serial.println(newCalib.gyro_offset_y);
  Serial.print("Gyro Z: "); Serial.println(newCalib.gyro_offset_z);
  Serial.print("Accel Radius: "); Serial.println(newCalib.accel_radius);
  Serial.print("Mag Radius: "); Serial.println(newCalib.mag_radius);

  delay(100);
  
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print("Status: SYS:");
  Serial.print(sys);
  Serial.print(" G:");
  Serial.print(gyro);
  Serial.print(" A:");
  Serial.print(accel);
  Serial.print(" M:");
  Serial.println(mag);

  delay(100);

  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
}


void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(10000);
  Serial.println("Start Initialization...");

  // Initialize built-in LED for heartbeat
  pinMode(LED_BUILTIN, OUTPUT);

  // I2C auf deinen Pins starten
  if(!I2CBNO.begin(MotionSensorSdaPin, MotionSensorSclPin)) {
    Serial.println("Fehler beim I2C-Bus für BNO055!");
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(150);
    }
  }
  initBNO055();

  initGps();

  // Init Buzzer Pin
  pinMode(BuzzerPin, OUTPUT);
  digitalWrite(BuzzerPin, HIGH); // Buzzer aus
  
  Serial.println("Finished Initialization.");
  delay(1000);
}

// ky-035 magnet sensor reading function
void readMagnetSensor() {
    int sensorValue = analogRead(ThrottleSensorPin); // Read the sensor value

    //convert to voltage
    float voltage = sensorValue * (3.3 / 4095.0); // ESP32 ADC is 12-bit, so max value is 4095 for 3.3V

    // Print the sensor value to the Serial Monitor
    Serial.print("Default/Magnet Sensor[V]: ");
    Serial.println(voltage);
}

void readBNO055Sensor() {
    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Serial.print("Gravity/BNO055_X[raw]: ");
    Serial.println(gravity.x());
    Serial.print("Gravity/BNO055_Y[raw]: ");
    Serial.println(gravity.y());
    Serial.print("Gravity/BNO055_Z[raw]: ");
    Serial.println(gravity.z());
}

float nmeaToDecimal(String coord, String dir) {
  if (coord == "" || coord.length() < 2) return 0.0;
  
  float degrees = coord.substring(0, coord.length() - 2).toFloat();
  float minutes = coord.substring(coord.length() - 2).toFloat() / 60.0;
  float decimal = degrees + minutes;
  
  if (dir == "S" || dir == "W") decimal = -decimal;
  return decimal;
}

void parseRMC(String rmc) {
  // NMEA RMC: $GPRMC,123519,A,4807.038,N,01131.000,E,0.02,087.7,230394,004.2,W,A*47
  String fields[12];
  int fieldCount = 0;
  
  int start = rmc.indexOf('$') + 1;
  int end = 0;
  
  // Alle Felder extrahieren
  while (start < rmc.length() && fieldCount < 12) {
    end = rmc.indexOf(',', start);
    if (end == -1) end = rmc.indexOf('*', start);
    if (end == -1) end = rmc.length();
    
    fields[fieldCount] = rmc.substring(start, end);
    fields[fieldCount].trim();
    fieldCount++;
    
    start = end + 1;
  }
  
  if (fieldCount < 9) return;  // Mindestens Status, Lat, Lon, Speed
  
  // Felder zuweisen (robust!)
  String status = fields[2];           // A=active, V=void
  String lat_str = fields[3];          // 4807.038
  String lat_dir = fields[4];          // N
  String lon_str = fields[5];          // 01131.000  
  String lon_dir = fields[6];          // E
  String speed_kts = fields[7];        // 0.02
  
  float lat = nmeaToDecimal(lat_str, lat_dir);
  float lon = nmeaToDecimal(lon_str, lon_dir);
  float speed = speed_kts.toFloat() * 1.852;  // Knoten → km/h
  
  Serial.printf("RMC: %.6f,%.6f,%.1fkm/h,%s,%s\n", 
                lat, lon, speed, status.c_str(), fields[9].c_str());  // Datum
}


void parseGGA(String gga) {
  // NMEA GGA: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  String fields[15];
  int fieldCount = 0;
  
  int start = gga.indexOf('$') + 1;
  int end = 0;
  
  while (start < gga.length() && fieldCount < 15) {
    end = gga.indexOf(',', start);
    if (end == -1) end = gga.indexOf('*', start);
    if (end == -1) end = gga.length();
    
    fields[fieldCount] = gga.substring(start, end);
    fields[fieldCount].trim();
    fieldCount++;
    
    start = end + 1;
  }
  
  if (fieldCount < 10) return;
  
  String lat_str = fields[2];
  String lat_dir = fields[3];
  String lon_str = fields[4];
  String lon_dir = fields[5];
  String fix = fields[6];
  String sats = fields[7];
  String hdop = fields[8];
  String alt = fields[9];
  
  float lat = nmeaToDecimal(lat_str, lat_dir);
  float lon = nmeaToDecimal(lon_str, lon_dir);
  float altitude = alt.toFloat();
  int num_sats = sats.toInt();
  bool valid = (fix == "1");
  
  Serial.printf("GGA: %.6f,%.6f,%4.1fm,%2dsats,%.1f, FIX:%s\n", 
                lat, lon, altitude, num_sats, hdop.toFloat(), valid ? "YES" : "NO");
}



void readGPSNonBlocking() {
  static String gpsBuffer = "";
  static bool inSentence = false;
  
  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    
    if (c == '$') {
      if (inSentence && gpsBuffer.length() > 5) {
        // Vorherigen Satz parsen
        gpsBuffer.trim();
        if (gpsBuffer.startsWith("$GPGGA") || gpsBuffer.startsWith("$GNGGA")) {
          parseGGA(gpsBuffer);
        } else if (gpsBuffer.startsWith("$GPRMC") || gpsBuffer.startsWith("$GNRMC")) {
          parseRMC(gpsBuffer);
        }
      }
      gpsBuffer = "$";
      inSentence = true;
    } 
    else if (inSentence && c == '\n') {
      gpsBuffer += '\n';
      gpsBuffer.trim();
      
      if (gpsBuffer.startsWith("$GPGGA") || gpsBuffer.startsWith("$GNGGA")) {
        parseGGA(gpsBuffer);
      } else if (gpsBuffer.startsWith("$GPRMC") || gpsBuffer.startsWith("$GNRMC")) {
        parseRMC(gpsBuffer);
      }
      
      gpsBuffer = "";
      inSentence = false;
    } 
    else if (inSentence && gpsBuffer.length() < 100) {  // Buffer Overflow Schutz
      gpsBuffer += c;
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
  // Kalibrierungsstatus alle 5s
  static unsigned long lastCalib = 0;
  if (now - lastCalib > 5000) {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("Status: SYS:");
    Serial.print(sys);
    Serial.print(" G:"); Serial.print(gyro);
    Serial.print(" A:"); Serial.print(accel);
    Serial.print(" M:"); Serial.println(mag);
    lastCalib = now;
  }
  
  // GPS non-blocking Check (20Hz)
  if (now - lastGPS >= GPS_INTERVAL) {
    readGPSNonBlocking();
    lastGPS = now;
  }
  
  // Magnet Sensor (2Hz)
  if (now - lastMagnet >= MAG_INTERVAL) {
    readMagnetSensor();
    lastMagnet = now;
  }
  
  // LED Heartbeat (1Hz)
  static bool ledState = false;
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    ledState = !ledState;
    lastHeartbeat = now;
  }
}


