#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>

#define ThrottleSensorPin A0 // Analog pin connected to the throttle sensor
#define ExpressLRSUartTxPin 7 // UART TX pin for ExpressLRS module
#define ExpressLRSUartRxPin 8 // UART RX pin for ExpressLRS module
#define GpsUartTxPin 3 // UART TX pin for GPS module
#define GpsUartRxPin 4 // UART RX pin for GPS module
#define BuzzerPin 9 // Pin connected to the buzzer
#define MotionSensorSdaPin 5 // I2C SDA pin for motion sensor
#define MotionSensorSclPin 6 // I2C SCL pin for motion sensor
#define MotionSensorAddress 0x29 // I2C address for the motion sensor

TwoWire I2CBNO = TwoWire(0);      // eigenen I2C-Bus anlegen (Bus 0 oder 1)
Adafruit_BNO055 bno = Adafruit_BNO055(55, MotionSensorAddress, &I2CBNO);

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
    // uint8_t sys, gyro, accel, mag;
    // bno.getCalibration(&sys, &gyro, &accel, &mag);
    // Serial.print("Status: SYS:");
    // Serial.print(sys);
    // Serial.print(" G:");
    // Serial.print(gyro);
    // Serial.print(" A:");
    // Serial.print(accel);
    // Serial.print(" M:");
    // Serial.println(mag);

    // imu::Quaternion quat = bno.getQuat();
    // Serial.print("Quaternion/BNO055_W[raw]: ");
    // Serial.println(quat.w());
    // Serial.print("Quaternion/BNO055_X[raw]: ");
    // Serial.println(quat.x());
    // Serial.print("Quaternion/BNO055_Y[raw]: ");
    // Serial.println(quat.y());
    // Serial.print("Quaternion/BNO055_Z[raw]: ");
    // Serial.println(quat.z());

    // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // Serial.print("Euler/BNO055_Heading[raw]: ");
    // Serial.println(euler.x());
    // Serial.print("Euler/BNO055_Roll[raw]: ");
    // Serial.println(euler.y());
    // Serial.print("Euler/BNO055_Pitch[raw]: ");
    // Serial.println(euler.z());

    // imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // Serial.print("LinearAccel/BNO055_X[raw]: ");
    // Serial.println(linAccel.x());
    // Serial.print("LinearAccel/BNO055_Y[raw]: ");
    // Serial.println(linAccel.y());
    // Serial.print("LinearAccel/BNO055_Z[raw]: ");
    // Serial.println(linAccel.z());

    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Serial.print("Gravity/BNO055_X[raw]: ");
    Serial.println(gravity.x());
    Serial.print("Gravity/BNO055_Y[raw]: ");
    Serial.println(gravity.y());
    Serial.print("Gravity/BNO055_Z[raw]: ");
    Serial.println(gravity.z());
}

void loop() {
  readMagnetSensor();
  // displaySensorDetails();
  readBNO055Sensor();

  delay(100); // wait for 20 milliseconds before next reading

  // LED heartbeat: toggle once per second
  static bool led = false;
  static unsigned hb = 0;
  if ((++hb % 10) == 0) {
    led = !led;
    digitalWrite(LED_BUILTIN, led ? HIGH : LOW);
  }
}


