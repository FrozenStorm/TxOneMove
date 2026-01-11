#pragma once

#include "RadioClass.hpp"
#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>

#define MOTION_SENSOR_ID      55
#define MOTION_SENSOR_ADDRESS 0x29
#define MOTION_SENSOR_BUS     0
#define PIN_MOTION_SENSOR_SCL 6
#define PIN_MOTION_SENSOR_SDA 5

#define GPS_SERIAL_NUM        2 // UART3
#define PIN_GPS_TX            3
#define PIN_GPS_RX            4
#define GPS_BAUD              9600


class SensorToDigital : public RadioClass
{
private:
    TwoWire             I2CBNO = TwoWire(MOTION_SENSOR_BUS);
    Adafruit_BNO055     bno = Adafruit_BNO055(MOTION_SENSOR_ID, MOTION_SENSOR_ADDRESS, &I2CBNO);
    TinyGPSPlus         gps;

    float analogToDigital(float value, const RadioData::SensorToDigitalData::AngleLimit& limit);
    void updateOrientation();
    float accelToAngle(float accelValue);
    void limitToRange(float &value, float min, float max);
    void selectAxis();
    void initGps();
    void initBNO055();
public:
    SensorToDigital(RadioData& newRadioData):RadioClass(newRadioData){}
    void doFunction();
    void begin();
};

void SensorToDigital::initGps()
{
    Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    delay(200);

    Serial.println("=== ATGM336H GPS + TinyGPS++ ===");
    Serial.printf("GPS UART3: RX=%d, TX=%d, Baud=%d\n", PIN_GPS_RX, PIN_GPS_TX, GPS_BAUD);

    // TinyGPS++ PMTK-Konfig (optional)
    Serial2.println("$PMTK220,1000*1F");  // 1Hz
    Serial2.println("$PMTK314,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");  // GGA+RMC
    Serial.println("✓ GPS konfiguriert (TinyGPS++ ready)");
}

void SensorToDigital::initBNO055()
{
if (!I2CBNO.begin(PIN_MOTION_SENSOR_SDA, PIN_MOTION_SENSOR_SCL)) {
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

void SensorToDigital::begin()
{
    initBNO055();
    initGps();
}

void SensorToDigital::doFunction()
{
    // TODO: GPS Daten verarbeiten
    while(Serial2.available() > 0)
    {
        char read = Serial2.read();
        // Serial.write(read);  // Optional: Rohdaten ausgeben
        if (gps.encode(read)) {
        // Neue GPS-Daten verfügbar!
        if (gps.location.isValid()) {
            radioData.digitalData.gpsLatitude = gps.location.lat();
            radioData.digitalData.gpsLongitude = gps.location.lng();
            radioData.digitalData.gpsSpeedKmph = gps.speed.kmph();
            radioData.digitalData.gpsAltitudeMeters = gps.altitude.meters();
            radioData.digitalData.gpsSatellites = gps.satellites.value();
            // Serial.printf("GPS/TinyGPS: %.6f, %.6f | Speed: %.1f km/h | Alt: %.1f m\n",
            //             gps.location.lat(), gps.location.lng(),
            //             gps.speed.kmph(), gps.altitude.meters());
            // Serial.printf("GPS/Sats: %d | HDOP: %.1f | FixAge: %lu\n", 
            //             gps.satellites.value(), gps.hdop.hdop(), gps.location.age());
        }
        }
    }

    // RAW Einlesen
    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    radioData.rawData.gravityX = gravity.x();
    radioData.rawData.gravityY = gravity.y();
    radioData.rawData.gravityZ = gravity.z();

    updateOrientation();
    selectAxis();

    radioData.analogData.pitch = accelToAngle(radioData.analogData.accelPitch);
    radioData.analogData.roll = accelToAngle(radioData.analogData.accelRoll);

    radioData.digitalData.pitch = analogToDigital(radioData.analogData.pitch, radioData.sensorToDigitalData.angleLimitPitch);
    radioData.digitalData.roll = analogToDigital(radioData.analogData.roll, radioData.sensorToDigitalData.angleLimitRoll);

    if(isnan(radioData.digitalData.pitch) ||  isnan(radioData.digitalData.roll)) 
    {
        radioData.analogData.pitch = 0;
        radioData.analogData.roll = 0;
        radioData.digitalData.pitch = 0;
        radioData.digitalData.roll = 0;
        // Serial.println("Fehler: Pitch oder Roll ist NaN!");
        return;
    }
}

void SensorToDigital::selectAxis()
{
    switch (radioData.digitalData.orientation)
    {
    case RadioData::Orientation::T_UP:
        radioData.analogData.accelPitch = radioData.rawData.gravityY;
        radioData.analogData.accelRoll = radioData.rawData.gravityZ;
        break;
    case RadioData::Orientation::T_DOWN:
        radioData.analogData.accelPitch = radioData.rawData.gravityY;
        radioData.analogData.accelRoll = -radioData.rawData.gravityZ;
        break;
    case RadioData::Orientation::T_LEFT:
        radioData.analogData.accelPitch = radioData.rawData.gravityY;
        radioData.analogData.accelRoll = -radioData.rawData.gravityX;
        break;
    case RadioData::Orientation::T_RIGHT:
        radioData.analogData.accelPitch = radioData.rawData.gravityY;
        radioData.analogData.accelRoll = radioData.rawData.gravityX;
        break;
    case RadioData::Orientation::T_LEFT_DOWN:
        radioData.analogData.accelPitch = radioData.rawData.gravityZ;
        radioData.analogData.accelRoll = -radioData.rawData.gravityX;
        break;
    case RadioData::Orientation::T_LEFT_UP:
        radioData.analogData.accelPitch = -radioData.rawData.gravityZ;
        radioData.analogData.accelRoll = -radioData.rawData.gravityX;
        break;
    case RadioData::Orientation::UNKNOWN:
        radioData.analogData.accelPitch = 0;
        radioData.analogData.accelRoll = 0;
        break;
    }
}

void SensorToDigital::updateOrientation()
{
    if(radioData.functionData.armed == false) 
    {
        if(abs(radioData.rawData.gravityX) > abs(radioData.rawData.gravityY) && abs(radioData.rawData.gravityX) > abs(radioData.rawData.gravityZ))
        {
            if(radioData.rawData.gravityX > 0)
            {
                radioData.digitalData.orientation = RadioData::Orientation::T_DOWN;
            }
            else
            {
                radioData.digitalData.orientation = RadioData::Orientation::T_UP;
            }
        }
        else if(abs(radioData.rawData.gravityY) > abs(radioData.rawData.gravityZ))
        {
            if(radioData.rawData.gravityZ > 0)
            {
                radioData.digitalData.orientation = RadioData::Orientation::T_LEFT_DOWN;
            }
            else
            {
                radioData.digitalData.orientation = RadioData::Orientation::T_LEFT_UP;
            }
        }
        else
        {
            if(radioData.rawData.gravityZ > 0)
            {
                radioData.digitalData.orientation = RadioData::Orientation::T_RIGHT;
            }
            else
            {
                radioData.digitalData.orientation = RadioData::Orientation::T_LEFT;
            }
        }
    }
}
void SensorToDigital::limitToRange(float &value, float min, float max)
{
    if(value < min)
    {
        value = min;
    }
    if(value > max)
    {
        value = max;
    }
}

float SensorToDigital::accelToAngle(float accelValue)
{
    // Begrenzen des Accel Wertes auf den Bereich -9.81 bis 9.81
    limitToRange(accelValue, -9.81, 9.81);
    // Umwandeln von Accel Wert in Winkel in Grad
    float angle = asin(accelValue / 9.81) * (180.0 / M_PI);
    return angle;
}

float SensorToDigital::analogToDigital(float value, const RadioData::SensorToDigitalData::AngleLimit& limit)
{
    // Offset wegrechnen
    value = value - limit.center;
    // Umwandeln von analog Bereich zu +/- 1
    value = value / limit.delta;
    // Limitieren auf digital Bereich
    limitValue(value);
    return value;
}