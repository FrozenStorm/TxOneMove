#pragma once

#include "RadioClass.hpp"
#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>


class SensorToDigital : public RadioClass
{
private:
    Adafruit_BNO055*    bno;
    TinyGPSPlus*        gps;

    float analogToDigital(float value, const RadioData::SensorToDigitalData::AngleLimit& limit);
    void updateOrientation();
    float accelToAngle(float accelValue);
    void limitToRange(float &value, float min, float max);
    void selectAxis();
public:
    SensorToDigital(RadioData& newRadioData, Adafruit_BNO055* newBno, TinyGPSPlus* newGps);
    void doFunction();
};

SensorToDigital::SensorToDigital(RadioData& newRadioData, Adafruit_BNO055* newBno, TinyGPSPlus* newGps):RadioClass(newRadioData)
{
    bno = newBno;
    gps = newGps;
}

void SensorToDigital::doFunction()
{
    // TODO: GPS Daten verarbeiten

    // RAW Einlesen
    imu::Vector<3> gravity = bno->getVector(Adafruit_BNO055::VECTOR_GRAVITY);
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
        Serial.println("Fehler: Pitch oder Roll ist NaN!");
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