#pragma once

#include "RadioClass.hpp"

#define BATTERY_WARNING_VOLTAGE 3.5 // Voltage at which the TX module will warn about low battery

class DigitalToFunction : public RadioClass
{
private:
public:
    DigitalToFunction(RadioData& newRadioData) : RadioClass(newRadioData){}
    void doFunction() override;
};

void DigitalToFunction::doFunction()
{
    // Trim
    if(radioData.digitalData.armLongPressEvent == 1 && radioData.functionData.armed == 0){ // Nur wenn aus armed mode heraus mit long press gegangen wird, wird getrimmt
        if(radioData.digitalData.pitch != NAN && radioData.digitalData.roll != NAN && radioData.analogData.pitch != NAN && radioData.analogData.roll != NAN){
            radioData.trimData.pitch = radioData.digitalData.pitch;
            radioData.trimData.roll = radioData.digitalData.roll;
            radioData.sensorToDigitalData.angleLimitPitch.center = radioData.analogData.pitch;
            radioData.sensorToDigitalData.angleLimitRoll.center = radioData.analogData.roll;
            radioData.storeTrimData();
        }
    }

    // Arm Switch
    if(radioData.digitalData.armEvent == 1)
    {
        radioData.functionData.armed = !radioData.functionData.armed;
        if(radioData.functionData.armed == 1)
        {
            radioData.sensorToDigitalData.angleLimitPitch.center = radioData.analogData.pitch;
            radioData.sensorToDigitalData.angleLimitRoll.center = radioData.analogData.roll;
        }
    }
    if(radioData.functionData.armed == 1)
    {
        radioData.functionData.pitch = radioData.digitalData.pitch;
        radioData.functionData.roll = radioData.digitalData.roll;
        radioData.functionData.throttle = radioData.digitalData.throttle;
    }
    else
    {
        radioData.functionData.pitch = 0;
        radioData.functionData.roll = 0;
        radioData.functionData.throttle = -1;
    } 

    // Batterie
    if(radioData.analogData.battery > BATTERY_WARNING_VOLTAGE && (radioData.transmitterData.receiverBatteryVoltage > (2*BATTERY_WARNING_VOLTAGE) || radioData.transmitterData.receiverBatteryVoltage == 0))
    {
        digitalWrite(PIN_LED,1);
        digitalWrite(PIN_VIBRATION,0);
    }
    else
    {
        static int slowDown = 0;
        slowDown +=1;
        if(slowDown % 10 == 0) 
        {
            digitalWrite(PIN_LED,digitalRead(PIN_LED) == 1 ? 0 : 1);
            digitalWrite(PIN_VIBRATION,digitalRead(PIN_VIBRATION) == 1 ? 0 : 1);
        }
    }
}