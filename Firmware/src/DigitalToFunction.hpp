#pragma once

#include "RadioClass.hpp"

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

    // Vibration Feedback and Battery Warning
    if(radioData.digitalData.batteryWarning == true || radioData.digitalData.feedbackVibration == true)
    {
        digitalWrite(PIN_VIBRATION, HIGH);
    }
    else
    {
        digitalWrite(PIN_VIBRATION, LOW);
    }
    
    // Led Indicator
    digitalWrite(PIN_LED, HIGH);
}