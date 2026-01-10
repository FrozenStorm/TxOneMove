#pragma once

#include "RadioClass.hpp"
#include <math.h>

class Expo : public RadioClass
{
private:
    void calcExpo(float& value, const float expo);
public:
    Expo(RadioData& newRadioData) : RadioClass(newRadioData){}
    void doFunction();

};

void Expo::doFunction()
{
    calcExpo(radioData.functionData.throttle, radioData.expoData.throttle);
    calcExpo(radioData.functionData.roll, radioData.expoData.roll);
    calcExpo(radioData.functionData.pitch, radioData.expoData.pitch);
}

void Expo::calcExpo(float& value, const float expo)
{
    value = ((1 - expo) * value + expo * pow(value,3));
    limitValue(value);
}
