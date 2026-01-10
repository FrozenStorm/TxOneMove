#pragma once

#include "RadioClass.hpp"

class DualRate : public RadioClass
{
private:
    void calcRate(float& value, float rate);
public:
    DualRate(RadioData& newRadioData) : RadioClass(newRadioData){}
    void doFunction();
};

void DualRate::doFunction()
{
    calcRate(radioData.functionData.throttle, radioData.dualRateData.throttle);
    calcRate(radioData.functionData.roll, radioData.dualRateData.roll);
    calcRate(radioData.functionData.pitch, radioData.dualRateData.pitch);
}

void DualRate::calcRate(float& value, float rate)
{
    if(value > rate) value = rate;
    if(value < -rate) value = -rate;
}
