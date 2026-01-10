#pragma once

#include "RadioClass.hpp"

class Mixer : public RadioClass
{
private:

public:
    Mixer(RadioData& newRadioData) : RadioClass(newRadioData){}
    void doFunction();
};

void Mixer::doFunction()
{
    radioData.functionData.pitch += radioData.mixerData.throttleToPitch * ((radioData.functionData.throttle + 1) / 2);
    limitValue(radioData.functionData.pitch);
    radioData.functionData.vTailLeft = (-radioData.functionData.roll - radioData.functionData.pitch);
    limitValue(radioData.functionData.vTailLeft);
    radioData.functionData.vTailRight = (-radioData.functionData.roll + radioData.functionData.pitch);
    limitValue(radioData.functionData.vTailRight);
}