#pragma once

#include <Arduino.h>
#include <RadioData.hpp>
#include <string.h>

class RadioClass
{

protected:
    RadioData& radioData;
    void limitValue(float& value);
public:
    RadioClass(RadioData& newRadioData): radioData(newRadioData){}
    virtual void doFunction();
};

void RadioClass::limitValue(float& value)
{
    if(value > 1) value = 1;
    if(value < -1) value = -1;
}