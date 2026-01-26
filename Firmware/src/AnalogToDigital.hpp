#pragma once

#include "RadioClass.hpp"
#include "esp_adc_cal.h"

#define PIN_THROTTLE            ADC1_CHANNEL_0 // GPIO01
#define PIN_VBAT                ADC1_CHANNEL_1 // GPIO02

#define PIN_ARM                 9

#define ADC_WIDTH_BIT           ADC_WIDTH_BIT_12
#define ADC_ATTEN               ADC_ATTEN_DB_12

class AnalogToDigital : public RadioClass
{
private:
    esp_adc_cal_characteristics_t   adc_chars;    
    esp_adc_cal_value_t             val_type;
    unsigned int                    changedTimeArmMs = 0;
    unsigned int                    startPressTimeArmMs = 0;
    float throttleToDigital(float value, const RadioData::AnalogToDigitalData::ThrottleLimit& limit);
    void getButton(const bool& value, unsigned int& changeTimeMs, bool& button, bool& buttonEvent);
    void getLongPress(bool& longPressEvent, unsigned int& startTimeMs, const bool& state, const bool& event);
public:
    AnalogToDigital(RadioData& newRadioData);
    void doFunction();
};

AnalogToDigital::AnalogToDigital(RadioData& newRadioData) : RadioClass(newRadioData)
{
    pinMode(PIN_ARM, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_VIBRATION, OUTPUT);

    adc1_config_width(ADC_WIDTH_BIT);
    adc1_config_channel_atten(PIN_VBAT, ADC_ATTEN);
    val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT, 1100, &adc_chars);
    switch (val_type) {
        case ESP_ADC_CAL_VAL_EFUSE_TP: Serial.println("Two Point Calibration"); break;
        case ESP_ADC_CAL_VAL_EFUSE_VREF: Serial.println("eFUSE Vref Calibration"); break;
        case ESP_ADC_CAL_VAL_DEFAULT_VREF: Serial.println("Default Calibration (1100 mV)"); break;
        case ESP_ADC_CAL_VAL_EFUSE_TP_FIT: Serial.println("ESP_ADC_CAL_VAL_EFUSE_TP_FIT"); break;
        case ESP_ADC_CAL_VAL_MAX: Serial.println("ESP_ADC_CAL_VAL_MAX"); break;
    }
}

void AnalogToDigital::doFunction()
{
    radioData.rawData.battery = radioData.rawData.battery * 0.9 + 0.1 * adc1_get_raw(PIN_VBAT);

    radioData.rawData.throttle = 0;
    for(int i = 0; i < 20; i++){
        
        radioData.rawData.throttle += adc1_get_raw(PIN_THROTTLE);
    }
    radioData.rawData.throttle = radioData.rawData.throttle / 20;
    

    radioData.analogData.battery = 2 * esp_adc_cal_raw_to_voltage(radioData.rawData.battery, &adc_chars)/1000.0 + 0.25; // Korrektur aufgrund Spannungsabfall in Leitung
    radioData.analogData.throttle = esp_adc_cal_raw_to_voltage(radioData.rawData.throttle, &adc_chars)/1000.0;

    radioData.digitalData.throttle = throttleToDigital(radioData.analogData.throttle, radioData.analogToDigitalData.throttleLimit);

    getButton(!digitalRead(PIN_ARM),changedTimeArmMs,radioData.digitalData.arm,radioData.digitalData.armEvent);
    getLongPress(radioData.digitalData.armLongPressEvent,startPressTimeArmMs,radioData.digitalData.arm,radioData.digitalData.armEvent);

    // Battery Warning Logic
    if(radioData.analogData.battery > radioData.analogToDigitalData.batteryWarningVoltage && (radioData.transmitterData.receiverBatteryVoltage > (2*radioData.analogToDigitalData.batteryWarningVoltage) || radioData.transmitterData.receiverBatteryVoltage == 0))
    { // TODO Batterie Anzahl Erkennung einbauen
        radioData.digitalData.batteryWarning = false;
    }
    else
    {
        static int slowDown = 0;
        slowDown +=1;
        if(slowDown % 10 == 0) 
        {
            radioData.digitalData.batteryWarning = !radioData.digitalData.batteryWarning;
        }
    }
}

void AnalogToDigital::getButton(const bool& value, unsigned int& changeTimeMs, bool& button, bool& buttonEvent)
{
    bool newButtonState;
    if(value == true) 
    {
        newButtonState = true;
        if(changeTimeMs == 0)
        {
            changeTimeMs = millis();
        }
    }
    else 
    {
        newButtonState = false;
        changeTimeMs = 0;
    }

    if(button == false && newButtonState == true)
    {
        if(millis() - changeTimeMs >= radioData.analogToDigitalData.longPressDurationMs/10)
        {
            buttonEvent = true;
            button = newButtonState;
        }
    }
    else 
    {
        buttonEvent = false;
        button = newButtonState;
    }
}

void AnalogToDigital::getLongPress(bool& longPressEvent, unsigned int& startTimeMs, const bool& state, const bool& event)
{
    if(event == true) 
    {
        startTimeMs = millis();
    }
    if(state == true && ((float)millis() - (float)startTimeMs) >= radioData.analogToDigitalData.longPressDurationMs)
    {
        longPressEvent = true;
        startTimeMs = millis() + 60000;       
    }
    else 
    {
        longPressEvent = false;
    }
}

float AnalogToDigital::throttleToDigital(float value, const RadioData::AnalogToDigitalData::ThrottleLimit& limit)
{
    float b = limit.min;
    float a = (limit.max - b);
    // Umwandeln von analog Bereich zu +/- 1
    if(((value - b) > 0) && (a > 0)){
        value = 2*sqrt((value - b)/a)-1;
    }
    else{
        value = -1;
    }
    // Limitieren auf digital Bereich
    limitValue(value);

    return value;
}