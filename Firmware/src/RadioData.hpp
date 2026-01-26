#pragma once

#include <Preferences.h>
#include <list.h>
#include "Protocol.hpp"
#include <WiFi.h>

#define MAX_NUMBER_OF_MODELS    3
#define MODEL_NAME_LENGTH       12
#define CHARACTER_SET_LENGTH    38
#define SUPPORTED_CHANNELS      8
#define CHANNEL_COUNT           16
#define CHANNEL_NEUTRAL         992
#define CHANNEL_MIN             172
#define CHANNEL_MAX             1811

#define PIN_LED                 43
#define PIN_VIBRATION           44

class RadioData
{
private:
    char modelNameString[MODEL_NAME_LENGTH+1] = "123456789_12";
    Preferences pref;

public:

    struct AnalogToDigitalData
    {
        int longPressDurationMs;
        struct ThrottleLimit{
            float min;
            float max;
        };
        ThrottleLimit throttleLimit;
        float batteryWarningVoltage;
    };
    AnalogToDigitalData analogToDigitalData;

    struct SensorToDigitalData{
        struct AngleLimit{
            float delta;
            float center;
        };
        AngleLimit angleLimitPitch;
        AngleLimit angleLimitRoll;
        float feebackUpperLimitPitch;
        float feebackLowerLimitPitch;
        float feebackUpperLimitRoll;
        float feebackLowerLimitRoll;
        unsigned long feebackDurationUs;
    };
    SensorToDigitalData sensorToDigitalData;
    

    struct ExpoData{
        float roll;
        float pitch;
        float throttle;
    };
    ExpoData expoData;

    struct DualRateData
    {
        float roll;
        float pitch;
        float throttle;
    };
    DualRateData dualRateData;

    struct TrimData
    {
        float roll;
        float pitch;
    };
    TrimData trimData;

    struct MixerData
    {
        float throttleToPitch;
    };
    MixerData mixerData;

    enum Function{ NONE, PITCH, ROLL, VTAIL_LEFT, VTAIL_RIGHT, THROTTLE, NUMBER_OF_FUNCTIONS};
    const char* functionNames[NUMBER_OF_FUNCTIONS] = {"NONE", "PITCH", "ROLL", "VTAIL_LEFT", "VTAIL_RIGHT", "THROTTLE"};

    struct FunctionToChannelData
    {
        bool invertChannel[SUPPORTED_CHANNELS];
        Function functionOnChannel[SUPPORTED_CHANNELS];
        int upperLimitChannel[SUPPORTED_CHANNELS];
        int lowerLimitChannel[SUPPORTED_CHANNELS];
    };
    FunctionToChannelData functionToChannelData;

    struct RawData
    {
        float battery = 0;
        float throttle = 0;
        float gravityX = 0;
        float gravityY = 0;
        float gravityZ = 0;
    };
    RawData rawData;

    struct AnalogData
    {
        float battery = 0; 
        float throttle = 0;
        float accelPitch = 0;
        float accelRoll = 0;
        float pitch = 0;
        float roll = 0;
    };
    AnalogData analogData;

    enum Orientation{ UNKNOWN, T_UP, T_LEFT, T_RIGHT, T_LEFT_DOWN, T_LEFT_UP, T_DOWN, NUMBER_OF_ORIENTATIONS};
    const char* orientationNames[NUMBER_OF_ORIENTATIONS] = {"UNKNOWN", "T_UP", "T_LEFT", "T_RIGHT", "T_LEFT_DOWN", "T_LEFT_UP", "T_DOWN"};

    struct DigitalData
    {
        float throttle = 0;
        float stickUpDown = 0;
        float stickLeftRight = 0;

        bool arm = 0;

        bool armEvent = 0;

        bool armLongPressEvent = 0;

        float pitch = 0;
        float roll = 0;
        float yaw = 0;
        float temperature = 0;

        Orientation orientation = UNKNOWN;

        double gpsLatitude = 0;
        double gpsLongitude = 0;
        double gpsSpeedKmph = 0;
        double gpsAltitudeMeters = 0;
        int    gpsSatellites = 0;
        double gpsHdop = 0;

        bool feedbackVibration = 0;
        bool batteryWarning = 0;
    };
    DigitalData digitalData;

    struct FunctionData
    {
        float pitch = 0;
        float roll = 0;
        float throttle = 0;
        bool armed = 0;
        float vTailLeft = 0;
        float vTailRight = 0;
    };
    FunctionData functionData;
    
    struct ChannelData
    {
        volatile int channel[CHANNEL_COUNT] = {CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL,
                                               CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL, CHANNEL_NEUTRAL};
    };
    ChannelData channelData;

    struct TransmitterData
    {
        float receiverBatteryVoltage = 0;
    };
    TransmitterData transmitterData;
    
    const char modelNameCharacters[CHARACTER_SET_LENGTH] = {' ','-','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','0','1','2','3','4','5','6','7','8','9'};
    unsigned int selectedModel;
    struct ModelData
    {
        char modelName[MODEL_NAME_LENGTH];
    };
    ModelData modelData;

    void storeModelData();
    void loadModelData();
    void storeTrimData();
    void storeGlobalData();
    void loadGlobalData();    
    void resetData();

    char* getModelName(void);
    
    RadioData(/* args */);
    ~RadioData();
};

RadioData::RadioData(/* args */)
{
 
}

RadioData::~RadioData()
{
}

char* RadioData::getModelName(void)
{
    int i;
    bool firstCharacterFound = false;
    for(i=MODEL_NAME_LENGTH-1;i>=0;i--)
    {
        if(modelData.modelName[i] == 0 && firstCharacterFound == false && i != 0){
            modelNameString[i] = '\n';
        }
        else{
            firstCharacterFound = true;
            modelNameString[i] = modelNameCharacters[modelData.modelName[i]];
        }
    }
    return modelNameString;
}

void RadioData::storeGlobalData()
{
    if(!pref.begin("Global"))
    {
        Serial.println("storeGlobalData begin error");
        return;
    }
    pref.putUInt("sm", selectedModel);

    pref.putInt("atdd.lpdm", analogToDigitalData.longPressDurationMs);
    pref.putFloat("atdd.tl.min", analogToDigitalData.throttleLimit.min);
    pref.putFloat("atdd.tl.max", analogToDigitalData.throttleLimit.max);
    pref.putFloat("atdd.bwv", analogToDigitalData.batteryWarningVoltage);

    pref.putFloat("stdd.alp.d", sensorToDigitalData.angleLimitPitch.delta);
    pref.putFloat("stdd.alr.d", sensorToDigitalData.angleLimitRoll.delta);

    pref.putFloat("stdd.fulp", sensorToDigitalData.feebackUpperLimitPitch);
    pref.putFloat("stdd.fllp", sensorToDigitalData.feebackLowerLimitPitch);
    pref.putFloat("stdd.fulr", sensorToDigitalData.feebackUpperLimitRoll);
    pref.putFloat("stdd.fllr", sensorToDigitalData.feebackLowerLimitRoll);

    pref.putULong("stdd.fdu", sensorToDigitalData.feebackDurationUs);

    Serial.printf("Global entries left = %u\n", pref.freeEntries());
    pref.end();
}

void RadioData::storeModelData()
{
    char name[15];

    sprintf(name,"Model-%d",selectedModel);
    if(!pref.begin(name))
    {
        Serial.printf("storeModelData %s begin error", name);
        return;
    }
    
    pref.putFloat("ed.rol", expoData.roll);
    pref.putFloat("ed.pit", expoData.pitch);
    pref.putFloat("ed.thr", expoData.throttle); 

    pref.putFloat("drd.rol", dualRateData.roll);
    pref.putFloat("drd.pit", dualRateData.pitch);
    pref.putFloat("drd.thr", dualRateData.throttle); 

    pref.putFloat("td.rol", trimData.roll);
    pref.putFloat("td.pit", trimData.pitch);

    pref.putFloat("md.ttp", mixerData.throttleToPitch);

    for(int i=0; i<SUPPORTED_CHANNELS; i++)
    {
        sprintf(name,"ftcd.ic.%d",i);
        pref.putBool(name, functionToChannelData.invertChannel[i]);
        sprintf(name,"ftcd.ftc.%d",i);
        pref.putInt(name, functionToChannelData.functionOnChannel[i]);
        sprintf(name,"ftcd.ulc.%d",i);
        pref.putInt(name, functionToChannelData.upperLimitChannel[i]);
        sprintf(name,"ftcd.llc.%d",i);
        pref.putInt(name, functionToChannelData.lowerLimitChannel[i]);
    }

    pref.putBytes("md.mn", modelData.modelName, sizeof(modelData.modelName));
    
    Serial.printf("Model-%d entries left = %u\n", selectedModel, pref.freeEntries());
    pref.end();
}

void RadioData::storeTrimData()
{
    char name[15];

    sprintf(name,"Model-%d",selectedModel);
    if(!pref.begin(name))
    {
        Serial.printf("storeTrimData %s begin error", name);
        return;
    }

    pref.putFloat("td.rol", trimData.roll);
    pref.putFloat("td.pit", trimData.pitch);
    
    Serial.printf("Model-%d entries left = %u\n", selectedModel, pref.freeEntries());
    pref.end();    
}

void RadioData::loadGlobalData()
{
    if(!pref.begin("Global"))
    {
        Serial.println("loadGlobalData begin error");
        return;
    }
    selectedModel = pref.getUInt("sm", 0);

    analogToDigitalData.longPressDurationMs = pref.getInt("atdd.lpdm", 600);
    analogToDigitalData.throttleLimit.min = pref.getFloat("atdd.tl.min", 1.8);
    analogToDigitalData.throttleLimit.max = pref.getFloat("atdd.tl.max", 2.3);
    analogToDigitalData.batteryWarningVoltage = pref.getFloat("atdd.bwv", 3.5);

    sensorToDigitalData.angleLimitPitch.delta = pref.getInt("stdd.alp.d", 45);
    sensorToDigitalData.angleLimitRoll.delta = pref.getInt("stdd.alr.d", 45);

    sensorToDigitalData.feebackUpperLimitPitch = pref.getFloat("stdd.fulp", 0.3);
    sensorToDigitalData.feebackLowerLimitPitch = pref.getFloat("stdd.fllp", 0.02);
    sensorToDigitalData.feebackUpperLimitRoll = pref.getFloat("stdd.fulr", 0.6);
    sensorToDigitalData.feebackLowerLimitRoll = pref.getFloat("stdd.fllr", 0.05);

    sensorToDigitalData.feebackDurationUs = pref.getULong("stdd.fdu", 100000);
    pref.end();
}

void RadioData::resetData()
{
    if(!pref.begin("Global"))
    {
        Serial.println("loadGlobalData begin error");
        return;
    }
    pref.clear();
    pref.end();
    for(int i=0; i<MAX_NUMBER_OF_MODELS; i++)
    {
        char name[15];
        sprintf(name,"Model-%d",i);
        if(!pref.begin(name))
        {
            Serial.printf("resetData %s begin error", name);
            return;
        }
        pref.clear();
        pref.end();
    }
}

void RadioData::loadModelData()
{
    char name[15];
    int readSize;

    sprintf(name,"Model-%d",selectedModel);
    if(!pref.begin(name))
    {
        Serial.printf("loadModelData %s begin error", name);
        return;
    }
    analogToDigitalData.longPressDurationMs = pref.getInt("atdd.lpdm", 600);
    
    expoData.roll = pref.getFloat("ed.rol", 0.3);
    expoData.pitch = pref.getFloat("ed.pit", 0.3);
    expoData.throttle = pref.getFloat("ed.thr", 0); 

    dualRateData.roll = pref.getFloat("drd.rol", 1);
    dualRateData.pitch = pref.getFloat("drd.pit", 1);
    dualRateData.throttle = pref.getFloat("drd.thr", 1); 

    trimData.roll = pref.getFloat("td.rol", 0);
    trimData.pitch = pref.getFloat("td.pit", 0);

    mixerData.throttleToPitch = pref.getFloat("md.ttp", 0);

    for(int i=0; i<SUPPORTED_CHANNELS; i++)
    {
        sprintf(name,"ftcd.ic.%d",i);
        functionToChannelData.invertChannel[i] = pref.getBool(name, false);
        sprintf(name,"ftcd.ftc.%d",i);
        functionToChannelData.functionOnChannel[i] = (Function)pref.getInt(name, NONE);
        sprintf(name,"ftcd.ulc.%d",i);
        functionToChannelData.upperLimitChannel[i] = pref.getInt(name, CHANNEL_MAX);
        sprintf(name,"ftcd.llc.%d",i);
        functionToChannelData.lowerLimitChannel[i] = pref.getInt(name, CHANNEL_MIN);
    }
    if(pref.isKey("ftcd.ftc.0") == false) // if no data is stored, set default values
    {
        functionToChannelData.functionOnChannel[0] = VTAIL_LEFT;
        functionToChannelData.functionOnChannel[1] = VTAIL_RIGHT;
        functionToChannelData.functionOnChannel[2] = THROTTLE;
    }

    readSize = pref.getBytes("md.mn", modelData.modelName, sizeof(modelData.modelName));
    if(readSize == 0) 
    {
        const char defaultModelName[MODEL_NAME_LENGTH] = {2,13,22,13,2,0,0,0,0,0,0,0};
        strncpy(modelData.modelName, defaultModelName, MODEL_NAME_LENGTH);
    }
    
    pref.end();
}
