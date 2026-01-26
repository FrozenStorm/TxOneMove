#include "RadioClass.hpp"
#include "RadioData.hpp"

#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_TX "12345678-1234-5678-1234-56789abcdef1"
#define CHARACTERISTIC_RX "12345678-1234-5678-1234-56789abcdef2"

BLECharacteristic* pTxChar = nullptr;
BLECharacteristic* pRxChar = nullptr;
bool deviceConnected = false;

class RxCallback : public BLECharacteristicCallbacks {
    JsonDocument doc;
    RadioData& radioData;

public:
    RxCallback(RadioData& newRadioData) : radioData(newRadioData) , BLECharacteristicCallbacks(){}

    void onWrite(BLECharacteristic* pChar) override {
        std::string rxValue = pChar->getValue();
        
        if (rxValue.length() > 0) {
            deserializeJson(doc, rxValue.c_str());

            // ============= COMMAND HANDLING =============
            if (doc["command"].is<const char*>()) {
                String cmd = doc["command"].as<const char*>();
                
                if (cmd == "SAVE_PARAMS") {
                    radioData.storeModelData();
                    radioData.storeGlobalData();
                    return;
                }
                
                if (cmd == "RESET_TRIM") {
                    radioData.trimData.pitch = 0;
                    radioData.trimData.roll = 0;
                    radioData.storeTrimData();
                    return;
                }
            }

            // ============= SINGLE PARAMETER =============
            if (doc["trim_pitch"].is<double>()) radioData.trimData.pitch = doc["trim_pitch"].as<double>();
            if (doc["trim_roll"].is<double>()) radioData.trimData.roll = doc["trim_roll"].as<double>();
            if (doc["ddr_pitch"].is<double>()) radioData.dualRateData.pitch = doc["ddr_pitch"].as<double>() / 100.0;
            if (doc["ddr_roll"].is<double>()) radioData.dualRateData.roll = doc["ddr_roll"].as<double>() / 100.0;
            if (doc["expo_pitch"].is<double>()) radioData.expoData.pitch = doc["expo_pitch"].as<double>() / 100.0;
            if (doc["expo_roll"].is<double>()) radioData.expoData.roll = doc["expo_roll"].as<double>() / 100.0;
            if (doc["expo_throttle"].is<double>()) radioData.expoData.throttle = doc["expo_throttle"].as<double>() / 100.0;
            if (doc["throttle_to_pitch"].is<double>()) radioData.mixerData.throttleToPitch = doc["throttle_to_pitch"].as<double>() / 100.0;
            if (doc["throttle_min"].is<int>()) radioData.analogToDigitalData.throttleLimit.min = doc["throttle_min"].as<int>();
            if (doc["throttle_max"].is<int>()) radioData.analogToDigitalData.throttleLimit.max = doc["throttle_max"].as<int>();

            // ============= MODEL NAME =============
            if (doc["model_name"].is<const char*>()) {
                const char* newName = doc["model_name"].as<const char*>();
                int length = strlen(newName);
                if (length <= MODEL_NAME_LENGTH) {
                    strncpy((char*)radioData.modelData.modelName, newName, MODEL_NAME_LENGTH);
                }
            }

            // ============= FUNCTION TO CHANNEL MAPPING =============
            if (doc["ftc"].is<JsonObject>()) {
                JsonObject ftc = doc["ftc"];
                
                // Channel-based configuration
                for (int ch = 0; ch < SUPPORTED_CHANNELS; ch++) {
                    String chKey = "ch" + String(ch);
                    
                    if (ftc[chKey].is<JsonObject>()) {
                        JsonObject chConfig = ftc[chKey];
                        
                        if (chConfig["invert"].is<bool>()) {
                            radioData.functionToChannelData.invertChannel[ch] = chConfig["invert"].as<bool>();
                        }
                        
                        if (chConfig["func"].is<int>()) {
                            int funcIdx = chConfig["func"].as<int>();
                            if (funcIdx >= 0 && funcIdx < RadioData::NUMBER_OF_FUNCTIONS) {
                                radioData.functionToChannelData.functionOnChannel[ch] = (RadioData::Function)funcIdx;
                            }
                        }
                        
                        if (chConfig["upper"].is<int>()) {
                            radioData.functionToChannelData.upperLimitChannel[ch] = chConfig["upper"].as<int>();
                        }
                        
                        if (chConfig["lower"].is<int>()) {
                            radioData.functionToChannelData.lowerLimitChannel[ch] = chConfig["lower"].as<int>();
                        }
                    }
                }
            }

            // ============= ORIENTATION =============
            if (doc["orientation"].is<int>()) {
                int oriIdx = doc["orientation"].as<int>();
                if (oriIdx >= 0 && oriIdx < RadioData::NUMBER_OF_ORIENTATIONS) {
                    radioData.digitalData.orientation = (RadioData::Orientation)oriIdx;
                }
            }
        }
    }
};

class ServerCallback : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
    }
};

class BluetoothComm : public RadioClass{
private:
    JsonDocument doc;
    char buffer[1024] = "";

public:
    BluetoothComm(RadioData& newRadioData): RadioClass(newRadioData){}

    void begin(void);
    void doFunction();
};

void BluetoothComm::begin()
{
    BLEDevice::init("TxOneMove");
    BLEDevice::setMTU(517);
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallback());
    BLEService* pService = pServer->createService(SERVICE_UUID);
    
    pTxChar = pService->createCharacteristic(
        CHARACTERISTIC_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxChar->addDescriptor(new BLE2902());
    
    pRxChar = pService->createCharacteristic(
        CHARACTERISTIC_RX,
        BLECharacteristic::PROPERTY_WRITE
    );
    pRxChar->setCallbacks(new RxCallback(radioData));
    
    pService->start();
    pServer->getAdvertising()->start();
}

void BluetoothComm::doFunction()
{
    static unsigned long lastUpdate = 0;
    
    if(!deviceConnected) return;
    if(millis() - lastUpdate < 50) return;
    
    lastUpdate = millis();
    
    // Clear document
    doc.clear();

    // ============= TELEMETRIE-DATEN =============
    doc["armed"] = radioData.functionData.armed ? 1 : 0;
    doc["gps_lat"] = radioData.digitalData.gpsLatitude;
    doc["gps_lon"] = radioData.digitalData.gpsLongitude;
    doc["gps_speed"] = radioData.digitalData.gpsSpeedKmph;
    doc["gps_altitude"] = radioData.digitalData.gpsAltitudeMeters;
    doc["gps_satellites"] = radioData.digitalData.gpsSatellites;
    
    doc["pitch_angle"] = radioData.analogData.pitch;
    doc["roll_angle"] = radioData.analogData.roll;
    doc["pitch"] = radioData.functionData.pitch * 100;
    doc["roll"] = radioData.functionData.roll * 100;
    doc["heading"] = radioData.dualRateData.pitch;
    
    doc["tx_voltage"] = radioData.analogData.battery;
    doc["rx_voltage"] = radioData.transmitterData.receiverBatteryVoltage;
    doc["throttle_voltage"] = radioData.analogData.throttle;
    
    doc["trim_pitch"] = radioData.trimData.pitch * 100;
    doc["trim_roll"] = radioData.trimData.roll * 100;
    doc["throttle"] = radioData.functionData.throttle * 100;

    // ============= PARAMETER =============
    doc["ddr_pitch"] = radioData.dualRateData.pitch * 100;
    doc["ddr_roll"] = radioData.dualRateData.roll * 100;
    doc["expo_pitch"] = radioData.expoData.pitch * 100;
    doc["expo_roll"] = radioData.expoData.roll * 100;
    doc["expo_throttle"] = radioData.expoData.throttle * 100;
    doc["throttle_to_pitch"] = radioData.mixerData.throttleToPitch * 100;
    doc["throttle_min"] = radioData.analogToDigitalData.throttleLimit.min;
    doc["throttle_max"] = radioData.analogToDigitalData.throttleLimit.max;

    // ============= MODEL NAME =============
    doc["model_name"] = radioData.getModelName();

    // ============= FUNCTION TO CHANNEL MAPPING =============
    JsonObject ftc = doc["ftc"].to<JsonObject>();
    for (int ch = 0; ch < SUPPORTED_CHANNELS; ch++) {
        String chKey = "ch" + String(ch);
        JsonObject chConfig = ftc[chKey].to<JsonObject>();
        
        chConfig["invert"] = radioData.functionToChannelData.invertChannel[ch];
        chConfig["func"] = (int)radioData.functionToChannelData.functionOnChannel[ch];
        chConfig["upper"] = radioData.functionToChannelData.upperLimitChannel[ch];
        chConfig["lower"] = radioData.functionToChannelData.lowerLimitChannel[ch];
    }

    // ============= FUNCTION NAMES =============
    JsonArray funcNames = doc["func_names"].to<JsonArray>();
    for (int i = 0; i < RadioData::NUMBER_OF_FUNCTIONS; i++) {
        funcNames.add(radioData.functionNames[i]);
    }

    // ============= ORIENTATION =============
    doc["orientation"] = (int)radioData.digitalData.orientation;
    
    // ============= ORIENTATION NAMES =============
    JsonArray oriNames = doc["orientation_names"].to<JsonArray>();
    for (int i = 0; i < RadioData::NUMBER_OF_ORIENTATIONS; i++) {
        oriNames.add(radioData.orientationNames[i]);
    }

    // ============= CHANNEL DATA =============
    JsonArray channels = doc["channels"].to<JsonArray>();
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        channels.add(radioData.channelData.channel[i]);
    }

    // ============= V-TAIL DATA =============
    doc["vtail_left"] = radioData.functionData.vTailLeft * 100;
    doc["vtail_right"] = radioData.functionData.vTailRight * 100;

    // JSON senden
    serializeJson(doc, buffer);
    pTxChar->setValue(buffer);
    pTxChar->notify();
}