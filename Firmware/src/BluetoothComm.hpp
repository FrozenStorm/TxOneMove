#include "RadioClass.hpp"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_TX   "12345678-1234-5678-1234-56789abcdef1"
#define CHARACTERISTIC_RX   "12345678-1234-5678-1234-56789abcdef2"

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
            if (doc.containsKey("command")) {
                String cmd = doc["command"].as<String>();
                
                if (cmd == "SAVE_PARAMS") {
                    radioData.storeModelData();
                    radioData.storeGlobalData();
                    return;
                }
            }
            
            // ============= SINGLE PARAMETER =============
            if (doc.containsKey("ddr_pitch")) radioData.dualRateData.pitch = doc["ddr_pitch"].as<float>() / 100.0;
            if (doc.containsKey("ddr_roll")) radioData.dualRateData.roll = doc["ddr_roll"].as<float>() / 100.0;
            if (doc.containsKey("expo_pitch")) radioData.expoData.pitch = doc["expo_pitch"].as<float>() / 100.0;
            if (doc.containsKey("expo_roll")) radioData.expoData.roll = doc["expo_roll"].as<float>() / 100.0;
            if (doc.containsKey("expo_throttle")) radioData.expoData.throttle = doc["expo_throttle"].as<float>() / 100.0;
            if (doc.containsKey("throttle_to_pitch")) radioData.mixerData.throttleToPitch = doc["throttle_to_pitch"].as<float>() / 100.0;
            if (doc.containsKey("throttle_min")) radioData.analogToDigitalData.throttleLimit.min = doc["throttle_min"].as<float>();
            if (doc.containsKey("throttle_max")) radioData.analogToDigitalData.throttleLimit.max = doc["throttle_max"].as<float>();
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
    char buffer[512] = "";
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

    if(millis() - lastUpdate < 100) return;
    lastUpdate = millis();

    // Telemetrie-Daten ins JSON-Dokument schreiben
    doc["gps_lat"] = radioData.digitalData.gpsLatitude;
    doc["gps_lon"] = radioData.digitalData.gpsLongitude;
    doc["pitch"] = radioData.analogData.pitch;
    doc["roll"] = radioData.analogData.roll;
    doc["heading"] = radioData.dualRateData.pitch;
    doc["tx_voltage"] = radioData.analogData.battery;
    doc["rx_voltage"] = radioData.transmitterData.receiverBatteryVoltage;
    doc["throttle_voltage"] = radioData.analogData.throttle;

    // Parameter ins JSON-Dokument schreiben
    doc["ddr_pitch"] = radioData.dualRateData.pitch * 100;
    doc["ddr_roll"] = radioData.dualRateData.roll * 100;
    doc["expo_pitch"] = radioData.expoData.pitch * 100;
    doc["expo_roll"] = radioData.expoData.roll * 100;
    doc["expo_throttle"] = radioData.expoData.throttle * 100;
    doc["throttle_to_pitch"] = radioData.mixerData.throttleToPitch * 100;
    doc["throttle_min"] = radioData.analogToDigitalData.throttleLimit.min;
    doc["throttle_max"] = radioData.analogToDigitalData.throttleLimit.max;

    // JSON senden
    serializeJson(doc, buffer);
    pTxChar->setValue(buffer);
    pTxChar->notify();
}