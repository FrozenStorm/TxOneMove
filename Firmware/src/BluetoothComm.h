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
    void onWrite(BLECharacteristic* pChar) override {
        std::string value = pChar->getValue();
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
    void serializeRadioData(JsonDocument& doc);
public:
    BluetoothComm(RadioData& newRadioData): RadioClass(newRadioData){}
    void begin(void);
    void doFunction() override{
        // Not used
    };
    void doFunction(TickType_t lastWakeTime);    
};

void BluetoothComm::begin()
{
    BLEDevice::init("TxOneMove");
        
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
    pRxChar->setCallbacks(new RxCallback());
    
    pService->start();
    pServer->getAdvertising()->start();
}

void BluetoothComm::serializeRadioData(JsonDocument& doc) {
        // Dynamisch alle RadioData-Felder serialisieren
        doc["longPressDurationMs"] = radioData.analogToDigitalData.longPressDurationMs;
        doc["throttleLimit_min"] = radioData.analogToDigitalData.throttleLimit.min;
        doc["throttleLimit_max"] = radioData.analogToDigitalData.throttleLimit.max;
        // Neue Felder: einfach eine Zeile hinzufügen!
    }

void BluetoothComm::doFunction(TickType_t lastWakeTime)
{
    if(!deviceConnected) return;
    JsonDocument doc;
        
    // Telemetrie-Daten (GPS, Lage, etc.) - Placeholder
    doc["gps_lat"] = 46.8;
    doc["gps_lon"] = 7.8;
    doc["pitch"] = 5.0;
    doc["roll"] = -2.0;
    doc["heading"] = 90.0;

    // RadioData serialisieren
    serializeRadioData(doc);

    // JSON senden
    char buffer[1024];
    serializeJson(doc, buffer);
    pTxChar->setValue(buffer);
    pTxChar->notify();
}