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
    JsonDocument doc;
    char buffer[512] = R"({
  "gps_lat": 47.4241,
  "gps_lon": 7.6895,
  "pitch": 0.0,
  "roll": 0.0,
  "heading": 90.0,
  "tx_voltage": 4.20,
  "rx_voltage": 3.80
})";
public:
    BluetoothComm(RadioData& newRadioData): RadioClass(newRadioData){}
    void begin(void);
    void doFunction();
};

void BluetoothComm::begin()
{
    BLEDevice::init("TxOneMove");
    BLEDevice::setMTU(200);
        
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

void BluetoothComm::doFunction()
{
    static unsigned long lastUpdate = 0;
    if(!deviceConnected) return;

    if(millis() - lastUpdate < 100) return;
    lastUpdate = millis();

    doc["gps_lat"] = radioData.digitalData.gpsLatitude;
    doc["gps_lon"] = radioData.digitalData.gpsLongitude;
    doc["pitch"] = radioData.analogData.pitch;
    doc["roll"] = radioData.analogData.roll;
    doc["heading"] = 90.0;
    doc["tx_voltage"] = radioData.analogData.battery;
    doc["rx_voltage"] = radioData.transmitterData.receiverBatteryVoltage;

    // JSON senden
    serializeJson(doc, buffer);
    // Serial.println(buffer);
    pTxChar->setValue(buffer);
    pTxChar->notify();
}