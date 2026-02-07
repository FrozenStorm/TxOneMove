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
BLEServer* pServer = nullptr;
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
                if (cmd == "RESET_TRIM") {
                    radioData.trimData.pitch = 0;
                    radioData.trimData.roll = 0;
                    radioData.storeTrimData();
                    return;
                }
            }
            
            // ============= SINGLE PARAMETER =============
            if (doc.containsKey("trim_pitch")) radioData.trimData.pitch = doc["trim_pitch"].as<float>();
            if (doc.containsKey("trim_roll")) radioData.trimData.roll = doc["trim_roll"].as<float>();
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
    void onConnect(BLEServer* pSrv) override {
        deviceConnected = true;
    }
    
    void onDisconnect(BLEServer* pSrv) override {
        deviceConnected = false;
        // Nach Disconnect: Advertising neu starten damit Wiederverbindung möglich ist
        if (pServer) {
            pServer->getAdvertising()->start();
        }
    }
};

class BluetoothComm : public RadioClass{
private:
    JsonDocument doc;
    JsonDocument rxDoc;
    char buffer[1024] = "";
    ServerCallback* pServerCallback = nullptr;
    RxCallback* pRxCallback = nullptr;
public:
    BluetoothComm(RadioData& newRadioData): RadioClass(newRadioData){}
    void begin(void);
    void doFunction();
    ~BluetoothComm();
};

void BluetoothComm::begin()
{
    BLEDevice::init("TxOneMove");
    BLEDevice::setMTU(517);
        
    pServer = BLEDevice::createServer();
    pServerCallback = new ServerCallback();
    pServer->setCallbacks(pServerCallback);
    
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
    pRxCallback = new RxCallback(radioData);
    pRxChar->setCallbacks(pRxCallback);
    
    pService->start();
    pServer->getAdvertising()->start();
}

void BluetoothComm::doFunction()
{
    static unsigned long lastUpdate = 0;
    if(!deviceConnected) return;

    if(millis() - lastUpdate < 100) return;
    lastUpdate = millis(); 
    
    // JSON-Dokument vor dem Senden leeren
    doc.clear();

    // Telemetrie-Daten ins JSON-Dokument schreiben
    doc["armed"] = radioData.functionData.armed ? 1 : 0;
    doc["gps_lat"] = radioData.digitalData.gpsLatitude;
    doc["gps_lon"] = radioData.digitalData.gpsLongitude;
    doc["gps_hdop"] = radioData.digitalData.gpsHdop;
    doc["gps_altitude"] = radioData.digitalData.gpsAltitudeMeters;
    doc["gps_satellites"] = radioData.digitalData.gpsSatellites;
    doc["pitch_angle"] = radioData.analogData.pitch;
    doc["roll_angle"] = radioData.analogData.roll;
    doc["pitch"] = radioData.functionData.pitch * 100;
    doc["roll"] = radioData.functionData.roll * 100;
    doc["heading"] = radioData.digitalData.heading;
    doc["tx_voltage"] = radioData.analogData.battery;
    doc["rx_voltage"] = radioData.transmitterData.receiverBatteryVoltage;
    doc["throttle_voltage"] = radioData.analogData.throttle;
    doc["trim_pitch"] = radioData.trimData.pitch * 100;
    doc["trim_roll"] = radioData.trimData.roll * 100;
    doc["throttle"] = radioData.functionData.throttle * 100;
    doc["v_speed"] = radioData.transmitterData.receiverVerticalSpeed;
    doc["altitude"] = radioData.transmitterData.receiverAltitude;
    doc["max_altitude"] = radioData.transmitterData.receiverMaxAlitude;

    // Parameter ins JSON-Dokument schreiben
    doc["ddr_pitch"] = radioData.dualRateData.pitch * 100;
    doc["ddr_roll"] = radioData.dualRateData.roll * 100;
    doc["expo_pitch"] = radioData.expoData.pitch * 100;
    doc["expo_roll"] = radioData.expoData.roll * 100;
    doc["expo_throttle"] = radioData.expoData.throttle * 100;
    doc["throttle_to_pitch"] = radioData.mixerData.throttleToPitch * 100;
    doc["throttle_min"] = radioData.analogToDigitalData.throttleLimit.min;
    doc["throttle_max"] = radioData.analogToDigitalData.throttleLimit.max;
    doc["v_max"] = radioData.transmitterData.v_sound_max;
    doc["v_min"] = radioData.transmitterData.v_sound_min;

    // JSON senden
    memset(buffer, 0, sizeof(buffer));
    serializeJson(doc, buffer, sizeof(buffer));
    pTxChar->setValue((uint8_t*)buffer, strlen(buffer));
    pTxChar->notify();
}

BluetoothComm::~BluetoothComm()
{
    if(pServerCallback) delete pServerCallback;
    if(pRxCallback) delete pRxCallback;
}