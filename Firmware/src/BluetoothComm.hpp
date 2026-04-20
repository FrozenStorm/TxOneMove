#include "RadioClass.hpp"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <cstring>

#define SERVICE_UUID             "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_DATA      "12345678-1234-5678-1234-56789abcde00"
#define CHARACTERISTIC_COMMAND    "12345678-1234-5678-1234-56789abcde05"

// Command IDs
#define CMD_SAVE_PARAMS     0x01
#define CMD_RESET_TRIM      0x02
#define CMD_UPDATE_PARAM    0x03

// Parameter IDs for UPDATE_PARAM
#define PARAM_TRIM_PITCH           0
#define PARAM_TRIM_ROLL            1
#define PARAM_DDR_PITCH            2
#define PARAM_DDR_ROLL             3
#define PARAM_EXPO_PITCH           4
#define PARAM_EXPO_ROLL            5
#define PARAM_EXPO_THROTTLE        6
#define PARAM_THROTTLE_TO_PITCH    7
#define PARAM_THROTTLE_MIN         8
#define PARAM_THROTTLE_MAX         9
#define PARAM_V_MIN                10
#define PARAM_V_MAX                11

BLECharacteristic* pDataChar = nullptr;
BLECharacteristic* pCommandChar = nullptr;
BLEServer* pServer = nullptr;
bool deviceConnected = false;
uint8_t cmdId = 0;
uint8_t paramId = 0;
float paramValue = 0.0f;

// Helper functions for binary serialization (little-endian)
inline void writeFloat(uint8_t* buffer, int& offset, float value) {
    std::memcpy(buffer + offset, &value, sizeof(float));
    offset += sizeof(float);
}

inline void writeDouble(uint8_t* buffer, int& offset, double value) {
    std::memcpy(buffer + offset, &value, sizeof(double));
    offset += sizeof(double);
}

inline void writeUint16(uint8_t* buffer, int& offset, uint16_t value) {
    std::memcpy(buffer + offset, &value, sizeof(uint16_t));
    offset += sizeof(uint16_t);
}

inline void writeUint8(uint8_t* buffer, int& offset, uint8_t value) {
    buffer[offset++] = value;
}

inline float readFloat(const uint8_t* buffer, int& offset) {
    float value;
    std::memcpy(&value, buffer + offset, sizeof(float));
    offset += sizeof(float);
    return value;
}

inline double readDouble(const uint8_t* buffer, int& offset) {
    double value;
    std::memcpy(&value, buffer + offset, sizeof(double));
    offset += sizeof(double);
    return value;
}

inline uint16_t readUint16(const uint8_t* buffer, int& offset) {
    uint16_t value;
    std::memcpy(&value, buffer + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    return value;
}

inline uint8_t readUint8(const uint8_t* buffer, int& offset) {
    return buffer[offset++];
}

class RxCallback : public BLECharacteristicCallbacks {
private:
    RadioData& radioData;
public:
    RxCallback(RadioData& newRadioData) : radioData(newRadioData), BLECharacteristicCallbacks() {}
    void onWrite(BLECharacteristic* pChar) override {
        std::string rxValue = pChar->getValue();
        
        const uint8_t* data = (const uint8_t*)rxValue.c_str();
        int offset = 0; 
        cmdId = readUint8(data, offset);

        if(cmdId == CMD_UPDATE_PARAM){
            paramId = readUint8(data, offset);
            paramValue = readFloat(data, offset);
        }
        // Serial.printf("Received command: %d, paramId: %d, value: %.2f\n", cmdId, paramId, paramValue);
    }
};

class ServerCallback : public BLEServerCallbacks {
    void onConnect(BLEServer* pSrv) override {
        deviceConnected = true;
        //Serial.println("BLE Client connected");
    }
    
    void onDisconnect(BLEServer* pSrv) override {
        deviceConnected = false;
        //Serial.println("BLE Client disconnected");
        if (pServer) {
            pServer->getAdvertising()->start();
            //Serial.println("Advertising restarted");
        }
    }
};

class BluetoothComm : public RadioClass {
private:
    ServerCallback* pServerCallback = nullptr;
    RxCallback* pRxCallback = nullptr;
public:
    BluetoothComm(RadioData& newRadioData) : RadioClass(newRadioData) {}
    void begin(void);
    void doFunction();
    ~BluetoothComm();
};

void BluetoothComm::begin()
{
    BLEDevice::init("TxOneMove");
    BLEDevice::setMTU(256);  // Reduziert von 517 auf 256 für Stabilität
    // Serial.println("BLE init done");
        
    pServer = BLEDevice::createServer();
    pServerCallback = new ServerCallback();
    pServer->setCallbacks(pServerCallback);
    // Serial.println("BLE server created");
    
    BLEService* pService = pServer->createService(SERVICE_UUID);
    // Serial.println("BLE service created");
    
    pDataChar = pService->createCharacteristic(
        CHARACTERISTIC_DATA,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pDataChar->addDescriptor(new BLE2902());
    // Serial.println("Data Char created");

    pCommandChar = pService->createCharacteristic(
        CHARACTERISTIC_COMMAND,
        BLECharacteristic::PROPERTY_WRITE
    );
    pRxCallback = new RxCallback(radioData);
    pCommandChar->setCallbacks(pRxCallback);
    // Serial.println("Command Char created");

    pService->start();
    // Serial.println("Service started");
    pServer->getAdvertising()->start();
    // Serial.println("Advertising started");
}


void BluetoothComm::doFunction()
{
    static unsigned long lastUpdate = 0;
    if (!deviceConnected) return;

    noInterrupts(); // Verhindert Unterbrechungen während der Verarbeitung von Befehlen und Senden von Daten

    switch (cmdId) {
        case CMD_SAVE_PARAMS:
            cmdId = 0; // Reset command ID after processing
            interrupts();
            radioData.storeModelData();
            radioData.storeGlobalData();
            // Serial.println("BLE: SAVE_PARAMS executed");
            break;
            
        case CMD_RESET_TRIM:
            cmdId = 0; // Reset command ID after processing
            interrupts();
            radioData.trimData.pitch = 0;
            radioData.trimData.roll = 0;
            radioData.storeTrimData();
            // Serial.println("BLE: RESET_TRIM executed");
            break;
            
        case CMD_UPDATE_PARAM:
            switch (paramId) {
                case PARAM_TRIM_PITCH:
                    radioData.trimData.pitch = paramValue;
                    break;
                case PARAM_TRIM_ROLL:
                    radioData.trimData.roll = paramValue;
                    break;
                case PARAM_DDR_PITCH:
                    radioData.dualRateData.pitch = paramValue / 100.0;
                    break;
                case PARAM_DDR_ROLL:
                    radioData.dualRateData.roll = paramValue / 100.0;
                    break;
                case PARAM_EXPO_PITCH:
                    radioData.expoData.pitch = paramValue / 100.0;
                    break;
                case PARAM_EXPO_ROLL:
                    radioData.expoData.roll = paramValue / 100.0;
                    break;
                case PARAM_EXPO_THROTTLE:
                    radioData.expoData.throttle = paramValue / 100.0;
                    break;
                case PARAM_THROTTLE_TO_PITCH:
                    radioData.mixerData.throttleToPitch = paramValue / 100.0;
                    break;
                case PARAM_THROTTLE_MIN:
                    radioData.analogToDigitalData.throttleLimit.min = paramValue;
                    break;
                case PARAM_THROTTLE_MAX:
                    radioData.analogToDigitalData.throttleLimit.max = paramValue;
                    break;
                case PARAM_V_MIN:
                    radioData.transmitterData.v_sound_min = paramValue;
                    break;
                case PARAM_V_MAX:
                    radioData.transmitterData.v_sound_max = paramValue;
                    break;
            }
            cmdId = 0; // Reset command ID after processing
            interrupts();
            break;
    }
    
    interrupts();


    if (millis() - lastUpdate < 100) return;
    lastUpdate = millis();

    uint8_t buffer[128];
    int offset = 0;

    // TX_DATA (31 bytes)
    {
        writeUint8(buffer, offset, radioData.functionData.armed ? 1 : 0);
        writeFloat(buffer, offset, radioData.analogData.battery);
        writeFloat(buffer, offset, radioData.analogData.throttle);
        writeFloat(buffer, offset, radioData.trimData.pitch * 100);
        writeFloat(buffer, offset, radioData.trimData.roll * 100);
        writeFloat(buffer, offset, radioData.functionData.throttle * 100);
        writeFloat(buffer, offset, radioData.functionData.pitch * 100);
        writeFloat(buffer, offset, radioData.functionData.roll * 100);
    }

    // RX_DATA (17 bytes)
    {
        writeFloat(buffer, offset, radioData.transmitterData.receiverBatteryVoltage);
        writeFloat(buffer, offset, radioData.transmitterData.receiverAltitude);
        writeFloat(buffer, offset, radioData.transmitterData.receiverMaxAlitude);
        writeFloat(buffer, offset, radioData.transmitterData.receiverVerticalSpeed);
    }

    // GPS_DATA (35 bytes)
    {
        writeDouble(buffer, offset, radioData.digitalData.gpsLatitude);
        writeDouble(buffer, offset, radioData.digitalData.gpsLongitude);
        writeFloat(buffer, offset, radioData.digitalData.gpsHdop);
        writeFloat(buffer, offset, radioData.digitalData.gpsAltitudeMeters);
        writeUint16(buffer, offset, radioData.digitalData.gpsSatellites);
        writeFloat(buffer, offset, radioData.digitalData.heading);
    }

    // PARAM_DATA (41 bytes)
    {
        writeFloat(buffer, offset, radioData.dualRateData.pitch * 100);
        writeFloat(buffer, offset, radioData.dualRateData.roll * 100);
        writeFloat(buffer, offset, radioData.expoData.pitch * 100);
        writeFloat(buffer, offset, radioData.expoData.roll * 100);
        writeFloat(buffer, offset, radioData.expoData.throttle * 100);
        writeFloat(buffer, offset, radioData.mixerData.throttleToPitch * 100);
        writeFloat(buffer, offset, radioData.analogToDigitalData.throttleLimit.min);
        writeFloat(buffer, offset, radioData.analogToDigitalData.throttleLimit.max);
        writeFloat(buffer, offset, radioData.transmitterData.v_sound_max);
        writeFloat(buffer, offset, radioData.transmitterData.v_sound_min);
    }

    pDataChar->setValue(buffer, offset);
    pDataChar->notify();
}

BluetoothComm::~BluetoothComm()
{
    if(pServerCallback) delete pServerCallback;
    if(pRxCallback) delete pRxCallback;
}