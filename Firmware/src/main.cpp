/* -------------------- Include --------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "esp32/clk.h"
#include "AnalogToDigital.hpp"
#include "DigitalToFunction.hpp"
#include "Expo.hpp"
#include "Trim.hpp"
#include "Mixer.hpp"
#include "FunctionToChannel.hpp"
#include "Transmitter.hpp"
#include "Model.hpp"
#include "DualRate.hpp"
#include "SensorToDigital.hpp"
#include "BluetoothComm.hpp"
#include <esp_task_wdt.h>

/* -------------------- Defines --------------------------------------------------------------------------------*/

/* -------------------- Variable -------------------------------------------------------------------------------*/
uint32_t                          targetTime = 0;         
RadioData                         radioData = RadioData();
AnalogToDigital                   analogToDigital = AnalogToDigital(radioData);
DigitalToFunction                 digitalToFunction = DigitalToFunction(radioData);
Expo                              expo = Expo(radioData);
DualRate                          dualRate = DualRate(radioData);
Trim                              trim = Trim(radioData);
Mixer                             mixer = Mixer(radioData);
FunctionToChannel                 functionToChannel = FunctionToChannel(radioData);
Transmitter                       transmitter = Transmitter(radioData);
Model                             model = Model(radioData);
SensorToDigital                   sensorToDigital = SensorToDigital(radioData);
BluetoothComm                     bluetoothComm = BluetoothComm(radioData);

const TickType_t                  loopDelay = 20 / portTICK_PERIOD_MS;
TickType_t                        lastWakeTime;
/* -------------------- Functions Prototypes -------------------------------------------------------------------*/

/* -------------------- Setup ----------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  // Nur für Debugging im USB-CDC Modus einkommentieren sonst startet das Board nicht ohne angeschlossenen Serial Monitor
  // while (!Serial) delay(10); 
  // Serial.setDebugOutput(true);
  // esp_log_level_set("*", ESP_LOG_VERBOSE);

  // Factory Reset FLASH
  // radioData.resetData();

  // Load Models
  radioData.loadGlobalData();
  radioData.loadModelData();
  Serial.println("Model loaded");
  
  sensorToDigital.begin();
  transmitter.begin();

  bluetoothComm.begin();

  Serial.println("Init done");
  lastWakeTime = xTaskGetTickCount();
}

/* -------------------- Main -----------------------------------------------------------------------------------*/
void loop() { // Core 1
    xTaskDelayUntil(&lastWakeTime, loopDelay);
    analogToDigital.doFunction();
    sensorToDigital.doFunction();
    digitalToFunction.doFunction();
    expo.doFunction();
    trim.doFunction();
    dualRate.doFunction();
    mixer.doFunction();
    functionToChannel.doFunction();
    transmitter.doFunction();
    bluetoothComm.doFunction();
}