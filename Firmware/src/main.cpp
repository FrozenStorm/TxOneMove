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
/* -------------------- Functions Prototypes -------------------------------------------------------------------*/
void myMainTask();
void mySerialTask();

/* -------------------- Setup ----------------------------------------------------------------------------------*/
void setup() {
  // delay(5000);
  Serial.begin(115200);
  Serial.println("Init started");

  Serial.setDebugOutput(true);
  esp_log_level_set("*", ESP_LOG_VERBOSE);

  // Factory Reset FLASH
  // radioData.resetData();

  // Load Models
  radioData.loadGlobalData();
  radioData.loadModelData();
  // radioData.storeGlobalData();
  // radioData.storeModelData();
  Serial.println("Model loaded");
  
  sensorToDigital.begin();

  bluetoothComm.begin();
  
  Serial.println("Init done");
}

void printMemoryInfo() {
    Serial.println("=== Memory Info ===");
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" Bytes");
    
    Serial.print("Stack High Water Mark (Core 0): ");
    Serial.print(uxTaskGetStackHighWaterMark(xTaskGetIdleTaskHandleForCPU(0)));
    Serial.println(" Bytes");
    
    Serial.print("Stack High Water Mark (Core 1): ");
    Serial.print(uxTaskGetStackHighWaterMark(xTaskGetIdleTaskHandleForCPU(1)));
    Serial.println(" Bytes");
}

/* -------------------- Main -----------------------------------------------------------------------------------*/
void loop() { // Core 1
  myMainTask(); // FreeRTOS Task
  // mySerialTask(); // FreeRTOS Task
}

/* -------------------- Task -----------------------------------------------------------------------------------*/
void myMainTask() {
  const TickType_t loopDelay = 20 / portTICK_PERIOD_MS;
  TickType_t lastWakeTime = xTaskGetTickCount();
  Serial.println("Main Task Started");
  
  unsigned long totalLoopTime = 0;
  int loopCount = 100;
  UBaseType_t prevStack = uxTaskGetStackHighWaterMark(NULL);
  size_t prevHeap = ESP.getFreeHeap();
  
  for (;;) {
    xTaskDelayUntil(&lastWakeTime, loopDelay);
    
    unsigned long loopStart = micros();
    
    unsigned long t1_start = micros(); analogToDigital.doFunction(); unsigned long t1 = micros() - t1_start;
    unsigned long t2_start = micros(); sensorToDigital.doFunction(); unsigned long t2 = micros() - t2_start;
    unsigned long t3_start = micros(); digitalToFunction.doFunction(); unsigned long t3 = micros() - t3_start;
    unsigned long t4_start = micros(); expo.doFunction(); unsigned long t4 = micros() - t4_start;
    unsigned long t5_start = micros(); trim.doFunction(); unsigned long t5 = micros() - t5_start;
    unsigned long t6_start = micros(); dualRate.doFunction(); unsigned long t6 = micros() - t6_start;
    unsigned long t7_start = micros(); mixer.doFunction(); unsigned long t7 = micros() - t7_start;
    unsigned long t8_start = micros(); functionToChannel.doFunction(); unsigned long t8 = micros() - t8_start;
    unsigned long t9_start = micros(); transmitter.doFunction(); unsigned long t9 = micros() - t9_start;
    unsigned long t10_start = micros(); bluetoothComm.doFunction(lastWakeTime); unsigned long t10 = micros() - t10_start;
    
    totalLoopTime += (micros() - loopStart);
    
    loopCount++;
    if (loopCount >= 100) {  // Alle 2s (100*20ms) Stats drucken
      UBaseType_t stackUsed = (prevStack - uxTaskGetStackHighWaterMark(NULL)) * 4;  // Bytes approx.
      size_t heapUsed = prevHeap - ESP.getFreeHeap();
      
      Serial.printf("Loop Avg: %lu us | Stack Peak: %u B | High Water Mark %u B | Heap Delta: %u B | Free Heap: %u B\n", 
                    totalLoopTime / 100, stackUsed, uxTaskGetStackHighWaterMark(NULL) * 4, heapUsed, ESP.getFreeHeap());
      Serial.printf("analogToDigital: %lu us\n", t1);
      Serial.printf("sensorToDigital: %lu us\n", t2);
      Serial.printf("digitalToFunction: %lu us\n", t3);
      Serial.printf("expo: %lu us\n", t4);
      Serial.printf("trim: %lu us\n", t5);
      Serial.printf("dualRate: %lu us\n", t6);
      Serial.printf("mixer: %lu us\n", t7);
      Serial.printf("functionToChannel: %lu us\n", t8);
      Serial.printf("transmitter: %lu us\n", t9);
      Serial.printf("bluetoothComm: %lu us\n", t10);
      
      totalLoopTime = 0; loopCount = 0;
      prevStack = uxTaskGetStackHighWaterMark(NULL);
      prevHeap = ESP.getFreeHeap();
    }
  }
}


void mySerialTask() {
  const TickType_t loopDelay = 500 / portTICK_PERIOD_MS;
  TickType_t lastWakeTime = xTaskGetTickCount();
  Serial.println("Serial Task Started");
  for (;;) {
    xTaskDelayUntil(&lastWakeTime, loopDelay);
    
    // Serial.println("-----------------------------------------------");
    // Serial.print("AP IP-Adresse = ");Serial.println(WiFi.softAPIP());
    // Serial.printf("Freier Heap = %u Bytes\n", ESP.getFreeHeap());

    // Serial.println("**** DigitalData ****");
    // Serial.print("radioData.digitalData.stickLeftRight = "); Serial.println(radioData.digitalData.stickLeftRight);
    // Serial.print("radioData.digitalData.stickUpDown = "); Serial.println(radioData.digitalData.stickUpDown);
    // Serial.print("radioData.digitalData.arm = "); Serial.println(radioData.digitalData.arm);
    // Serial.print("radioData.digitalData.trim = "); Serial.println(radioData.digitalData.trim);
    // Serial.print("radioData.digitalData.pitch = "); Serial.println(radioData.digitalData.pitch);
    // Serial.print("radioData.digitalData.roll = "); Serial.println(radioData.digitalData.roll);
    // Serial.print("radioData.digitalData.yaw = "); Serial.println(radioData.digitalData.yaw);
    // Serial.print("radioData.digitalData.altitude = "); Serial.println(radioData.digitalData.altitude);
    // Serial.print("radioData.digitalData.temperature = "); Serial.println(radioData.digitalData.temperature);
    // Serial.print("radioData.digitalData.orientation = "); Serial.println(radioData.orientationNames[radioData.digitalData.orientation]);

    // Serial.println("**** FunctionData ****");
    // Serial.print("radioData.functionData.pitch = "); Serial.println(radioData.functionData.pitch);
    // Serial.print("radioData.functionData.roll = "); Serial.println(radioData.functionData.roll);
    // Serial.print("radioData.functionData.throttle = "); Serial.println(radioData.functionData.throttle);
    // Serial.print("radioData.functionData.armed = "); Serial.println(radioData.functionData.armed);
    // Serial.print("radioData.functionData.vTailLeft = "); Serial.println(radioData.functionData.vTailLeft);
    // Serial.print("radioData.functionData.vTailRight = "); Serial.println(radioData.functionData.vTailRight);

    // Serial.println("**** AnalogToDigitalData ****");
    // Serial.print("radioData.analogToDigitalData.stickLimitUpDown.min = "); Serial.println(radioData.analogToDigitalData.stickLimitUpDown.min);
    // Serial.print("radioData.analogToDigitalData.stickLimitUpDown.max = "); Serial.println(radioData.analogToDigitalData.stickLimitUpDown.max);
    // Serial.print("radioData.analogToDigitalData.stickLimitUpDown.center = "); Serial.println(radioData.analogToDigitalData.stickLimitUpDown.center);
    // Serial.print("radioData.analogToDigitalData.stickLimitUpDown.invert = "); Serial.println(radioData.analogToDigitalData.stickLimitUpDown.invert);
    // Serial.print("radioData.analogToDigitalData.stickLimitLeftRight.min = "); Serial.println(radioData.analogToDigitalData.stickLimitLeftRight.min);
    // Serial.print("radioData.analogToDigitalData.stickLimitLeftRight.max = "); Serial.println(radioData.analogToDigitalData.stickLimitLeftRight.max);
    // Serial.print("radioData.analogToDigitalData.stickLimitLeftRight.center = "); Serial.println(radioData.analogToDigitalData.stickLimitLeftRight.center);
    // Serial.print("radioData.analogToDigitalData.stickLimitLeftRight.invert = "); Serial.println(radioData.analogToDigitalData.stickLimitLeftRight.invert);
    // Serial.print("radioData.analogToDigitalData.longPressDurationMs = "); Serial.println(radioData.analogToDigitalData.longPressDurationMs);

    // Serial.println("**** SensorToDigitalData ****");
    // Serial.print("radioData.sensorToDigitalData.angleLimitPitch.delta = "); Serial.println(radioData.sensorToDigitalData.angleLimitPitch.delta);
    // Serial.print("radioData.sensorToDigitalData.angleLimitPitch.center = "); Serial.println(radioData.sensorToDigitalData.angleLimitPitch.center);
    // Serial.print("radioData.sensorToDigitalData.angleLimitRoll.delta = "); Serial.println(radioData.sensorToDigitalData.angleLimitRoll.delta);
    // Serial.print("radioData.sensorToDigitalData.angleLimitRoll.center = "); Serial.println(radioData.sensorToDigitalData.angleLimitRoll.center);
    // Serial.print("radioData.sensorToDigitalData.seaLevelPressure = "); Serial.println(radioData.sensorToDigitalData.seaLevelPressure);

    // Serial.println("**** ExpoData ****");
    // Serial.print("radioData.expoData.roll = "); Serial.println(radioData.expoData.roll);
    // Serial.print("radioData.expoData.pitch = "); Serial.println(radioData.expoData.pitch);
    // Serial.print("radioData.expoData.throttle = "); Serial.println(radioData.expoData.throttle);

    // Serial.println("**** DualRateData ****");
    // Serial.print("radioData.dualRateData.roll = "); Serial.println(radioData.dualRateData.roll);
    // Serial.print("radioData.dualRateData.pitch = "); Serial.println(radioData.dualRateData.pitch);
    // Serial.print("radioData.dualRateData.throttle = "); Serial.println(radioData.dualRateData.throttle);

    // Serial.println("**** TrimData ****");
    // Serial.print("radioData.trimData.roll = "); Serial.println(radioData.trimData.roll);
    // Serial.print("radioData.trimData.pitch = "); Serial.println(radioData.trimData.pitch);

    // Serial.println("**** MixerData ****");
    // Serial.print("radioData.mixerData.throttleToPitch = "); Serial.println(radioData.mixerData.throttleToPitch);

    // Serial.println("**** TransmitterData ****");
    // Serial.print("radioData.transmitterData.receiverBatteryVoltage = "); Serial.println(radioData.transmitterData.receiverBatteryVoltage);

    // Serial.println("**** FunctionToChannelData ****");
    // for (int i = 0; i < SUPPORTED_CHANNELS; i++)
    // {
    //     Serial.printf("radioData.functionToChannelData.invertChannel[%d] = %d\n", i, radioData.functionToChannelData.invertChannel[i]);
    //     Serial.printf("radioData.functionToChannelData.functionOnChannel[%d] = %s\n", i, radioData.functionNames[radioData.functionToChannelData.functionOnChannel[i]]);
    //     Serial.printf("radioData.functionToChannelData.upperLimitChannel[%d] = %d\n", i, radioData.functionToChannelData.upperLimitChannel[i]);
    //     Serial.printf("radioData.functionToChannelData.lowerLimitChannel[%d] = %d\n", i, radioData.functionToChannelData.lowerLimitChannel[i]);
    // }

    // Serial.println("**** ChannelData ****");
    // for (int i = 0; i < CHANNEL_COUNT; i++)
    // {
    //     Serial.printf("radioData.channelData.channel[%d] = %d\n", i, radioData.channelData.channel[i]);
    // }

    // Serial.println("**** RawData ****");
    // Serial.print("radioData.rawData.stickUpDown = "); Serial.println(radioData.rawData.stickUpDown);
    // Serial.print("radioData.rawData.stickLeftRight = "); Serial.println(radioData.rawData.stickLeftRight);
    // Serial.print("radioData.rawData.battery = "); Serial.println(radioData.rawData.battery);
    // Serial.print("radioData.rawData.gyroX = "); Serial.println(radioData.rawData.gyroX);
    // Serial.print("radioData.rawData.gyroY = "); Serial.println(radioData.rawData.gyroY);
    // Serial.print("radioData.rawData.gyroZ = "); Serial.println(radioData.rawData.gyroZ);
    // Serial.print("radioData.rawData.accelX = "); Serial.println(radioData.rawData.accelX);
    // Serial.print("radioData.rawData.accelY = "); Serial.println(radioData.rawData.accelY);
    // Serial.print("radioData.rawData.accelZ = "); Serial.println(radioData.rawData.accelZ);

    // Serial.println("**** AnalogData ****");
    // Serial.print("radioData.analogData.stickUpDown = "); Serial.println(radioData.analogData.stickUpDown);
    // Serial.print("radioData.analogData.stickLeftRight = "); Serial.println(radioData.analogData.stickLeftRight);
    // Serial.print("radioData.analogData.battery = "); Serial.println(radioData.analogData.battery);
    // Serial.print("radioData.analogData.throttle = "); Serial.println(radioData.analogData.throttle);
    // Serial.print("radioData.analogData.gyroPitch = "); Serial.println(radioData.analogData.gyroPitch);
    // Serial.print("radioData.analogData.gyroRoll = "); Serial.println(radioData.analogData.gyroRoll);
    // Serial.print("radioData.analogData.gyroYaw = "); Serial.println(radioData.analogData.gyroYaw);
    // Serial.print("radioData.analogData.accelPitch = "); Serial.println(radioData.analogData.accelPitch);
    // Serial.print("radioData.analogData.accelRoll = "); Serial.println(radioData.analogData.accelRoll);
    // Serial.print("radioData.analogData.accelYaw = "); Serial.println(radioData.analogData.accelYaw);
    // Serial.print("radioData.analogData.pitch = "); Serial.println(radioData.analogData.pitch);
    // Serial.print("radioData.analogData.roll = "); Serial.println(radioData.analogData.roll);
    // Serial.print("radioData.analogData.yaw = "); Serial.println(radioData.analogData.yaw);

    // Serial.println("**** ModelData ****");
    // Serial.print("radioData.modelData.modelName = "); Serial.println(radioData.getModelName());

    // ACHTUNG MACHT MANCHMAL PROBLEME BEI FREERTOS!!!
    // Serial.println("**** FreeRTOS ****");
    // Serial.printf("Free Heap: %u\n", ESP.getFreeHeap());
    // Serial.printf("Min Free Heap: %u\n", ESP.getMinFreeHeap());
    // Serial.printf("Max Alloc Heap: %u\n", ESP.getMaxAllocHeap());
    // Serial.printf("Free Stack: %u\n", uxTaskGetStackHighWaterMark(NULL));
    // Serial.printf("Task Count: %u\n", uxTaskGetNumberOfTasks());
    // Serial.printf("🔁 Reset-Grund: %d\n", esp_reset_reason());
    // Serial.printf("APB Clock: %u Hz\n", esp_clk_apb_freq());    // UART hängt an APB Clock
    // Serial.printf("CPU Clock: %u Hz\n", esp_clk_cpu_freq());    // CPU Core Speed
  }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  Serial.print("🧨 Stack Overflow in Task: ");
  Serial.println(pcTaskName);
  // optional: Endlosschleife oder Neustart
  while (true) {
    delay(1000);
  }
}