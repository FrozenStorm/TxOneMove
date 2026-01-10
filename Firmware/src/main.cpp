/* -------------------- Include --------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "esp_clk.h"
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

#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>

/* -------------------- Defines --------------------------------------------------------------------------------*/
#define MOTION_SENSOR_ID      55
#define MOTION_SENSOR_ADDRESS 0x29
#define MOTION_SENSOR_BUS     0
#define PIN_MOTION_SENSOR_SCL 6
#define PIN_MOTION_SENSOR_SDA 5

#define GPS_SERIAL_NUM        2 // UART3
#define PIN_GPS_TX            3
#define PIN_GPS_RX            4
#define GPS_BAUD              9600

/* -------------------- Variable -------------------------------------------------------------------------------*/
TwoWire                           I2CBNO = TwoWire(MOTION_SENSOR_BUS);
Adafruit_BNO055                   bno = Adafruit_BNO055(MOTION_SENSOR_ID, MOTION_SENSOR_ADDRESS, &I2CBNO);
TinyGPSPlus                       gps;

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
SensorToDigital                   sensorToDigital = SensorToDigital(radioData, &bno, &gps);
/* -------------------- Functions Prototypes -------------------------------------------------------------------*/
void myMainTask(void *pvParameters);
void mySerialTask(void *pvParameters);
void initBNO055(void);
void initGps(void);

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

  // Motion Sensor Init
  initBNO055();
  
  // GPS Init
  // initGps();

  // Create Tasks
  xTaskCreatePinnedToCore(myMainTask, "MainTask", 20000, NULL, 2, NULL, 1);
  // xTaskCreatePinnedToCore(mySerialTask, "SerialTask", 10000, NULL, 1, NULL, 1);
  
  Serial.println("Init done");
}

void initBNO055() {
  if (!I2CBNO.begin(PIN_MOTION_SENSOR_SDA, PIN_MOTION_SENSOR_SCL)) {
    Serial.println("❌ I2C-Bus Fehler BNO055!");
    while (1) { digitalWrite(LED_BUILTIN, millis() % 200 < 100); delay(50); }
  }
  
  if (!bno.begin()) {
    Serial.println("❌ Kein BNO055 gefunden!");
    while (1) { digitalWrite(LED_BUILTIN, millis() % 200 < 100); delay(50); }
  }
  
  Serial.println("✅ BNO055 gefunden & initialisiert");
  
  // Fixed Kalibrierung (Ihre Werte)
  adafruit_bno055_offsets_t fixedCalib = {
    0, 4, -8,     // Accel X,Y,Z
    29, 366, 245, // Mag X,Y,Z  
    0, -3, -1,    // Gyro X,Y,Z
    1000, 602     // Accel/Mag Radius
  };
  
  bno.setExtCrystalUse(true);
  bno.setSensorOffsets(fixedCalib);
  bno.setMode(OPERATION_MODE_NDOF);
  
  Serial.println("✅ BNO055 Kalibrierung geladen");
  
  // Initial Status
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.printf("Initial Kalib: SYS:%d G:%d A:%d M:%d\n", sys, gyro, accel, mag);
}

void initGps() {
  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(200);
  
  Serial.println("=== ATGM336H GPS + TinyGPS++ ===");
  Serial.printf("GPS UART3: RX=%d, TX=%d, Baud=%d\n", PIN_GPS_RX, PIN_GPS_TX, GPS_BAUD);
  
  // TinyGPS++ PMTK-Konfig (optional)
  Serial2.println("$PMTK220,1000*1F");  // 1Hz
  Serial2.println("$PMTK314,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");  // GGA+RMC
  Serial.println("✓ GPS konfiguriert (TinyGPS++ ready)");
}

/* -------------------- Main -----------------------------------------------------------------------------------*/
void loop() { // Core 1
}

/* -------------------- Task -----------------------------------------------------------------------------------*/
void myMainTask(void *pvParameters) {
  const TickType_t loopDelay = 20 / portTICK_PERIOD_MS;
  TickType_t lastWakeTime = xTaskGetTickCount();
  Serial.println("Main Task Started");
  for (;;) {
    xTaskDelayUntil(&lastWakeTime, loopDelay);
    analogToDigital.doFunction(); // 3ms
    sensorToDigital.doFunction(); // 4ms
    digitalToFunction.doFunction(); // <<1ms
    expo.doFunction(); // <<1ms
    trim.doFunction(); // <<1ms
    dualRate.doFunction(); // <<1ms
    mixer.doFunction(); // <<1ms
    functionToChannel.doFunction(); // <<1ms
    transmitter.doFunction(); // <<1ms
  }
}

void mySerialTask(void *pvParameters) {
  const TickType_t loopDelay = 500 / portTICK_PERIOD_MS;
  TickType_t lastWakeTime = xTaskGetTickCount();
  Serial.println("Serial Task Started");
  for (;;) {
    xTaskDelayUntil(&lastWakeTime, loopDelay);

    // Serial.println("-----------------------------------------------");
    // Serial.print("AP IP-Adresse = ");Serial.println(WiFi.softAPIP());
    // Serial.printf("Freier Heap = %u Bytes\n", ESP.getFreeHeap());

    Serial.println("**** DigitalData ****");
    // Serial.print("radioData.digitalData.stickLeftRight = "); Serial.println(radioData.digitalData.stickLeftRight);
    // Serial.print("radioData.digitalData.stickUpDown = "); Serial.println(radioData.digitalData.stickUpDown);
    // Serial.print("radioData.digitalData.arm = "); Serial.println(radioData.digitalData.arm);
    // Serial.print("radioData.digitalData.trim = "); Serial.println(radioData.digitalData.trim);
    // Serial.print("radioData.digitalData.pitch = "); Serial.println(radioData.digitalData.pitch);
    // Serial.print("radioData.digitalData.roll = "); Serial.println(radioData.digitalData.roll);
    // Serial.print("radioData.digitalData.yaw = "); Serial.println(radioData.digitalData.yaw);
    // Serial.print("radioData.digitalData.altitude = "); Serial.println(radioData.digitalData.altitude);
    // Serial.print("radioData.digitalData.temperature = "); Serial.println(radioData.digitalData.temperature);
    Serial.print("radioData.digitalData.orientation = "); Serial.println(radioData.orientationNames[radioData.digitalData.orientation]);

    Serial.println("**** FunctionData ****");
    Serial.print("radioData.functionData.pitch = "); Serial.println(radioData.functionData.pitch);
    Serial.print("radioData.functionData.roll = "); Serial.println(radioData.functionData.roll);
    Serial.print("radioData.functionData.throttle = "); Serial.println(radioData.functionData.throttle);
    Serial.print("radioData.functionData.armed = "); Serial.println(radioData.functionData.armed);
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

    Serial.println("**** TrimData ****");
    Serial.print("radioData.trimData.roll = "); Serial.println(radioData.trimData.roll);
    Serial.print("radioData.trimData.pitch = "); Serial.println(radioData.trimData.pitch);

    // Serial.println("**** MixerData ****");
    // Serial.print("radioData.mixerData.throttleToPitch = "); Serial.println(radioData.mixerData.throttleToPitch);

    Serial.println("**** TransmitterData ****");
    Serial.print("radioData.transmitterData.receiverBatteryVoltage = "); Serial.println(radioData.transmitterData.receiverBatteryVoltage);

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

    Serial.println("**** AnalogData ****");
    // Serial.print("radioData.analogData.stickUpDown = "); Serial.println(radioData.analogData.stickUpDown);
    // Serial.print("radioData.analogData.stickLeftRight = "); Serial.println(radioData.analogData.stickLeftRight);
    Serial.print("radioData.analogData.battery = "); Serial.println(radioData.analogData.battery);
    Serial.print("radioData.analogData.throttle = "); Serial.println(radioData.analogData.throttle);
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