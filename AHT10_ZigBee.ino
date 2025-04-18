// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates Zigbee temperature and humidity sensor Sleepy device.
 *
 * The example demonstrates how to use Zigbee library to create an end device temperature and humidity sensor.
 * The sensor is a Zigbee end device, which is reporting data to the Zigbee network.
 *
 * Proper Zigbee mode must be selected in Tools->Zigbee mode
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 *
 * Please check the README.md for instructions and more detailed description.
 *
 * Created by Jan Procházka (https://github.com/P-R-O-C-H-Y/)
 */

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include <Wire.h>
#include <AHTxx.h>
#include "Zigbee.h"

/* Zigbee temperature + humidity sensor configuration */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  (2*60*60-5)         /* Sleep for 1h59min55s + 5s delay for establishing connection => data reported every 2 hours */
//#define VBAT_PIN A0
//#define BATTV_MAX    4.1     // maximum voltage of battery
//#define BATTV_MIN    3.2     // what we regard as an empty battery
//#define BATTV_LOW    3.4     // voltage considered to be low battery

uint8_t button = BOOT_PIN;
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); //sensor address, sensor type
ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);

/************************ Temp sensor *****************************/
void meausureAndSleep() {
  rgbLedWrite(RGB_BUILTIN, 5, 5, 5);  // white
  
  while (aht10.begin() != true)
  {
    Serial.println(F("AHT1x not connected or fail to load calibration coefficient"));

    delay(5000);
  }

  rgbLedWrite(RGB_BUILTIN, 0, 5, 0);  // Green
  Serial.println(F("AHT10 OK"));

  // Measure temperature sensor value
  float temperature = aht10.readTemperature();

  // Use temperature value as humidity value to demonstrate both temperature and humidity
  float humidity = aht10.readHumidity(AHTXX_USE_READ_DATA);

  //float battv = ((float)analogRead(VBAT_PIN) / 4095) * 3.3 * (150.0/620.0) * 1.05;
  //float battpc = (uint8_t)(((battv - BATTV_MIN) / (BATTV_MAX - BATTV_MIN)) * 100);
  //zbTempSensor.setBatteryPercentage(battpc);

  // Update temperature and humidity values in Temperature sensor EP
  zbTempSensor.setTemperature(temperature);
  zbTempSensor.setHumidity(humidity);

  // Report temperature and humidity values
  zbTempSensor.report();
  //Serial.printf("Reported temperature: %.2f°C, Humidity: %.2f%%, Battery: %.1f%%\r\n", temperature, humidity, battpc);
  Serial.printf("Reported temperature: %.2f°C, Humidity: %.2f%%\r\n", temperature, humidity);

  rgbLedWrite(RGB_BUILTIN, 0, 0, 0);  // Off / black
  // Add small delay to allow the data to be sent before going to sleep
  delay(5000);

  // Put device to deep sleep
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
}

/********************* Arduino functions **************************/
void setup() {
  Serial.begin(115200);
  Serial.println();
  rgbLedWrite(RGB_BUILTIN, 5, 0, 0);  // Red
  //pinMode(VBAT_PIN, INPUT);

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Configure the wake up source and set to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Optional: set Zigbee device name and model
  zbTempSensor.setManufacturerAndModel("Espressif", "SleepyZigbeeTempSensorTest");

  // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
  zbTempSensor.setMinMaxValue(-40, 85);

  // Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
  zbTempSensor.setTolerance(0.3);

  // Set power source to battery and set battery percentage to measured value (now 100% for demonstration)
  // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) anytime
  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100);

  // Add humidity cluster to the temperature sensor device with min, max and tolerance values
  zbTempSensor.addHumiditySensor(0, 100, 2);

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbTempSensor);

  // Create a custom Zigbee configuration for End Device with keep alive 10s to avoid interference with reporting data
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;

  rgbLedWrite(RGB_BUILTIN, 0, 0, 5);  // Blue
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    rgbLedWrite(RGB_BUILTIN, 0, 0, 0);  // Black
    //ESP.restart();
    esp_deep_sleep_start();
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.println("Successfully connected to Zigbee network");

  rgbLedWrite(RGB_BUILTIN, 5, 5, 0);  // Yellow
  // Delay approx 1s (may be adjusted) to allow establishing proper connection with coordinator, needed for sleepy devices
  delay(1000);
}

void loop() {
  // Checking button for factory reset
  if (digitalRead(button) == LOW) {  // Push button pressed
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(button) == LOW) {
      delay(50);
      if ((millis() - startTime) > 3000) {
        // If key pressed for more than 3secs, factory reset Zigbee and reboot
        Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
        delay(1000);
        Zigbee.factoryReset();
      }
    }
  }

  // Call the function to measure temperature and put the device to sleep
  meausureAndSleep();
}
