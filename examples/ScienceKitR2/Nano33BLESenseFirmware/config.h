#include "ArduinoScienceJournal.h"

// Flash memory
#include "SerialFlash.h"
#include <SPI.h>

const int FlashChipSelect = 2;

#include <Arduino_APDS9960.h>
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>

// IMU
#include <Arduino_LSM6DSOX.h>
// 3 axis magnetometer
#include <BoschSensorClass.h>
// INA
#include <INA.h>
#include <avr/dtostrf.h>


LSM6DSOXClass IMU_SK = LSM6DSOXClass(Wire,0x6A);
BoschSensorClass BME = BoschSensorClass(Wire);

const uint32_t SHUNT_MICRO_OHM{100000};  ///< Shunt resistance in Micro-Ohm, e.g. 100000 is 0.1 Ohm
const uint16_t MAXIMUM_AMPS{1};          ///< Max expected amps, clamped from 1A to a max of 1022A

INA_Class      INA;

SerialFlashFile file;

void blinkLoop() {
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void sensorsInit() {
  // INA Init
  if (!INA.begin(MAXIMUM_AMPS, SHUNT_MICRO_OHM)) {
    Serial.println(F("No INA device found, retrying in 10 seconds..."));
    while(1);
  }                                       // while no devices detected
  Serial.println("INA init success!");
  INA.setBusConversion(8500);             // Maximum conversion time 8.244ms
  INA.setShuntConversion(8500);           // Maximum conversion time 8.244ms
  INA.setAveraging(128);                  // Average each reading n-times
  INA.setMode(INA_MODE_CONTINUOUS_BOTH);  // Bus/shunt measured continuously
  INA.alertOnBusOverVoltage(true, 5000);  // Trigger alert if over 5V on bus

  // bmm150
  if (!BME.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // LSM6DSOX init
  if (!IMU_SK.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU Initialized");

  // Flash Init
  if (!SerialFlash.begin(FlashChipSelect)) {
    Serial.println(F("Failed to initialize Flash!"));
    while (1);
  }

  Serial.println("Flash Initialized");

   if (!APDS.begin()) {
    Serial.println("Failed to initialized APDS!");
    blinkLoop();
  }

  if (!HTS.begin()) {
    Serial.println("Failed to initialized HTS!");
    blinkLoop();
  }

  if (!BARO.begin()) {
    Serial.println("Failed to initialized BARO!");
    blinkLoop();
  }
}
