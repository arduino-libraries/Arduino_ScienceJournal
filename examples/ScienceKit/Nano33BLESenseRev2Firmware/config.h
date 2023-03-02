#include "ArduinoScienceJournal.h"

// Flash memory
#include "SerialFlash.h"
#include <SPI.h>

const int FlashChipSelect = 2;

#include <Arduino_APDS9960.h>
#include <Arduino_HS300x.h> // hs3003
#include <Arduino_LPS22HB.h>

// IMU
// BMM150 and bmi 270
#include "Arduino_BMI270_BMM150.h"


#include <avr/dtostrf.h>



const uint32_t SHUNT_MICRO_OHM{100000};  ///< Shunt resistance in Micro-Ohm, e.g. 100000 is 0.1 Ohm
const uint16_t MAXIMUM_AMPS{1};          ///< Max expected amps, clamped from 1A to a max of 1022A

// INA_Class      INA;

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
  // bmm150 and bmi270
  if (!IMU.begin()) {
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

  if (!HS300x.begin()) {
    Serial.println("Failed to initialized HTS!");
    blinkLoop();
  }

  if (!BARO.begin()) {
    Serial.println("Failed to initialized BARO!");
    blinkLoop();
  }
}
