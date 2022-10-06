/*
  This file is part of the PhysicsLabFirmware library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <ArduinoBLE.h>       // click here to install the library: http://librarymanager#ArduinoBLE
#include "config.h"


#define SCIENCE_KIT_UUID(val) ("555a0001-" val "-467a-9538-01f0652c74e8")

BLEService                     service                    (SCIENCE_KIT_UUID("0000"));
BLEUnsignedIntCharacteristic   versionCharacteristic      (SCIENCE_KIT_UUID("0001"), BLERead);
BLEByteCharacteristic          ledCharacteristic          (SCIENCE_KIT_UUID("1001"), BLERead | BLEWrite);
BLEUnsignedShortCharacteristic input1Characteristic       (SCIENCE_KIT_UUID("2001"), BLENotify);
BLEUnsignedShortCharacteristic input2Characteristic       (SCIENCE_KIT_UUID("2002"), BLENotify);
BLEUnsignedShortCharacteristic input3Characteristic       (SCIENCE_KIT_UUID("2003"), BLENotify);
BLEByteCharacteristic          ouput1Characteristic       (SCIENCE_KIT_UUID("3001"), BLERead | BLEWrite);
BLEByteCharacteristic          ouput2Characteristic       (SCIENCE_KIT_UUID("3002"), BLERead | BLEWrite);
BLEFloatCharacteristic         voltageCharacteristic      (SCIENCE_KIT_UUID("4001"), BLENotify);
BLEFloatCharacteristic         currentCharacteristic      (SCIENCE_KIT_UUID("4002"), BLENotify);
BLEFloatCharacteristic         resistanceCharacteristic   (SCIENCE_KIT_UUID("4003"), BLENotify);
BLECharacteristic              accelerationCharacteristic (SCIENCE_KIT_UUID("5001"), BLENotify, 3 * sizeof(float));
BLECharacteristic              gyroscopeCharacteristic    (SCIENCE_KIT_UUID("5002"), BLENotify, 3 * sizeof(float));
BLECharacteristic              magneticFieldCharacteristic(SCIENCE_KIT_UUID("5003"), BLENotify, 3 * sizeof(float));
BLECharacteristic              lightCharacteristic        (SCIENCE_KIT_UUID("6001"), BLENotify, 3 * sizeof(float));
BLEFloatCharacteristic         TemperatureCharacteristic  (SCIENCE_KIT_UUID("6002"), BLENotify);

// check
const int LED_PIN            =  0; // check
const int INPUT1_PIN         = A3;
const int INPUT2_PIN         = A1;
const int INPUT3_PIN         = A0;
const int OUTPUT1_PIN        =  5;
const int OUTPUT2_PIN        =  1;
const int RESISTANCE_PIN     = A2;

String name;
unsigned long lastNotify = 0;

unsigned long imuTime;

#define RESISTOR_AUX_LOW  47000.0
#define RESISTOR_AUX_HIGH 979.16 // 47k in parallel with 1k = 979.16 Ohm

#define IMU_UPDATE_TIME 50

//#define DEBUG //uncomment to debug the code :)

void setup() {
  Serial.begin(9600);
#ifdef DEBUG
  while (!Serial);
  Serial.println("Started");
#endif

  pinMode(LED_PIN, OUTPUT);
  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);
  pinMode(INPUT3_PIN, INPUT);
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);

  sensorsInit();

  if (!BLE.begin()) {
    Serial.println("Failed to initialized BLE!");

    while (1);
  }

  String address = BLE.address();

  address.toUpperCase();

  name = "MKRSci";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);

  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(ledCharacteristic);
  service.addCharacteristic(input1Characteristic);
  service.addCharacteristic(input2Characteristic);
  service.addCharacteristic(input3Characteristic);
  service.addCharacteristic(ouput1Characteristic);
  service.addCharacteristic(ouput2Characteristic);
  service.addCharacteristic(voltageCharacteristic);
  service.addCharacteristic(currentCharacteristic);
  service.addCharacteristic(resistanceCharacteristic);
  service.addCharacteristic(accelerationCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);
  service.addCharacteristic(magneticFieldCharacteristic);
  service.addCharacteristic(lightCharacteristic);
  service.addCharacteristic(TemperatureCharacteristic);
  BLE.addService(service);

  BLE.advertise();
  imuTime = millis();
}

void loop() {
  lastNotify = 0;

  while (BLE.connected()) {
    if (ledCharacteristic.written()) { // si usa?
      analogWrite(LED_PIN, ledCharacteristic.value());
    }

    if (ouput1Characteristic.written()) {
      analogWrite(OUTPUT1_PIN, ouput1Characteristic.value());
    }

    if (ouput2Characteristic.written()) {
      analogWrite(OUTPUT2_PIN, ouput2Characteristic.value());
    }

    unsigned long now = millis();

    if (abs((long)now - (long)lastNotify) >= 100) {
      lastNotify = now;

      // every 100ms update subscribed characteristics
      updateSubscribedCharacteristics();
    }

    updateSubscribedIMUCharacteristics();
  }
}

void updateSubscribedCharacteristics() {
  if (input1Characteristic.subscribed()) {
    input1Characteristic.writeValue(analogReadAverage(INPUT1_PIN, 30));
  }

  if (input2Characteristic.subscribed()) {
    input2Characteristic.writeValue(analogReadAverage(INPUT2_PIN, 30));
  }

  if (input3Characteristic.subscribed()) {
    input3Characteristic.writeValue(analogReadAverage(INPUT3_PIN, 30));
  }

  if (voltageCharacteristic.subscribed()) {
    //dtostrf(, 7, 4, busChar);
    float voltage = INA.getBusMilliVolts(0) / 1000.0;
#ifdef DEBUG
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println("V");
#endif
    voltageCharacteristic.writeValue(voltage);
  }

  if (currentCharacteristic.subscribed()) {
    float current = (INA.getBusMicroAmps(0))/1000.0F;
#ifdef DEBUG
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println("mA");
#endif
    currentCharacteristic.writeValue(current/1000.0F);
  }

  if (resistanceCharacteristic.subscribed()) {
    float resistanceAvg = INFINITY; //open circuit as default
    resistanceAvg = ResSens.getResistorValue();
#ifdef DEBUG
    Serial.print("Resistance: ");
    Serial.print(resistanceAvg);
    Serial.println(" Ohm");
#endif
    resistanceCharacteristic.writeValue(resistanceAvg);
  }
}


int analogReadAverage(int pin, int numberOfSamples) {
  int averageValue = 0;
  for (int i = 0; i < numberOfSamples; i++) {
    averageValue += analogRead(pin);
  }

  return (averageValue / numberOfSamples);
}

void updateSubscribedIMUCharacteristics() {
  if (millis() - imuTime > IMU_UPDATE_TIME) {
    imuTime = millis();
    if (accelerationCharacteristic.subscribed()) {
      if (IMU_SK.accelerationAvailable()) {
        float x, y, z;
        IMU_SK.readAcceleration(x, y, z);
        float acceleration[3];

        acceleration[0] = x;
        acceleration[1] = y;
        acceleration[2] = z;
        accelerationCharacteristic.writeValue((byte*)acceleration, sizeof(acceleration));
      }
    }

    if (gyroscopeCharacteristic.subscribed()) {
      if (IMU_SK.gyroscopeAvailable()) {
        float x, y, z;
        IMU_SK.readGyroscope(x, y, z);
        float gyroscope[3];

        gyroscope[0] = x;
        gyroscope[1] =y;
        gyroscope[2] = z;
        gyroscopeCharacteristic.writeValue((byte*)gyroscope, sizeof(gyroscope));
      }
    }


    if (magneticFieldCharacteristic.subscribed()) {
      float x, y, z;
      if (BME.magneticFieldAvailable()) {
        BME.readMagneticField(x, y, z);
        float magneticField[3];
        magneticField[0] = x;
        magneticField[1] = y;
        magneticField[2] = z;
        magneticFieldCharacteristic.writeValue((byte*)magneticField, sizeof(magneticField));
      }
    }
  }
}