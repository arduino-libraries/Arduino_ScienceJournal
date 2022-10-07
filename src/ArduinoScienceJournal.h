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

#ifndef _ARDUINO_SCIENCE_JOURNAL_H_
#define _ARDUINO_SCIENCE_JOURNAL_H_

#include "Arduino.h"

#ifdef ARDUINO_ARCH_MBED
  #include "mbed.h"
#endif


#ifdef ARDUINO_ARCH_SAMD
  #define IMU_ADDRESS 0x00;
  #define VOLTAGE_REF_LEVEL 3300
  #define LIGHT_SENSOR_PIN A5
  #define TEMP_SENSOR_PIN A4
#else
  #define IMU_ADDRESS 0x00;
  #define VOLTAGE_REF_LEVEL 1100
  #define LIGHT_SENSOR_PIN A7
  #define TEMP_SENSOR_PIN 6
#endif

class ResistorSensorClass {
public:
  int getResistorValue() {
    int raw_voltage = analogRead(A2);
    int voltage_in = (raw_voltage * VOLTAGE_REF_LEVEL / 1024);
    int res =  ((3300 * 1000) / voltage_in) - 1000;
    return res;
  }

  int getResistor1KValue() {
    digitalWrite(6, HIGH);
    int raw_voltage = analogRead(A2);
    int voltage_in = (raw_voltage * VOLTAGE_REF_LEVEL / 1024);
    int res =  ((3300 * (47000*1000/48000)) / voltage_in) - (47000*1000/48000);
    digitalWrite(6, LOW);
    return res;
  }
};

extern ResistorSensorClass ResSens;

class TemperatureSensorClass {
public:
  int getTemperature(){
    int raw_voltage = analogRead(TEMP_SENSOR_PIN);
    int voltage_in = (raw_voltage * VOLTAGE_REF_LEVEL / 1024);
    float temperature = ((voltage_in) - 500.0F) / 10.0F;
    return temperature;
  }
};

extern TemperatureSensorClass TempSens;


class LightSensorClass {
public:
  int getLux(){
    int raw_voltage = analogRead( LIGHT_SENSOR_PIN);
    int voltage_in = (raw_voltage * VOLTAGE_REF_LEVEL / 1024);
    float lux = (voltage_in * 0.5F) + 0.5;
    return lux;
  }
};

extern LightSensorClass LightSens;

#endif
