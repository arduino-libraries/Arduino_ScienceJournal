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

#ifndef _INA226_H_
#define _INA226_H_

#include <Arduino.h>
#include <Wire.h>

class INA226Class {
public:
  INA226Class(TwoWire& wire);
  virtual ~INA226Class();

  int begin(uint8_t address, float shuntResistance = 0.0002);
  void end();

  float readCurrent();
  float readBusVoltage();

private:
  long readRegister(uint8_t address);
  int writeRegister(uint8_t address, uint16_t value);

private:
  TwoWire* _wire;
  uint8_t _i2cAddress;
};

extern INA226Class INA226;

#endif
