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

#include "INA226.h"

#define INA226_BUS_VOLTAGE_REG 0x02
#define INA226_CURRENT_REG     0x04
#define INA226_CALIBRATION_REG 0x05

INA226Class::INA226Class(TwoWire& wire) :
  _wire(&wire)
{  
}

INA226Class::~INA226Class()
{
}

int INA226Class::begin(uint8_t address, float shuntResistance)
{
  _i2cAddress = address;

  // for 1 mA/bit
  uint16_t calibration = (0.00512 / (0.001 * shuntResistance));

  _wire->begin();

  // force 
  if (!writeRegister(INA226_CALIBRATION_REG, calibration)) {
    end();
    
    return 0;
  }

  return 1;
}

void INA226Class::end()
{
  _wire->end();
}

float INA226Class::readCurrent()
{
  long value = readRegister(INA226_CURRENT_REG);

  if (value == NAN) {
    return -1;
  }
  
  return (((int16_t)value) / 1000.0);
}

float INA226Class::readBusVoltage()
{
  long value = readRegister(INA226_BUS_VOLTAGE_REG);

  if (value == -1) {
    return NAN;
  }

  return (value * 0.00125);
}

long INA226Class::readRegister(uint8_t address)
{
  _wire->beginTransmission(_i2cAddress);
  _wire->write(address);
  if (_wire->endTransmission() != 0) {
    return -1;
  }

  if (_wire->requestFrom(_i2cAddress, 2) != 2) {
    return -1;
  }

  return ((_wire->read() << 8) | _wire->read());
}

int INA226Class::writeRegister(uint8_t address, uint16_t value)
{
  _wire->beginTransmission(_i2cAddress);
  _wire->write(address);
  _wire->write(value >> 8);
  _wire->write(value & 0xff);
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}

INA226Class INA226(Wire);
