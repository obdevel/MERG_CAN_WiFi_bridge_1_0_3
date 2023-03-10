/*

  (c) Duncan Greenwood, 2019, 2020

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                   and indicate if changes were made. You may do so in any reasonable manner,
                   but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                  your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                  legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

//
/// a class to implement an MCP23008 I2C GPIO expander IC
//

#include "gpio.h"

void MCP23008::begin(uint8_t addr) {

  _i2caddr = addr;

  Wire.begin();
  Wire.beginTransmission(_i2caddr);

  Wire.write((byte)MCP23008_IODIR);
  Wire.write((byte)0xFF);  // all inputs
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);

  Wire.endTransmission();
}

uint8_t MCP23008::readGPIO(void) {

  return _read8(MCP23008_GPIO);
}

void MCP23008::writeGPIO(uint8_t pval) {

  _write8(MCP23008_GPIO, pval);
}

void MCP23008::pinMode(uint8_t p, uint8_t d) {

  uint8_t pval;

  if (p > 7)
    return;

  pval = _read8(MCP23008_IODIR);

  if (d == INPUT) {
    pval |= 1 << p;
  } else {
    pval &= ~(1 << p);
  }

  _write8(MCP23008_IODIR, pval);
}

void MCP23008::pullUp(uint8_t pin, uint8_t d) {

  uint8_t pval;

  if (pin > 7)
    return;

  pval = _read8(MCP23008_GPPU);

  if (d == HIGH) {
    pval |= 1 << pin;
  } else {
    pval &= ~(1 << pin);
  }

  _write8(MCP23008_GPPU, pval);
}

uint8_t MCP23008::digitalRead(uint8_t pin) {

  if (pin > 7)
    return 0;

  return (readGPIO() >> pin) & 0x1;
}

void MCP23008::digitalWrite(uint8_t p, uint8_t d) {

  uint8_t pval;

  if (p > 7)
    return;

  // read the current GPIO output latches
  pval = readGPIO();

  // set the pin and direction
  if (d == HIGH) {
    pval |= 1 << p;
  } else {
    pval &= ~(1 << p);
  }

  // write the new GPIO
  writeGPIO(pval);
}

uint8_t MCP23008::_read8(uint8_t reg) {

  Wire.beginTransmission(_i2caddr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(_i2caddr, (uint8_t)1);

  return Wire.read();
}

void MCP23008::_write8(uint8_t reg, uint8_t data) {

  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)data);
  Wire.endTransmission();
}

char *MCP23008::get_port_state_as_char(void) {

  uint8_t port_state = readGPIO();

  // note inverted logic

  for (byte i = 0; i < 8; i++) {
    if ((port_state >> i) & 0x01) {
      _switch_state_char[i] = '0';
    } else {
      _switch_state_char[i] = '1';
    }
  }

  _switch_state_char[8] = 0;
  return _switch_state_char;
}
