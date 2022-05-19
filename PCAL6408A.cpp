// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#include "PCAL6408A.h"

/// @brief empty constructor
PCAL6408A::PCAL6408A(){}

/// @brief creates PCAL6408A instance on i2c bus at given address
/// @param addr 7 bit i2c address for the device
/// @param i2c pointer to the TwoWire (i2c) instance
PCAL6408A::PCAL6408A(uint8_t addr, TwoWire &i2c)
  : _i2c_addr(addr), _i2c(&i2c) {}

/// @brief used to connect a PCAL6408A instance to i2c bus at given address
/// @param addr 7 bit i2c address for the device
/// @param i2c pointer to the TwoWire (i2c) instance
void PCAL6408A::connect(uint8_t addr, TwoWire &i2c){
  _i2c_addr = addr;
  _i2c = &i2c;
}

/// @brief checks to make sure we can communicate with the PCAL6408A
/// @returns success (true/false)
bool PCAL6408A::begin(){
  // make sure we can talk to the part or return false
  _i2c->beginTransmission(_i2c_addr);
  if(_i2c->endTransmission() != 0){
    return false;
  }
  return true;
}

/// @brief write 8-bit value to the expansion port (PCAL6408A)
/// @param val 8-bit value to write to the port
void PCAL6408A::writeExPort(uint8_t val){
  i2c_write8(pcal_reg::outport, val);
}

/// @brief read 8-bits from the expansion port (PCAL6408)
/// @returns 8-bit value from the port *** TODO *** find a way to return status of read
uint8_t PCAL6408A::readExPort(){
  return i2c_read8(pcal_reg::inport);
}

/// @brief helper function to read 8-bits from the instance i2c
/// @param reg 8-bit register of what we want to read
/// @returns 8-bit value that was retured from the i2c *** TODO *** find a way to return status of read
uint8_t PCAL6408A::i2c_read8(uint8_t reg){
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(reg);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
  return _i2c->read();
}

/// @brief function to setup the port expander i/o
/// @param inv inverts the output when written (1 = invert, 0 = normal) - 1-bit per pin
/// @param cfg sets pin as hi-z input (1), or output (0) - 1-bit per pin
/// @param outds1 drive strength for pins 0-3 : b00 = 2.5mA, b01 = 5mA, b10 = 7.5mA, b11 = 10mA - 2bits/pin
/// @param outds2 drive strength for pins 4-7 : b00 = 2.5mA, b01 = 5mA, b10 = 7.5mA, b11 = 10mA - 2bits/pin
/// @param pupd_en enables the pu/pd for pins - 1-bit per pin
/// @param pupd_sel 1 for pu and 0 for pd on the pin - 1-bit per pin
/// @param outp_cfg sets whole port output as open drain (0x01) or push-pull (0x00)
void PCAL6408A::setupExPort(uint8_t inv, uint8_t cfg, uint8_t outds1, uint8_t outds2, uint8_t pupd_en,
  uint8_t pupd_sel, uint8_t outp_cfg){

  // setup the port as specified
  i2c_write8(pcal_reg::invert, inv);
  i2c_write8(pcal_reg::config, cfg);
  i2c_write8(pcal_reg::drv_str1, outds1);
  i2c_write8(pcal_reg::drv_str2, outds2);
  i2c_write8(pcal_reg::pupd_sel, pupd_sel);
  i2c_write8(pcal_reg::pupd_en, pupd_en);
  i2c_write8(pcal_reg::outp_cfg, outp_cfg);
}

// **** TODO **** should we check the status and return comm failures?
/// @brief helper function to write 8-bits to the instance i2c
/// @param reg register to write to
/// @param value 8 bit value to write to the register
void PCAL6408A::i2c_write8(uint8_t reg, uint8_t value){
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(reg);
  _i2c->write(value);
  _i2c->endTransmission();
}
