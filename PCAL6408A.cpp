// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#include "PCAL6408A.h"

PCAL6408A::PCAL6408A(){}

PCAL6408A::PCAL6408A(uint8_t addr, TwoWire &i2c)
  : _i2c_addr(addr), _i2c(&i2c) {}

void PCAL6408A::connect(uint8_t addr, TwoWire &i2c){
  _i2c_addr = addr;
  _i2c = &i2c;
}

bool PCAL6408A::begin(){
  // make sure we can talk to the part or return false
  _i2c->beginTransmission(_i2c_addr);
  if(_i2c->endTransmission() != 0){
    return false;
  }
  return true;
}

void PCAL6408A::writeExPort(uint8_t val){
  i2c_write8(PCAL6408A_OUTPORT, val);
}

uint8_t PCAL6408A::readExPort(){
  return i2c_read8(PCAL6408A_INPORT);
}

uint8_t PCAL6408A::i2c_read8(uint8_t reg){
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(reg);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
  return _i2c->read();
}

void PCAL6408A::setupExPort(uint8_t inv, uint8_t cfg, uint8_t outds1, uint8_t outds2, uint8_t pupd_en,
  uint8_t pupd_sel, uint8_t outp_cfg){

  // setup the port as specified
  i2c_write8(PCAL6408A_INVERT, inv);
  i2c_write8(PCAL6408A_CONFIG, cfg);
  i2c_write8(PCAL6408A_OUTDS1, outds1);
  i2c_write8(PCAL6408A_OUTDS2, outds2);
  i2c_write8(PCAL6408A_PUPD_SEL, pupd_sel);
  i2c_write8(PCAL6408A_PUPD_EN, pupd_en);
  i2c_write8(PCAL6408A_OUTP_CFG, outp_cfg);
}

// **** TODO **** should we check the status and return comm failures?
void PCAL6408A::i2c_write8(uint8_t reg, uint8_t value){
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(reg);
  _i2c->write(value);
  _i2c->endTransmission();
}
