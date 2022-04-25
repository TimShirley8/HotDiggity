// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#ifndef PCAL6408A_h
#define PCAL6408A_h
#include <Arduino.h>
#include "Wire.h"

// all registers are by pin (each bit selects or deselects)
// except the output configure (PCAL6408A_CFG)
#define PCAL6408A_INPORT  0x00
#define PCAL6408A_OUTPORT 0x01
#define PCAL6408A_INVERT  0x02    // 0 = normal, 1 = invert
#define PCAL6408A_CONFIG  0x03    // 1 = hi-z in, 0 = out
#define PCAL6408A_OUTDS1  0x40    // drive strength pins 0-3
#define PCAL6408A_OUTDS2  0x41    // drive strength pins 4-7
#define PCAL6408A_LATCH   0x42
#define PCAL6408A_PUPD_EN 0x43    // disable pu/pd = 0, enable = 1
#define PCAL6408A_PUPD_SEL  0x44  // 1 = pu, 0 = pd
#define PCAL6408A_IRQ_MASK  0x45  // 1 = irq, 0 = masked
#define PCAL6408A_IRQ_STAT  0x46
#define PCAL6408A_OUTP_CFG  0x4F  // lsb: 1 = OD, 0 = push-pull

class PCAL6408A{
public:
  PCAL6408A();
  PCAL6408A(uint8_t addr, TwoWire &i2c);
  bool begin(void);
  void connect(uint8_t addr, TwoWire &i2c);
  void writeExPort(uint8_t val);
  uint8_t readExPort(void);
  void setupExPort(uint8_t inv, uint8_t cfg, uint8_t outds1, uint8_t outds2, uint8_t pupd_en,
    uint8_t pupd_sel, uint8_t outp_cfg);

private:
  uint8_t _i2c_addr;
  TwoWire *_i2c;

  uint8_t i2c_read8(uint8_t reg);
  void i2c_write8(uint8_t reg, uint8_t value);
};
#endif
