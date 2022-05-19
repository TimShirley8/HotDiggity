// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#ifndef PCAL6408A_h
#define PCAL6408A_h
#include <Arduino.h>
#include "Wire.h"

namespace pcal_reg{
  // registers for the PCAL6408A
  // all registers are by pin (each bit selects or deselects)
  // except the output configure (oupt_cfg)
  const uint8_t inport = 0x00;      /// ro - register to read from
  const uint8_t outport = 0x01;     /// rw - register to write to (can be read - only reflects output register)
  const uint8_t invert = 0x02;      /// rw - allows inversion of individual pins (0 = normal, 1 = invert)
  const uint8_t config = 0x03;      /// rw - sets as input (hi-z) - 1, or output - 0
  const uint8_t drv_str1 = 0x40;    /// rw - drive strength register for pins 0-3
  const uint8_t drv_str2 = 0x41;    /// rw - drive strength register for pins 4-7
  const uint8_t latch = 0x42;       /// rw - inp interrupt latching (0 = not latched, 1 = latch input, read clears)
  const uint8_t pupd_en = 0x43;     /// rw - enables (1), or disables (0) pull-up/pull-down
  const uint8_t pupd_sel = 0x44;    /// rw - turns pull-up or pull-down on (1), or off (0)
  const uint8_t irq_mask = 0x45;    /// rw - 1 = interrupt active, 0 = interrupt masked
  const uint8_t irq_status = 0x46;  /// ro - 1 = interrupt active, 0 = not source of interrupt
  const uint8_t outp_cfg = 0x4F;    /// rw - global setting for outputs as push-pull or open-drain
};

namespace pcal_drv_str{
  // drive strength settings (2 bits per pin)
  const uint8_t ds_2p5ma = 0;   /// set for 2.5mA drive strength
  const uint8_t ds_5p0ma = 1;   /// set for 5.0mA drive strength
  const uint8_t ds_7p5ma = 2;   /// set for 7.5mA drive strength
  const uint8_t ds_10ma = 3;    /// set for 10 mA drive strength
  // shift values for each pin
  const uint8_t shft0 = 0;
  const uint8_t shft1 = 2;
  const uint8_t shft2 = 4;
  const uint8_t shft3 = 6;
  const uint8_t shft4 = 0;
  const uint8_t shft5 = 2;
  const uint8_t shft6 = 4;
  const uint8_t shft7 = 6;
};

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
