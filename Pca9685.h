/***************************************************************************//**
 *  @file
 *
 *  module for PCA9685 PWM driver
 *
 *  @copyright 2022-present Meta Platforms, Inc. and affiliates.
 *              Confidential and proprietary.
 *//***************************************************************************/

#ifndef Pca9685_H_
#define Pca9685_H_

#include <Arduino.h>
#include <Wire.h>


// registers
namespace pwm_reg{
	enum Regs : uint8_t {
		mode1 = 0,
		mode2,
		subaddr1,
		subaddr2,
		subaddr3,
		allcall,
		on_lsb,
		on_msb,
		off_lsb,
		off_msb,
		all_on_lsb = 	0xFA,
		all_on_msb = 	0xFB,
		all_off_lsb = 	0xFC,
		all_off_msb = 	0xFD,
		prescale = 		0xFE,
		test_mode = 	0xFF
	};
	enum Mode1Bits : uint8_t {
		mode1_restart = 0x80,
		mode1_extclk = 0x40,
		mode1_auto_inc = 0x20,
		mode1_sleep = 0x10,
		mode1_sub1 = 0x08,
		mode1_sub2 = 0x04,
		mode1_sub3 = 0x02,
		mode1_allcall = 0x01
	};
	enum Mode2Bits : uint8_t {
		mode2__ro = 0xE0,
		mode2_invert = 0x10,
		mode2_out_ch = 0x08,
		mode2_outdrive = 0x04,
		mode2_dis_hiZ = 0x02,
		mode2_dis_sel = 0x01
	};

	const uint8_t kNextRegInc = 4;
	const uint8_t kPwmMinNum = 0;
	const uint8_t kPwmMaxMax = 15;
}

// PWM base register bitmasks
#define PWM_MSB_ON      0x10  // applied to the ON high byte
#define PWM_MSB_OFF     0x10  // applied to the OFF high byte (off takes precedence)

// prescale value calc stuff
#define PWM_INT_CLK         25000000.0
#define PWM_RES             4096.0
#define PWM_PRESCALE_MIN    0x03
#define PWM_PRESCALE_MAX    0xFF
#define PWM_MIN_FREQ        48
#define PWM_MAX_FREQ        3052

namespace pwm_out_types{
	enum PwmOutTypeVals : bool {
		kPwmOutOpenDrain = false,
		kPwmOutTotemPole = true
	};
}

namespace pwm_errors{
	const uint16_t kPwmValReadError = 9999;
}

class PwmDriver {
public:
  PwmDriver();
  PwmDriver(const uint8_t addr, TwoWire &i2c);
  bool init(const uint8_t addr, TwoWire &i2c);
  void reset();
  void sleep();
  void wakeup();
  void setPwmFreq(float freq);
  void setOutputMode(pwm_out_types::PwmOutTypeVals out_mode);
  uint16_t getPwm(uint8_t pwm_num);
  void setPwm(uint8_t pwm_num, uint16_t tc_on, uint16_t tc_off);
  void setPwmOut(uint8_t num, uint16_t pin_val, bool invert = false);

private:
  uint8_t   PrescaleVal_;
  uint32_t  OscFreq_;
  uint8_t   I2cAddr_;
  TwoWire   *I2c_;

  uint8_t i2cRead8(uint8_t reg);
  uint8_t i2cWrite8(uint8_t reg, uint8_t value);
  uint8_t i2c_readBytes(uint8_t reg, uint8_t len, uint8_t* data);
};

#endif /* Pca9685_H_ */
