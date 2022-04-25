#ifndef pwm_driver_h
#define pwm_driver_h

#include <Arduino.h>
#include <Wire.h>


// registers
namespace pwm_reg{
	enum regs : uint8_t {
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
}


// for PWMs 0 thru 15 "math"
// mult PWM base by PWM# (0-15) and add to base reg
#define PWM_NEXT_REG_INC  4
#define PWM_NUM_MIN       0
#define PWM_NUM_MAX       15

// MODE1 register bit masks
#define PWM_MODE1_RESTART   0x80
#define PWM_MODE1_EXTCLK    0x40
#define PWM_MODE1_AUTO_INC  0x20
#define PWM_MODE1_SLEEP     0x10
#define PWM_MODE1_SUB1      0x08
#define PWM_MODE1_SUB2      0x04
#define PWM_MODE1_SUB3      0x02
#define PWM_MODE1_ALLCALL   0x01

// MODE2 register bit masks
#define PWM_MODE2_RO        0xE0
#define PWM_MODE2_INVERT    0x10
#define PWM_MODE2_OUT_CH    0x08
#define PWM_MODE2_OUTDRV    0x04  // OD (0), totem pole (1)
#define PWM_MODE2_DIS_HIZ   0x02  // this bit is dominant over lower one
#define PWM_MODE2_DIS_SEL   0x01  // low (0) vs. OUTDRV (1) (OD = hi-z, tp = high)

#define PWM_OUT_TOTEM_P     true
#define PWM_OUT_OPEN_D      false

// PWM base register bitmasks
#define PWM_MSB_ON      0x10  // applied to the ON high byte
#define PWM_MSB_OFF     0x10  // a[[;oed tp tje PFF jogj byte (off takes precedence)

// prescale value calc stuff
#define PWM_INT_CLK         25000000.0
#define PWM_RES             4096.0
#define PWM_PRESCALE_MIN    0x03
#define PWM_PRESCALE_MAX    0xFF
#define PWM_MIN_FREQ        48
#define PWM_MAX_FREQ        3052

// some error codes
#define PWM_VAL_ERROR       10001

class pwm_driver {
public:
  pwm_driver();
  pwm_driver(const uint8_t addr, TwoWire &i2c);
  bool init(const uint8_t addr, TwoWire &i2c);
  void reset();
  void sleep();
  void wakeup();
  void setPwmFreq(float freq);
  void setOutputMode(bool out_mode);
  uint16_t getPwm(uint8_t pwm_num);
  void setPwm(uint8_t pwm_num, uint16_t tc_on, uint16_t tc_off);
  void setPin(uint8_t num, uint16_t pin_val, bool invert = false);

private:
  uint8_t   _prescale_val;
  uint32_t  _osc_freq;
  uint8_t   _i2c_addr;
  TwoWire   *_i2c;

  uint8_t i2c_read8(uint8_t reg);
  void i2c_write8(uint8_t reg, uint8_t value);
};

#endif
