// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#include "pwm_driver.h"

///@brief an empty constructor
pwm_driver::pwm_driver(void){

}

/// @brief constructor with params.  prescale and osc freq are autoset here too
/// @param addr i2c address the pwm chips is on
/// @param i2c receives a pointer to the i2c the chip is on
pwm_driver::pwm_driver(const uint8_t addr, TwoWire &i2c)
  : _i2c_addr(addr), _i2c(&i2c),
  _prescale_val(0x1E), _osc_freq(PWM_INT_CLK)
{
  // empty all done in the constructor
}

/// @brief init of pwm_driver object.  prescale and osc freq are autoset here too
/// @param addr i2c address the pwm chips is on
/// @param i2c receives a pointer to the i2c the chip is on
/// @returns true if the device is found at the address on the given i2c
bool pwm_driver::init(const uint8_t addr, TwoWire &i2c)
{
  _i2c_addr = addr;
  _i2c = &i2c;
  _prescale_val = 0x1E;     // default value
  _osc_freq = PWM_INT_CLK;  // internal clock

  _i2c->beginTransmission(_i2c_addr);
  if(_i2c->endTransmission() != 0){
    return false;
  }

  else{
    return true;
  }
}

/// @brief resets the chip (and gets it ready to run...).  MUSB be called for PWM
///         chip to work properly
void pwm_driver::reset(){
  i2c_write8(pwm_reg::mode1, PWM_MODE1_RESTART);
  delay(10);
}

/// @brief puts the pwm chip into sleep mode (lower power)
void pwm_driver::sleep(){
  uint8_t tmp = i2c_read8(pwm_reg::mode1);
  uint8_t to_sleep = tmp | PWM_MODE1_SLEEP; // set sleep bit
  i2c_write8(pwm_reg::mode1, to_sleep);
  delay(5);  // wait till counter ticks over for sleep active
}

/// @brief wakes the pwm chip from sleep
void pwm_driver::wakeup(){
  uint8_t tmp = i2c_read8(pwm_reg::mode1);
  uint8_t to_wake = tmp & ~PWM_MODE1_SLEEP; // clr sleep bit
  i2c_write8(pwm_reg::mode1, to_wake);
  delay(5);  // wait till counter ticks over for sleep active
}

/// @brief sets up the pwm to run at a given frequency
/// @param freq float value of frequency to run at
/// @note pre-configured for internal oscillator freq (_osc_freq)
void pwm_driver::setPwmFreq(float freq){
  if(freq < PWM_MIN_FREQ) freq = PWM_MIN_FREQ;
  if(freq > PWM_MAX_FREQ) freq = PWM_MAX_FREQ;

  float prescale = ((_osc_freq / (freq * PWM_RES)) + 0.5) - 1;
  if(prescale < PWM_PRESCALE_MIN) prescale = PWM_PRESCALE_MIN;
  if(prescale > PWM_PRESCALE_MAX) prescale = PWM_PRESCALE_MAX;

  _prescale_val = (uint8_t) prescale;

  // have to put the device to sleep to change prescalar
  uint8_t temp = i2c_read8(pwm_reg::mode1); // store current val
  uint8_t set_to = (temp & ~PWM_MODE1_RESTART) | PWM_MODE1_SLEEP;
  i2c_write8(pwm_reg::mode1, set_to);
  i2c_write8(pwm_reg::prescale, _prescale_val);
  i2c_write8(pwm_reg::mode1, temp);
  // give it a break to process the request
  delay(5);
  // turn it back on and use auto_inc
  set_to = temp | PWM_MODE1_RESTART | PWM_MODE1_AUTO_INC;
  i2c_write8(pwm_reg::mode1, set_to);
}

/// @brief sets the outputs to totem pole or open draing
/// @param out_mode pwm_out_type_val: totem pole, open drain
void pwm_driver::setOutputMode(pwm_out_types::pwm_out_type_vals out_mode){
  uint8_t tmp = i2c_read8(pwm_reg::mode2);
  uint8_t set_to_mode;
  set_to_mode = ((bool)out_mode) ? (tmp | PWM_MODE2_OUTDRV) : (tmp & ~PWM_MODE2_OUTDRV);
  i2c_write8(pwm_reg::mode2, set_to_mode);
}

/// @brief returns pwm active time (assumes all pwms start at t0 and become
///       inactive at the active time).
/// @param pwm_num number of the pwm that we are interested in
/// @returns value of active time (0-4095)
uint16_t pwm_driver::getPwm(uint8_t pwm_num){
  // calc address to proper PWM base address)
  int start_reg = pwm_reg::on_lsb + (PWM_NEXT_REG_INC * pwm_num);
  // ask for 3 bytes (ON_LSB, ON_MSB, OFF_LSB, OFF_MSB)
  int to_read = _i2c->requestFrom((int)_i2c_addr, start_reg, (int)4);
  uint16_t pwm_on_val = 0;
  if(_i2c->available()){
    uint8_t lsbyte = _i2c->read();
    pwm_on_val += lsbyte;
    if(_i2c->available()){
      uint8_t msbyte = _i2c->read();
      if(msbyte & PWM_MSB_ON){
        pwm_on_val = 4095;
      }
      pwm_on_val |= (((uint16_t)(msbyte & 0x0F)) << 8);
      if(_i2c->available()){
        uint8_t lval_lsbyte = _i2c->read();   // read and ignore
        if(_i2c->available()){
          uint8_t lval_msbyte = _i2c->read();
          // check for low override
          if((lval_msbyte & PWM_MSB_OFF) == PWM_MSB_OFF){
            pwm_on_val = 0;
          }
          return pwm_on_val;
        }
      }
    }
  }
  return PWM_VAL_ERROR;    // use this an error
}

/// @brief sets the PWM registers for active "on" and inactive "off" time
/// @param pwm_num the pwm number to set
/// @param tc_on the clock count that the pwm turns on
/// @param tc_off the clock count that the pwm turns off
void pwm_driver::setPwm(uint8_t pwm_num, uint16_t tc_on, uint16_t tc_off){
  _i2c->beginTransmission(_i2c_addr);
  uint8_t start_reg = pwm_reg::on_lsb + (PWM_NEXT_REG_INC * pwm_num);
  _i2c->write(start_reg);
  _i2c->write((uint8_t)(tc_on & 0xFF));
  _i2c->write((uint8_t)(tc_on >> 8));
  _i2c->write((uint8_t)(tc_off & 0xFF));
  _i2c->write((uint8_t)(tc_off >> 8));
  _i2c->endTransmission();
}

/// @brief sets the PWM output for a pin (active time)
/// @param num the pwm number to set
/// @param pin_val active time
/// @param invert false: active high, true: active low
void pwm_driver::setPwmOut(uint8_t num, uint16_t pin_val, bool invert){
  pin_val = min(pin_val, (uint16_t)4095);
  if(invert){
    if(0 == pin_val){
      setPwm(num, 4096, 0);
    } else if (4095 == pin_val){
      setPwm(num, 0, 4096);
    } else {
      uint16_t off_val = 4095 - pin_val;
      setPwm(num, 0, off_val);
    }
  } else {
    if(4095 == pin_val){
      setPwm(num, 4096, 0);
    } else if (0 == pin_val){
      setPwm(num, 0, 4096);
    } else {
      setPwm(num, 0, pin_val);
    }
  }
}

uint8_t pwm_driver::i2c_read8(uint8_t reg){
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(reg);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
  return _i2c->read();
}

void pwm_driver::i2c_write8(uint8_t reg, uint8_t value){
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(reg);
  _i2c->write(value);
  if(_i2c->endTransmission() != 0){
    // for debug
    //SerialUSB.println("failed i2c wrie for pwm");
  }
}
