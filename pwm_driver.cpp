// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#include "pwm_driver.h"

// #define DEBUG_CHK 1

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

#if DEBUG_CHK
  for(uint8_t add_num = 0x40; add_num <= 0x7F; add_num++){
    _i2c->beginTransmission(add_num);
    uint8_t ret_val = _i2c->endTransmission();
    if(ret_val == 0){
      String msg = "i2c found: @addr: " + String(add_num, HEX);
      SerialUSB.println(msg);
      _i2c_addr = add_num;
    } else {
      SerialUSB.println("fail: " + String(add_num, HEX));
    }
  }

  // for(uint8_t add_num = 0x4B; add_num <= 0x7F; add_num++){
  //   _i2c->beginTransmission(add_num);
  //   uint8_t ret_val = _i2c->endTransmission();
  //   if(ret_val == 0){
  //     String msg = "i2c found: @addr: " + String(add_num, HEX);
  //     SerialUSB.println(msg);
  //     _i2c_addr = add_num;
  //     return true;
  //   } else {
  //     SerialUSB.println("fail: " + String(add_num, HEX));
  //   }
  // }
  return false;

#else
  _i2c->beginTransmission(_i2c_addr);
  uint8_t ret_val = _i2c->endTransmission();
  if(ret_val != 0){
    String msg = "i2c_err: " + String(ret_val) + " addr: " + String(_i2c_addr, HEX);
    SerialUSB.println(msg);
    return false;
  }

  else{
    return true;
  }
#endif
}

/// @brief resets the chip (and gets it ready to run...).  MUSB be called for PWM
///         chip to work properly
void pwm_driver::reset(){
  uint8_t ret_val = i2c_write8(pwm_reg::mode1, pwm_reg::mode1_restart);
  if(ret_val != 0){
    SerialUSB.println("reset failed with: " + String(ret_val));
  }
  delay(10);
}

/// @brief puts the pwm chip into sleep mode (lower power)
void pwm_driver::sleep(){
  uint8_t tmp = i2c_read8(pwm_reg::mode1);
  uint8_t to_sleep = tmp | pwm_reg::mode1_sleep; // set sleep bit
  i2c_write8(pwm_reg::mode1, to_sleep);
  delay(5);  // wait till counter ticks over for sleep active
}

/// @brief wakes the pwm chip from sleep
void pwm_driver::wakeup(){
  uint8_t tmp = i2c_read8(pwm_reg::mode1);
  uint8_t to_wake = tmp & ~pwm_reg::mode1_sleep; // clr sleep bit
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
  uint8_t set_to = (temp & ~pwm_reg::mode1_restart) | pwm_reg::mode1_sleep;
  i2c_write8(pwm_reg::mode1, set_to);
  i2c_write8(pwm_reg::prescale, _prescale_val);
  i2c_write8(pwm_reg::mode1, temp);
  // give it a break to process the request
  delay(5);
  // turn it back on and use auto_inc
  set_to = temp | pwm_reg::mode1_restart | pwm_reg::mode1_auto_inc;
  i2c_write8(pwm_reg::mode1, set_to);
}

/// @brief sets the outputs to totem pole or open draing
/// @param out_mode pwm_out_type_val: totem pole, open drain
void pwm_driver::setOutputMode(pwm_out_types::pwm_out_type_vals out_mode){
  uint8_t tmp = i2c_read8(pwm_reg::mode2);
  uint8_t set_to_mode;
  set_to_mode = ((bool)out_mode) ? (tmp | pwm_reg::mode2_outdrive) : (tmp & ~pwm_reg::mode2_outdrive);
  i2c_write8(pwm_reg::mode2, set_to_mode);
}

/// @brief returns pwm active time (assumes all pwms start at t0 and become
///       inactive at the active time).
/// @param pwm_num number of the pwm that we are interested in
/// @returns value of active time (0-4095)
uint16_t pwm_driver::getPwm(uint8_t pwm_num){
  // calc address to proper PWM base address)
  int start_reg = pwm_reg::on_lsb + (pwm_reg::next_reg_inc * pwm_num);
  // ask for 4 bytes (ON_LSB, ON_MSB, OFF_LSB, OFF_MSB)
  uint8_t pwm_vals[4];
  uint8_t rcvd = pwm_driver::i2c_readBytes(start_reg, 4, &pwm_vals[0]);
  uint16_t pwm_on_val = 0;

  if(rcvd == 4){
    // check for full on/off conditions
    if(pwm_vals[3] & PWM_MSB_OFF){
      return 0;
    }
    if(pwm_vals[1] & PWM_MSB_ON){
      return 4095;
    }
    pwm_on_val = (((uint16_t)pwm_vals[3]) << 8) | pwm_vals[2];
    return pwm_on_val;
  }
  else {
    // output an error
    return pwm_errors::pwm_val_read_error;
  }
}

/// @brief sets the PWM registers for active "on" and inactive "off" time
/// @param pwm_num the pwm number to set
/// @param tc_on the clock count that the pwm turns on
/// @param tc_off the clock count that the pwm turns off
void pwm_driver::setPwm(uint8_t pwm_num, uint16_t tc_on, uint16_t tc_off){
  _i2c->beginTransmission(_i2c_addr);
  uint8_t start_reg = pwm_reg::on_lsb + (pwm_reg::next_reg_inc * pwm_num);
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

/// @brief helper function to get 1 byte from the i2c register
/// @param reg register that we want to read from
/// @returns 8-bit data from the i2c read operation
uint8_t pwm_driver::i2c_read8(uint8_t reg){
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(reg);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
  return _i2c->read();
}

/// @brief helper function to write a byte to an i2c register
/// @param reg register that we want to write to
/// @param value the value we wish to write to the register
uint8_t pwm_driver::i2c_write8(uint8_t reg, uint8_t value){
  // SerialUSB.println("pwm_i2c_addr = " + String(_i2c_addr, HEX));
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(reg);
  _i2c->write(value);
  return _i2c->endTransmission();
  // if(_i2c->endTransmission() != 0){
  //   return false;
    // for debug
    //SerialUSB.println("failed i2c wrie for pwm");
  // }
}

/// @brief helper function to read multiple bytes from i2c register
/// @param reg starting register that we want to read from
/// @param len number of bytes we want to read
/// @param data pointer to where data should be stored (caller must allocate)
/// @returns number of bytes read
uint8_t pwm_driver::i2c_readBytes(uint8_t reg, uint8_t len, uint8_t* data){
  uint8_t rcvd_cnt = 0;
  // let device know which reg we want to read from
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(reg);
  _i2c->endTransmission();

  _i2c->requestFrom(_i2c_addr, len);
  uint8_t to_get_cnt = len;
  uint8_t *tmp = data;   // this way we leave the data pointer where it is.

  while(_i2c->available() && to_get_cnt--){
    uint8_t b = _i2c->read();
    // SerialUSB.println("got: 0x" + String(b, HEX));
    *tmp++ = b;
    rcvd_cnt++;
  }
  return rcvd_cnt;
}
