#include "pwm_driver.h"

pwm_driver::pwm_driver(void){

}

pwm_driver::pwm_driver(const uint8_t addr, TwoWire &i2c)
  : _i2c_addr(addr), _i2c(&i2c),
  _prescale_val(0x1E), _osc_freq(PWM_INT_CLK)
{
  //_i2c_addr = addr;
  //_i2c = &i2c;
  //_prescale_val = 0x1E;     // default value
  //_osc_freq = PWM_INT_CLK;  // internal clock

}

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

void pwm_driver::reset(){
  i2c_write8(pwm_reg::mode1, PWM_MODE1_RESTART);
  delay(10);
}
void pwm_driver::sleep(){
  uint8_t tmp = i2c_read8(pwm_reg::mode1);
  uint8_t to_sleep = tmp | PWM_MODE1_SLEEP; // set sleep bit
  i2c_write8(pwm_reg::mode1, to_sleep);
  delay(5);  // wait till counter ticks over for sleep active
}
void pwm_driver::wakeup(){
  uint8_t tmp = i2c_read8(pwm_reg::mode1);
  uint8_t to_wake = tmp & ~PWM_MODE1_SLEEP; // clr sleep bit
  i2c_write8(pwm_reg::mode1, to_wake);
  delay(5);  // wait till counter ticks over for sleep active
}
// note: pre-configured for internal oscillator freq in _osc_freq
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

/*
 * sets the output to:
 *    totem pole output - PWM_OUT_TOTEM_P
 *    open drain output - PWM_OUT_OPEN_D
 */
void pwm_driver::setOutputMode(bool out_mode){
  uint8_t tmp = i2c_read8(pwm_reg::mode2);
  uint8_t set_to_mode;
  set_to_mode = (out_mode) ? (tmp | PWM_MODE2_OUTDRV) : (tmp & ~PWM_MODE2_OUTDRV);
  i2c_write8(pwm_reg::mode2, set_to_mode);
}

/*
 * returns pwm on time (assumes we have all our pwms start active at
 * 0 and become inactive at the pwm value, does not account for pwms
 * that don't start at 0 or that have the msb bit 4 set for low or high
 * ... keeping in mind low is dominant)
 *    *** TODO ***
 *      - see notes above and fix it... but for now... simple cases
 */
uint16_t pwm_driver::getPwm(uint8_t pwm_num){
  // calc address to proper PWM base address)
  int start_reg = pwm_reg::on_lsb + (PWM_NEXT_REG_INC * pwm_num);
  // ask for 2 bytes (ON_LSB, ON_MSB) next two would be: OFF_LSB, OFF_MSB
  int to_read = _i2c->requestFrom((int)_i2c_addr, start_reg, (int)2);
  uint16_t pwm_on_val = 0;
  if(_i2c->available()){
    uint8_t lsbyte = _i2c->read();
    pwm_on_val += lsbyte;
    if(_i2c->available()){
      uint8_t msbyte = _i2c->read();
      pwm_on_val |= (((uint16_t)(msbyte & 0x0F)) << 8);
      return pwm_on_val;
    }
  }
  return PWM_VAL_ERROR;    // use this an error
}

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

/*
 * set PWM
 *  -- pmw number = num
 *  -- active time = pin_val
 *  -- if output is active low -> invert = true
 */
void pwm_driver::setPin(uint8_t num, uint16_t pin_val, bool invert){
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
