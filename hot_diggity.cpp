// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#include "hot_diggity.h"

//-------------------------- INIT stuff ---------------------------
hot_diggity::hot_diggity(){
}

bool hot_diggity::begin(){
	bool device_begin_ok = true;
	bool all_device_begin_ok = true;
	// run all the ::begin sort of stuff
	_p_exp.connect(i2c_addrs::PCAL6408A_addr, Wire);
	if(_p_exp.begin() != true) device_begin_ok = false;
	if(!device_begin_ok){
		hds.println("failed to init p_ex");
		device_begin_ok = true;			// reset for next device
		all_device_begin_ok = false;
	}
	if(_pwm.init(i2c_addrs::PCA9685_addr, Wire) != true) device_begin_ok = false;
	if(!device_begin_ok){
		hds.println("failed to init pwm");
		device_begin_ok = true;			// reset for next device
		all_device_begin_ok = false;
	}
	// setup PWM for output
	_pwm.reset();
	delay(250);
	_pwm.setOutputMode(PWM_OUT_TOTEM_P);
	_pwm.setPwmFreq(1000.0);
	// initialize the htr power values
	for(int i = (int)pwm_info::pwm_left1; i <= (int)pwm_info::pwm_right2; i++){
		_htr_pwr[i] = 0;
	}
	// temp sensors
	for(int i = (int)tsense_info::ctrl1; i <= (int)tsense_info::flexi5; i++){
		if(_temp_sense[i].begin(tsense_info::tsense_adr[i], Wire) != true){
			device_begin_ok = false;
		}
	}
	if(!device_begin_ok){
		hds.println("failed to init temp on i2c0");
		device_begin_ok = true;			// reset for next device
		all_device_begin_ok = false;
	}

	for(int i = (int)tsense_info::flexi6; i <= (int)tsense_info::right2; i++){
		if(_temp_sense[i].begin(tsense_info::tsense_adr[i], Wire1) != true){
			device_begin_ok = false;
		}
	}
	if(!device_begin_ok){
		hds.println("failed to init temp on i2c1");
		device_begin_ok = true;			// reset for next device
		all_device_begin_ok = false;
	}
	if(all_device_begin_ok){
		hds.println("all parts found");
	}else{
		hds.println("failed to find one or more i2c parts");
	}
	// setup the port_expander
	// write to the port first before turning outputs on
  	_p_exp.writeExPort(P_EX_LED | P_EX_PWM_EN);
	// set port pins up
	{
		uint8_t ins = P_EX_BRD_ID_MASK | P_EX_BRD_REV_MASK;
		// (inv/norm, in/out, dr_str1,2, en/dis pupd, pu/pd, od/pushpull)
  		_p_exp.setupExPort(0x00, ins, 0xFF, 0xFF, ins, ins, 0);
	}
	// read board id and version
	get_board_info();
	// set the polling rate and polling active stuff
	_poll_rate = 5000;		// set to 5 second intervals
	_poll_active = false;
	// zero out tsense info
	for(int i = tsense_info::ctrl1; i <= tsense_info::right2; i++){
		_t_sense_data[i] = -99.9;
		_t_sense_time[i] = 0;
	}
	return all_device_begin_ok;
}

#pragma region port_expander
// ------------------------ Port Expander Functions -------------
uint8_t hot_diggity::getBoardId(){
	return _board_id;
}
uint8_t hot_diggity::getBoardRev(){
	return _board_rev;
}

void hot_diggity::setExLed(bool turn_on){
	uint8_t val = _p_exp.readExPort();
	val = (turn_on) ? (val & ~P_EX_LED) : (val | P_EX_LED);
	_p_exp.writeExPort(val);
}

void hot_diggity::toggleExLed(){
	uint8_t val = _p_exp.readExPort();
	val ^= P_EX_LED;			// toggle the current state of the LED
	_p_exp.writeExPort(val);
}

void hot_diggity::setPwmOutEn(bool turn_on){
	uint8_t val = _p_exp.readExPort();
	val = (turn_on) ? (val & ~P_EX_PWM_EN) : (val | P_EX_PWM_EN);
	_p_exp.writeExPort(val);
}
#pragma endregion

#pragma region heater_fx
// ----------------------- Heater funtions -------------------
void hot_diggity::setHeaterPower(pwm_info::pwm_sel htr_num, uint16_t power_mw){
	// see if there is enough power to grant the request
	int budget = getTotalHeaterPwr();
	budget -= _htr_pwr[htr_num];
	budget += power_mw;
	bool over_budget = (budget > 3000) ? true : false;
	bool over_power = (power_mw > 1000) ? true : false;
	if (over_budget || over_power){
		// send error message, do not execute
		if (over_budget) {
			Serial.println("** ERROR -- requested over 3000mW total");
		}
		if(over_power){
			Serial.println("** ERROR -- over 1000mW requested for heater");
		}
	}

	// else set the heater
	_htr_pwr[htr_num] = power_mw;
	uint16_t set_to = power_mw * 4;	// power goes in 250uW steps
	_pwm.setPin(htr_num, set_to, false);
}

uint16_t hot_diggity::getHeaterPower(pwm_info::pwm_sel htr_num){
	uint16_t val = _pwm.getPwm(htr_num);
	if(val == PWM_VAL_ERROR){
		Serial.println("error reading heater: " + String(htr_num));
		return PWM_VAL_ERROR;
	}
	val /= 4;	// need to change to mW from 250uW increments
	// *** TODO -- make sure this value matches _htr_pwr[htr_num] value
	return val;
}

uint16_t hot_diggity::getTotalHeaterPwr(){
	uint16_t tot_power = 0;
	for(int i = (int)pwm_info::pwm_left1; i <= (int)pwm_info::pwm_right2; i++){
		tot_power += _htr_pwr[i];
	}
	return tot_power;
}
#pragma endregion

#pragma region temp_sense
// --------------------- Temp Sense functions -----------------
float hot_diggity::getTemperature(tsense_info::tsense temp_num){
	return _temp_sense[temp_num].readTempC();
}

float hot_diggity::getTemperature(tsense_info::tsense temp_num, ulong *temp_time){
	float tval;
	tval = _temp_sense[temp_num].readTempC();
	*temp_time = millis();
	return tval;
}

#pragma endregion

#pragma region board_intfc
// --------------------- Interface funcitons -------------------
String hot_diggity::getBoardInfo(){
	// for debug re-read the ID
	get_board_info();
	String msg = "Board ID: " + String(getBoardId());
	msg += ", Board Rev: " + String(getBoardRev());
	return msg;
}

void hot_diggity::setRgbValue(uint8_t red, uint8_t grn, uint8_t blu){
	// take the 8 bit vals, and write them as 12 bit values
	_pwm.setPin(pwm_info::pwm_red, ((uint16_t)red) << 4, false);
	_pwm.setPin(pwm_info::pwm_grn, ((uint16_t)grn) << 4, false);
	_pwm.setPin(pwm_info::pwm_blu, ((uint16_t)blu) << 4, false);
}
#pragma endregion

#pragma region temp_polling
void hot_diggity::setPollRate(uint16_t rate){
	_poll_rate = rate;
}

uint16_t hot_diggity::getPollRate(){
	return _poll_rate;
}

void hot_diggity::setPollingState(String state){
	if(state == "ON") {
		_poll_active = true;
		_last_poll_time = millis();
	}
	else {_poll_active = false;}
}

String hot_diggity::getPollingState(){
	String ret_val = (_poll_active) ? "ON" : "OFF";
	return ret_val;
}

bool hot_diggity::getPollingStateBool(){
	return _poll_active;
}

void hot_diggity::checkPoll(){
	if(_poll_active){
		// use time and see if it is time to get/send a new report
		ulong now_ms = millis();
		ulong elapsed = now_ms - _last_poll_time;
		if((uint16_t)elapsed >= _poll_rate){
			_last_poll_time += _poll_rate;
			/*
			String dbg_msg = String(now_ms) + ", ";
			dbg_msg += String(elapsed) + ", ";
			dbg_msg += String(_last_poll_time) + ", ";
			dbg_msg += String(_poll_rate);
			hds.println(dbg_msg);
			*/
			// read all temp sensors with time stamps
			String msg = "";
			for(int i = tsense_info::ctrl1; i <= tsense_info::right2; i++){
				_t_sense_data[i] = getTemperature((tsense_info::tsense)i, &_t_sense_time[i]);
				msg += String(_t_sense_data[i]) + ", " + String(_t_sense_time[i]);
				if(i < tsense_info::right2) { msg += ", "; }
			}
			// print results
			hds.println(msg);
		}
	}
}
#pragma endregion

#pragma region private_stuff
//---------------- Private ------------------------------------
void hot_diggity::get_board_info(){
	{
		uint8_t brd = _p_exp.readExPort();
		_board_rev = brd & P_EX_BRD_REV_MASK;
		_board_id = (brd & P_EX_BRD_ID_MASK) >> P_EX_BRD_ID_SHIFT;
	}
}
#pragma endregion
