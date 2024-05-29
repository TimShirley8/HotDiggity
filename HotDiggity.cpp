/***************************************************************************//**
 *  @file
 *
 *  main code for the Hot Diggity project
 *
 *  @copyright 2022-present Meta Platforms, Inc. and affiliates.
 *              Confidential and proprietary.
 *//***************************************************************************/

#include "HotDiggity.h"

//-------------------------- INIT stuff ---------------------------

/// @brief creates an instance of hot_diggity
hot_diggity::hot_diggity(){
}

/// @brief initializes/begins the port expander, the pwm chip, and the temp
///			sensors for the hot diggity system.  Also gets the board information
///			from the expander and stores it for retrieval later.
/// @returns true if all devices are found on their proper i2c bus, else false
bool hot_diggity::begin(){
	bool device_begin_ok = true;
	bool all_device_begin_ok = true;

	_cmd_str = "";
	_input_machine_state = in_mach::idle;

	// sw reset all devices on i2c bus 0
	// Wire.beginTransmission(0x00);
	// Wire.write(0x06);
	// Wire.endTransmission();
	// delayMicroseconds(500);

	// run all the ::begin sort of stuff
	_p_exp.connect(i2c_addrs::PCAL6408A_addr, Wire);
	if(_p_exp.begin() != true) device_begin_ok = false;
	if(!device_begin_ok){
		hds.println("failed to init p_ex");
		device_begin_ok = true;			// reset for next device
		all_device_begin_ok = false;
	}

	// setup the port_expander
	// write to the port first before turning outputs on
	// write to the port for enable PWM and LED off
  	_p_exp.writeExPort(P_EX_LED);
	// set port pins up
	{
		uint8_t ins = P_EX_BRD_ID_MASK | P_EX_BRD_REV_MASK;
		// (inv/norm, in/out, dr_str1,2, en/dis pupd, pu/pd, od/pushpull)
  		_p_exp.setupExPort(0x00, ins, 0x00, 0xF0, ins, ins, 0);
	}

	if(_pwm.init(i2c_addrs::PCA9685_addr, Wire) != true) device_begin_ok = false;
	if(!device_begin_ok){
		// ignore first failure... seems to have to try (get unknown error back)
		// hds.println("failed to init pwm");
		// device_begin_ok = true;			// reset for next device
		// all_device_begin_ok = false;
	}
	// now reset the pwm chip
	_pwm.reset();
	delay(250);

	// setup PWM for output
	// try reinitializing after reset....
	if(_pwm.init(i2c_addrs::PCA9685_addr, Wire) != true) device_begin_ok = false;
	if(!device_begin_ok){
		hds.println("failed to init pwm");
		device_begin_ok = true;			// reset for next device
		all_device_begin_ok = false;
	}

	_pwm.setOutputMode(pwm_out_types::kPwmOutTotemPole);
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

/// @brief will get the board id number (read only once during hot_diggity::begin)
/// @returns the stored board id.
uint8_t hot_diggity::getBoardId(){
	return _board_id;
}

/// @brief will get the board revision number (read only once during hot_diggity::begin)
/// @returns the stored board revision.
uint8_t hot_diggity::getBoardRev(){
	return _board_rev;
}

/// @brief sets the LED on the port expander to desired state
/// @param turn_on false = off, true = on
void hot_diggity::setExLed(bool turn_on){
	uint8_t val = _p_exp.readExPort();
	val = (turn_on) ? (val & ~P_EX_LED) : (val | P_EX_LED);
	_p_exp.writeExPort(val);
}

/// @brief toggles the LED on teh port expander
void hot_diggity::toggleExLed(){
	uint8_t val = _p_exp.readExPort();
	val ^= P_EX_LED;			// toggle the current state of the LED
	_p_exp.writeExPort(val);
}

/// @brief enables or disables the pwm outputs
/// @param turn_on true = enabled pwm outputs, false = disable pwm outputs
void hot_diggity::setPwmOutEn(bool turn_on){
	uint8_t val = _p_exp.readExPort();
	val = (turn_on) ? (val & ~P_EX_PWM_EN) : (val | P_EX_PWM_EN);
	_p_exp.writeExPort(val);
}
#pragma endregion

#pragma region heater_fx
// ----------------------- Heater funtions -------------------
/// @brief sets a selected heater's power level in mW
/// @param htr_num heater to set power level of (use pwm_info::pwm_sel enumeration)
/// @param power_mw integer value in milli-watts for heater power
void hot_diggity::setHeaterPower(pwm_info::pwm_sel htr_num, uint16_t power_mw){
	// make sure heater number is in range
	if(htr_num <= pwm_info::pwm_led){
		// see if there is enough power to grant the request
		int budget = getTotalHeaterPwr();
		budget -= _htr_pwr[htr_num];
		budget += power_mw;
		bool over_budget = (budget > 3000) ? true : false;
		bool over_power = (power_mw > 1000) ? true : false;
		if (over_budget || over_power){
			// send error message, do not execute
			if (over_budget) {
				hds.println("** ERROR -- requested over 3000mW total");
			}
			if(over_power){
				hds.println("** ERROR -- over 1000mW requested for heater");
			}
		} else {
		// else set the heater
			_htr_pwr[htr_num] = power_mw;
			uint16_t set_to = power_mw * 4;	// power goes in 250uW steps
			_pwm.setPwmOut(htr_num, set_to, false);
		}
	} else {
		// display an error
		hds.println("*** ERROR: not a valid heater number");
	}
}

/// @brief gives us the value that the heater is current set to (in mW)
/// @param htr_num heater to set power level of (use pwm_info::pwm_sel enumeration)
/// @returns heater setting in milli-watts
uint16_t hot_diggity::getHeaterPower(pwm_info::pwm_sel htr_num){
	uint16_t val = _pwm.getPwm(htr_num);
	if(val == pwm_errors::kPwmValReadError){
		hds.println("error reading heater: " + String(htr_num));
		return pwm_errors::kPwmValReadError;
	}
	val /= 4;	// need to change to mW from 250uW increments
	// *** TODO -- make sure this value matches _htr_pwr[htr_num] value
	return val;
}

/// @brief function to give us the sum of power in the heaters
/// @returns total heater power in milli-watts
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
/// @brief provides the temperature of a given sensor
/// @param temp_num number of the temperature sensor to read (use the tsense_info::tsense enumeration)
/// @returns floating point value of the temperature at the requested sensor in Celsius
float hot_diggity::getTemperature(tsense_info::tsense temp_num){
	return _temp_sense[temp_num].readTempC();
}

/// @brief provides the tmperature of a given sensor and passes back the relative time
/// @param temp_num number of the temperature sensor to read (use the tsense_info::tsense enumeration)
/// @param temp_time pointer to a location to put the current timestamp (millis())
/// @returns floating point value of the temperature at the requested sensor in Celsius
float hot_diggity::getTemperature(tsense_info::tsense temp_num, ulong *temp_time){
	float tval;
	tval = _temp_sense[temp_num].readTempC();
	*temp_time = millis();
	return tval;
}

#pragma endregion

#pragma region board_intfc
// --------------------- Interface funcitons -------------------
/// @brief will get the current board and revision infomration and send it back as a string
/// @returns a string with board id and board revision
String hot_diggity::getBoardInfo(){
	// for debug re-read the ID *** TODO *** remove geT_board_info() when hw is working
	get_board_info();
	String msg = "Board ID: " + String(getBoardId());
	msg += ", Board Rev: " + String(getBoardRev());
	return msg;
}

/// @brief sets an RGB value of the LEDs on V_HTR
/// @param red 8-bit red level
/// @param grn 8-bit green level
/// @param blu 8-bit blue level
void hot_diggity::setRgbValue(uint8_t red, uint8_t grn, uint8_t blu){
	// take the 8 bit vals, and write them as 12 bit values
	_pwm.setPwmOut(pwm_info::pwm_red, ((uint16_t)red) << 4, false);
	_pwm.setPwmOut(pwm_info::pwm_grn, ((uint16_t)grn) << 4, false);
	_pwm.setPwmOut(pwm_info::pwm_blu, ((uint16_t)blu) << 4, false);
}
#pragma endregion

#pragma region temp_polling
/// @brief sets the polling rate for the temp sensors (in msec)
/// @param rate number of milli-seconds between reading sensors
void hot_diggity::setPollRate(uint16_t rate){
	_poll_rate = rate;
}

/// @brief gets the polling rate for the temp sesnors (in msec)
/// @returns rate value; polling interval (in milli-seconds)
uint16_t hot_diggity::getPollRate(){
	return _poll_rate;
}

/// @brief turns periodic temperature polling on or off
/// @param state "ON" turns on temperature polling, anything else ("OFF") - stops temp polling
void hot_diggity::setPollingState(String state){
	if(state == "ON") {
		_poll_active = true;
		_last_poll_time = millis();
	}
	else {_poll_active = false;}
}

/// @brief returns the polling state as "ON" or "OFF"
/// @returns polling state as "ON" or "OFF"
String hot_diggity::getPollingState(){
	String ret_val = (_poll_active) ? "ON" : "OFF";
	return ret_val;
}

/// @brief returns the internal flag for whether or not we are polling
/// @returns boolean value false for "OFF", true for "ON"
bool hot_diggity::getPollingStateBool(){
	return _poll_active;
}

/// @brief when temperature polling is active, will determine if the specified
///			interval has elapsed, if so, it will get new readings and print them to the
///			hd serial interface
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

#pragma region self_test

/// @brief goes through and reports the status for all valid i2c addresses
void hot_diggity::scanI2cAddresses(){
	// make sure we can talk to parts or return false
	Wire.beginTransmission(i2c_addrs::PCAL6408A_addr);
	if(Wire.endTransmission() != 0){
		hds.println("Port Ex - Fail");
	} else {
		hds.println("Port Ex - OK");
	}
	Wire.beginTransmission(i2c_addrs::PCA9685_addr);
	if(Wire.endTransmission() != 0){
		hds.println("PWM chip - Fail");
	} else {
		hds.println("PWM chip - OK");
	}
	for(int i = (int)tsense_info::ctrl1; i <= (int)tsense_info::flexi5; i++){
		uint8_t t_addr = tsense_info::tsense_adr[i];
		Wire.beginTransmission(t_addr);
		if(Wire.endTransmission() != 0){
			hds.println(String(t_addr, HEX) + " - Fail");
		} else {
			hds.println(String(t_addr, HEX) + " - OK");
		}
	}
	for(int i = (int)tsense_info::flexi6; i <= (int)tsense_info::right2; i++){
		uint8_t t_addr = tsense_info::tsense_adr[i]	;
		Wire1.beginTransmission(t_addr);
		if(Wire1.endTransmission() != 0){
			hds.println(String(t_addr, HEX) + " - Fail");
		} else {
			hds.println(String(t_addr, HEX) + " - OK");
		}
	}
}

/// @brief will test the temp sensors and heaters on right board
void hot_diggity::rbTest(){
	float temp11, temp12;

	// turn off polling
	_poll_active = false;
	hds.println("Testing Right Board....");
	// turn on heater 7 and make sure temp 11 goes up
	hot_diggity::chk_t_rise(7, 11, 8000);
	// turn off heater 7 and make sure temp 11 goes down
	hot_diggity::chk_t_fall(7, 11, 16000);
	// turn on heater 8 and make sure temp 12 goes up
	hot_diggity::chk_t_rise(8, 12, 8000);
	// turn off heater 8 and make sure temp 12 goes down
	hot_diggity::chk_t_fall(8, 12, 16000) ;
	hds.println("Right Board Test Complete.");
}

/// @brief will test the temp sensors and heaters on control board
void hot_diggity::cbTest(){
	float temp0, temp1;
	// turn off polling
	_poll_active = false;
	hds.println("Testing Corner Board....");
	// turn on heater 0 and make sure temp 1 goes up
	hot_diggity::chk_t_rise(0, 1, 6000);
	// turn off heater 0 and make sure temp 1 goes down
	hot_diggity::chk_t_fall(0, 1, 15000);
	// turn on heater 1 and make sure temp 0 goes up
	hot_diggity::chk_t_rise(1, 0, 6000);
	// turn off heater 1 and make sure temp 0 goes down
	hot_diggity::chk_t_fall(1, 0, 15000);
	hds.println("Corner Board Test Complete.");
}

/// @brief will test the heaters with corresponding temp sensors on flex board
///			does not test the temp sensors that aren't near the heaters
void hot_diggity::fbTest(){
	// turn off polling
	_poll_active = false;
	hds.println("Testing Flex Board....");
	// turn on heaters 6, 2, 5, 3, 4 and make sure temps 2, 3, 6, 9, 10 respond
	// and turn off heaters 6, 2, 5, 3, 4 and make sure temps 2, 3, 6, 9, 10 respond
	hot_diggity::chk_t_rise(6, 2, 3000);
	hot_diggity::chk_t_fall(6, 2, 5000);
	hot_diggity::chk_t_rise(2, 3, 3000);
	hot_diggity::chk_t_fall(2, 3, 5000);
	hot_diggity::chk_t_rise(5, 6, 3000);
	hot_diggity::chk_t_fall(5, 6, 5000);
	hot_diggity::chk_t_rise(3, 9, 3000);
	hot_diggity::chk_t_fall(3, 9, 5000);
	hot_diggity::chk_t_rise(4, 10, 3000);
	hot_diggity::chk_t_fall(4, 10, 5000);
	hds.println("Flex Board Test Complete.");
}

#pragma endregion

#pragma region input_processing
/// @brief reads from the serial port 1 character at a time.  This keeps the Serial timeout
///			from sending a cmd before it is fully typed in
/// @returns if a commmand is ready for parsing (true or false), if true call getCommand()
bool hot_diggity::inputMachine(){
	bool retval = false;
	switch(hot_diggity::_input_machine_state){
		case in_mach::idle:
			if(hds.available()){
				int val = hds.read();
				if(val > -1){
					char c = (char)val;
					if((c == '\n') || (c == '\r')){
						// ignore it
						// hds.println("im: toss");
					} else {
						hot_diggity::_cmd_str += String(c);
						// hds.println("im first_ch: " + hot_diggity::_cmd_str);
						hot_diggity::_input_machine_state = in_mach::gather;
					}
				}
			}
			retval = false;
			break;
		case in_mach::gather:
			if(hds.available()){
				int val = hds.read();
				if(val > -1){
					char c = (char)val;
					if((c == '\n') || (c == '\r')){
						// hds.println("im - newline: cmd_rdy");
						// command is done
						hot_diggity::_input_machine_state = in_mach::cmd_ready;
					} else {
						// append to string
						hot_diggity::_cmd_str += String(c);
						// hds.println("im new_ch" + hot_diggity::_cmd_str);
					}
				}
			}
			retval = false;
		case in_mach::cmd_ready:
			retval = true;
			// stay here until the cmd is used
	}
	return retval;
}

/// @brief returns the current command and resets the inputMachine to idle or empty string
/// @returns current command or empty string (if command not ready)
String hot_diggity::getCommand(){
	String retval = "";
	if(hot_diggity::_input_machine_state == in_mach::cmd_ready){
		// send back the current command
		retval = hot_diggity::_cmd_str;
		// clear the _cmd_str location
		hot_diggity::_cmd_str = "";
		// start the machine over at idle
		hot_diggity::_input_machine_state = in_mach::idle;
	}
	return retval;
}

#pragma endregion

#pragma region private_stuff
//---------------- Private ------------------------------------
/// @brief function to read board information from port expander and store them
///		in the private instance variables _board_rev, and _board_id
void hot_diggity::get_board_info(){
	{
		uint8_t brd = _p_exp.readExPort();
		_board_rev = brd & P_EX_BRD_REV_MASK;
		_board_id = (brd & P_EX_BRD_ID_MASK) >> P_EX_BRD_ID_SHIFT;
	}
}

/// @brief will turn on heater (200 mW) and check for a temp increase (currently fixed at
///			3.0C for early out and 2.25C for timeout limit)
/// @param htr is the heater number to turn on
/// @param tsen is the temp sensor number to read
/// @param t_run is the max time (in msec) to run before declaring failure
void hot_diggity:: chk_t_rise(uint8_t htr, uint8_t tsen, unsigned long t_run){
	float t_base = hot_diggity::getTemperature((tsense_info::tsense)tsen);
	if(t_base < 0.0){
		hds.println("T" + String(tsen) + " failure: " + String(t_base));
		return;
	}
	float tval_fl = (float)t_run/1000.0;
	float t_temp = t_base;
	bool hit_target = false;
	unsigned long t_start = millis();
	unsigned long t_cur = t_start;
	// turn on heater and make sure temp goes up
	hds.println("check t:" + String(tsen) + " h: " + String(htr) + " --> heating (" + String(tval_fl) + " secs max)....");
	hot_diggity::setHeaterPower((pwm_info::pwm_sel)htr, 200);
	// check temp in loop (two thresholds 1 for faster than max time, other at max time)
	while(hit_target == false){
		t_temp = hot_diggity::getTemperature((tsense_info::tsense)tsen);
		t_cur = millis();
		if((t_cur - t_start) >= t_run){
			if((t_base + 2.25 ) < t_temp){
				// good
				String msg = "Heater " + String(htr) + ", Temp " +
					String(tsen) + " heat up - OK";
				hds.println(msg);
				hds.println("T[0]: " + String(t_base) + ", T[1]: " + String(t_temp));
			} else {
				// bad
				String msg = "*** FAIL *** Heater " + String(htr) + ", Temp " +
					String(tsen) + " heat up - FAIL";
				hds.println(msg);
				hds.println("T[0]: " + String(t_base) + ", T[1]: " + String(t_temp));
			}
			hit_target = true;		// just exit loop, hit_target may not be real
		} else {
			// if we haven't timed out we go with a higher temp diff
			if((t_base + 3.0 ) < t_temp){
				// good
				float t_tgt = (float)(millis() - t_start)/1000.0;
				String msg = "Heater " + String(htr) + ", Temp " +
					String(tsen) + " heat up - OK";
				msg += " -> " + String(t_tgt) + " secs";
				hds.println(msg);
				hds.println("T[0]: " + String(t_base) + ", T[1]: " + String(t_temp));
				hit_target = true;
			}
		}
	}
}

/// @brief will turn off heater and check for a temp decline (currently fixed at 2.5C for early
///			out and 1.75C for timeout limit)
/// @param htr is the heater number to turn off
/// @param tsen is the temp sensor number to read
/// @param t_run is the max time (in msec) to run before declaring failure
void hot_diggity:: chk_t_fall(uint8_t htr, uint8_t tsen, unsigned long t_run){
	float t_base = hot_diggity::getTemperature((tsense_info::tsense)tsen);
	if(t_base < 0.0){
		hds.println("T" + String(tsen) + " failure: " + String(t_base));
		return;
	}
	float tval_fl = (float)t_run/1000.0;
	float t_temp = t_base;
	bool hit_target = false;
	unsigned long t_start = millis();
	unsigned long t_cur = t_start;
	// turn on heater and make sure temp goes up
	hds.println("check t:" + String(tsen) + " h: " + String(htr) + " --> cooling (" + String(tval_fl) + " secs max)....");
	hot_diggity::setHeaterPower((pwm_info::pwm_sel)htr, 0);
	while(hit_target == false){
		// check temp
		t_temp = hot_diggity::getTemperature((tsense_info::tsense)tsen);
		t_cur = millis();
		if((t_cur - t_start) >= t_run){
			if((t_base - 1.75 ) > t_temp){
				// good
				String msg = "Heater " + String(htr) + ", Temp " +
					String(tsen) + " cool down - OK";
				hds.println(msg);
				hds.println("T[0]: " + String(t_base) + ", T[1]: " + String(t_temp));
			} else {
				// bad
				String msg = "*** FAIL *** Heater " + String(htr) + ", Temp " +
					String(tsen) + " cool down - FAIL";
				hds.println(msg);
				hds.println("T[0]: " + String(t_base) + ", T[1]: " + String(t_temp));
			}
			hit_target = true;
		} else {
			if((t_base - 2.5 ) > t_temp){
				float t_tgt = (float)(millis() - t_start)/1000.0;
				// good
				String msg = "Heater " + String(htr) + ", Temp " +
					String(tsen) + " cool down - OK";
				msg += " -> " + String(t_tgt) + " secs";
				hds.println(msg);
				hds.println("T[0]: " + String(t_base) + ", T[1]: " + String(t_temp));
				hit_target = true;
			}
		}
	}
}
#pragma endregion
