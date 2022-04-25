// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#ifndef hot_diggity_h
#define hot_diggity_h
#include <Arduino.h>
#include <Wire.h>
#include "pwm_driver.h"
#include "PCAL6408A.h"
#include "AS6212.h"
#include "hd_serial.h"

// related to i/o expander (PCAL6408A)
#define P_EX_LED			0x80
#define P_EX_PWM_EN			0x40
#define P_EX_BRD_ID_MASK	0x30
#define P_EX_BRD_ID_SHIFT	4
#define P_EX_BRD_REV_MASK	0x0F

// related to PWMs
namespace pwm_info{
	enum pwm_sel : uint8_t{
		pwm_left1 = 0,
		pwm_left2,
		pwm_flex1,
		pwm_flex2,
		pwm_flex3,
		pwm_flex4,
		pwm_flex5,
		pwm_right1,
		pwm_right2,
		pwm_nc1,
		pwm_nc2,
		pwm_nc3,
		pwm_grn,
		pwm_red,
		pwm_blu,
		pwm_led
	};
}
namespace tsense_info{
	enum tsense : uint8_t{
		ctrl1 = 0,
		ctrl2,
		flexi1,
		flexi2,
		flexi3,
		flexi4,
		flexi5,
		flexi6,
		flexi7,
		flexi8,
		flexi9,
		right1,
		right2
	};
	const uint8_t ctrl1_adr = 0x44;
	const uint8_t ctrl2_adr = 0x45;
	const uint8_t flexi1_adr = 0x46;
	const uint8_t flexi2_adr = 0x47;
	const uint8_t flexi3_adr = 0x48;
	const uint8_t flexi4_adr = 0x49;
	const uint8_t flexi5_adr = 0x4A;
	const uint8_t flexi6_adr = 0x44;
	const uint8_t flexi7_adr = 0x45;
	const uint8_t flexi8_adr = 0x46;
	const uint8_t flexi9_adr = 0x47;
	const uint8_t right1_adr = 0x48;
	const uint8_t right2_adr = 0x49;

	const uint8_t tsense_adr[] = {ctrl1_adr, ctrl2_adr, flexi1_adr, flexi2_adr,
		flexi3_adr, flexi4_adr, flexi5_adr, flexi6_adr, flexi7_adr, flexi8_adr,
		flexi9_adr, right1_adr, right2_adr};
}

// ------------------- Other Component Addresses --------------
namespace i2c_addrs{
	const uint8_t PCAL6408A_addr = 0x20;
	const uint8_t PCA9685_addr   = 0x40;
}

/*
 * Ideally this class will represent the higher level aggregated hardware:
 *    - 13 AS6212 temp sensors
 *    -  1 I/O port expander (PCAL6408A)
 *         - enable PWM controller output
 *         - read board id, revision
 *         - enable 3V3 LED
 *    -  1 PWM controller (PCA9685)
 *         - 9 heaters pwm settings of 0-4095 (0 to 1024mW in 250uW steps)
 *         - 1 RGB LED (on V_HTR)
 *         - 1 Green LED (relative to 3V3)
 */

class hot_diggity {
public:
	hd_serial hds;

	hot_diggity();
	bool begin();
	uint8_t getBoardId(void);
	uint8_t getBoardRev(void);
	String getBoardInfo(void);
	void setExLed(bool turn_on);
	void toggleExLed(void);
	void setPwmOutEn(bool turn_on);
	void setHeaterPower(pwm_info::pwm_sel htr_num, uint16_t power_mw);
	uint16_t getHeaterPower(pwm_info::pwm_sel htr_num);
	uint16_t getTotalHeaterPwr(void);		// *** TODO ***
	float getTemperature(tsense_info::tsense temp_num);
	float getTemperature(tsense_info::tsense temp_num, ulong *temp_time);
	void setRgbValue(uint8_t red, uint8_t grn, uint8_t blu);
	void setPollRate(uint16_t rate);
	uint16_t getPollRate(void);
	void setPollingState(String state);
	String getPollingState(void);
	bool getPollingStateBool(void);
	void checkPoll(void);

private:
	AS6212 _temp_sense[13];
	pwm_driver _pwm;			// PCA9685
	PCAL6408A _p_exp;
	uint8_t	_board_id;
	uint8_t _board_rev;
	uint16_t _htr_pwr[pwm_info::pwm_right2 + 1];
	uint16_t _poll_rate;
	bool _poll_active;
	ulong _last_poll_time;
	float _t_sense_data[tsense_info::right2 + 1];
	ulong _t_sense_time[tsense_info::right2 + 1];

	void get_board_info(void); // retrieve board id and rev to private vars
};

#endif
