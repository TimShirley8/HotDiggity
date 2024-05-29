/***************************************************************************//**
 *  @file
 *
 *  main code for the Hot Diggity project
 *
 *  @copyright 2022-present Meta Platforms, Inc. and affiliates.
 *              Confidential and proprietary.
*//***************************************************************************/

#ifndef HotDiggity_H_
#define HotDiggity_H_
#include <Arduino.h>
#include <Wire.h>
#include "Pca9685.h"
#include "Pcal6408a.h"
#include "As6212.h"
#include "HdSerial.h"

// related to i/o expander (PCAL6408A)
#define P_EX_LED			0x80
#define P_EX_PWM_EN			0x40
#define P_EX_BRD_ID_MASK	0x30
#define P_EX_BRD_ID_SHIFT	4
#define P_EX_BRD_REV_MASK	0x0F

// related to PWMs
namespace pwm_info{
	enum pwm_sel : uint8_t{
		pwm_flex0 = 0,
		pwm_flex1,
		pwm_flex2,	// pinout changes for ckt routing
		pwm_flex3,
		pwm_flex4,
		pwm_flex5,
		pwm_flex6,
		pwm_flex7,
		pwm_flex8,
		pwm_flex9,
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
		flexi1 = 0,
		flexi2,
		flexi3,
		flexi4,
		flexi5,
		flexi6,
		flexi7,
		flexi8,
		flexi9,
		flexi10,
		flexi11,
		flexi12,
    flexi13,
    flexi14,
		ctrl1
	};




	const uint8_t flexi1_adr  = 0x44; // 0x44 Temp Sense FB1
	const uint8_t flexi2_adr  = 0x45; // 0x45 Temp Sense FB2
	const uint8_t flexi3_adr  = 0x46; // 0x46 Temp Sense FB3
	const uint8_t flexi4_adr  = 0x47; // 0x47 Temp Sense FB4
	const uint8_t flexi5_adr  = 0x48; // 0x48 Temp Sense FB5
	const uint8_t flexi6_adr  = 0x49; // 0x49 Temp Sense FB6
	const uint8_t flexi7_adr  = 0x4A; // 0x4A Temp Sense FB7
	const uint8_t flexi8_adr  = 0x44; // 0x44 Temp Sense FB8
	const uint8_t flexi9_adr  = 0x45; // 0x45 Temp Sense FB9
	const uint8_t flexi10_adr = 0x46; // 0x46 Temp Sense FB10
	const uint8_t flexi11_adr = 0x47; // 0x47 Temp Sense FB11
	const uint8_t flexi12_adr = 0x48; // 0x48 Temp Sense FB12
  const uint8_t flexi13_adr = 0x49; // 0x49 Temp Sense FB13
  const uint8_t flexi14_adr = 0x4A; // 0x4A Temp Sense FB14
	const uint8_t ctrl1_adr   = 0x4B; // 0x4B Temp Sense CB1

	const uint8_t tsense_adr[] = {flexi1_adr, flexi2_adr, flexi3_adr, flexi4_adr,
  flexi5_adr, flexi6_adr, flexi7_adr, flexi8_adr, flexi9_adr, flexi10_adr,
   flexi11_adr, flexi12_adr, flexi13_adr, flexi14_adr, ctrl1_adr};
}

// ------------------- Other Component Addresses --------------
namespace i2c_addrs{
	const uint8_t PCAL6408A_addr = 0x20; // 0x20 Port Expander
	const uint8_t PCA9685_addr   = 0x40; // 0x40 PWM Controller
}

namespace in_mach{
	enum inm_states : uint8_t{
		idle,
		gather,
		cmd_ready,
		hold
	};
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
	HdSerial hds;

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
	void scanI2cAddresses(void);
void rbTest(void);
	void cbTest(void);
	void fbTest(void);
	bool inputMachine(void);
	String getCommand(void);

private:
	As6212 _temp_sense[13];
	PwmDriver _pwm;			// PCA9685
	Pcal6408a _p_exp;
	uint8_t	_board_id;
	uint8_t _board_rev;
	uint16_t _htr_pwr[pwm_info::pwm_flex9 + 1];
	uint16_t _poll_rate;
	bool _poll_active;
	ulong _last_poll_time;
	float _t_sense_data[tsense_info::ctrl1 + 1];
	ulong _t_sense_time[tsense_info::ctrl1 + 1];
	String _cmd_str;
	in_mach::inm_states _input_machine_state;
	void chk_t_rise(uint8_t htr, uint8_t tsen, unsigned long t_run);
	void chk_t_fall(uint8_t htr, uint8_t tsen, unsigned long t_run);

	void get_board_info(void); // retrieve board id and rev to private vars
};

#endif /* HotDiggity_H_ */
