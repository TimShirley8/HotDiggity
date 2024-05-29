/***************************************************************************//**
 *  @file
 *
 *  Parses and executes commands from the serial interface
 *
 *  @copyright 2022-present Meta Platforms, Inc. and affiliates.
 *              Confidential and proprietary.
 *//***************************************************************************/

#ifndef CmdParser_H_
#define CmdParser_H_
#include <Arduino.h>
#include "HotDiggity.h"

/// @namespace hd_commands
/// @brief list of valid commands to process
namespace hd_cmds
{
    /// @brief enumeration of th ecommands matching the command strings
    enum cmd_nums : uint8_t {
        set_htr = 0, get_htr, get_temp, get_board_info, get_fw_ver,
        tog_led3, set_led3, set_rgb, set_poll, polling, pwm_oe, i2c_scan,
        test_all, fb_test, cmd_count
    };
    /// @brief actual command strings to parse
    const String cmds[] = {
        "SETHTR ",          /// will set heater in mW (next arg)
        "GETHTR ",          /// will get the heater val in mW
        "GETTEMP ",         /// returns temp of temp sense # in C
        "GETBRDINFO",       /// returns a string of board id and rev
        "GETFWVER",         /// would return FW version if I was versioning
        "TOGLED3",          /// toggle the LED on the expander port
        "SETLED3 ",         /// set the led on the expander port
        "SETRGB ",          /// set the rgb led (on v_htr) pwm values
        "SETPOLL ",         /// sets the temperature polling interval
        "POLLING ",         /// start or stop temperature polling
        "PWMOE ",           /// turn the pwm output on or off
        "I2CSCAN",          /// scan all valid i2c addresses and report
        "TESTALL",          /// Test Heaters and Temp sensors
        "FBTEST"            /// Test Heaters and Temp Sensors on Flex Board
    };
    /// @brief an actual count of the commands so we can iterate (String array doesn't have a .count poperty)
    //const int cmd_count = fb_test + 1;    // see enum above :)
} // namespace hd_cmds

/// @class CmdParser
/// @brief parses commands, executes them, and sends responses
class CmdParser{
public:
    CmdParser(void);
    bool begin(hot_diggity &hd);
    void reset(void);
    void parseCmd(String cmd_str);

private:
    hot_diggity *_hd;
    void parseRunSetHeater(String cmd_str);
    void parseRunGetHeater(String cmd_str);
    void parseRunGetTemp(String cmd_str);
    void parseRunGetBoard(void);
    void parseRunGetFw(void);
    void parseRunToggleLed3(void);
    void parseRunsetLed3(String cmd_str);
    void parseRunSetRgb(String cmd_str);
    void parseRunSetPoll(String cmd_str);
    void parseRunPollState(String cmd_str);
    void parseRunPwmState(String cmd_str);
    void parseRunI2cScan(void);
    void parseRunTestAll(void);
    void parseRunFbTest(void);
    String stripCmdStr(String in_str);
    String getNextArg(String in_str, int start);
    String getNextArg(String in_str, int start, int *found_at);
};

#endif
