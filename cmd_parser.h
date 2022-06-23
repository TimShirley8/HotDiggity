// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#ifndef cmd_parser_h
#define cmd_parser_h
#include <Arduino.h>
#include "hot_diggity.h"

/// @namespace hd_commands
/// @brief list of valid commands to process
namespace hd_cmds
{
    /// @brief enumeration of th ecommands matching the command strings
    enum cmd_nums : uint8_t {
        set_htr = 0, get_htr, get_temp, get_board_info, get_fw_ver,
        tog_led3, set_led3, set_rgb, set_poll, polling, pwm_oe
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
        "PWMOE "            /// turn the pwm output on or off
    };
    /// @brief an actual count of the commands so we can iterate (String array doesn't have a .count poperty)
    const int cmd_count = pwm_oe + 1;
} // namespace hd_cmds

/// @class cmd_parser
/// @brief parses commands, executes them, and sends responses
class cmd_parser{
public:
    cmd_parser(void);
    bool begin(hot_diggity &hd);
    void reset(void);
    void parseCmd(String cmd_str);

private:
    hot_diggity *_hd;
    void parse_run_set_heater(String cmd_str);
    void parse_run_get_heater(String cmd_str);
    void parse_run_get_temp(String cmd_str);
    void parse_run_get_board(void);
    void parse_run_get_fw(void);
    void parse_run_toggle_led3(void);
    void parse_run_set_led3(String cmd_str);
    void parse_run_set_rgb(String cmd_str);
    void parse_run_set_poll(String cmd_str);
    void parse_run_poll_state(String cmd_str);
    void parse_run_pwm_state(String cmd_str);
    String strip_cmd_str(String in_str);
    String get_next_arg(String in_str, int start);
    String get_next_arg(String in_str, int start, int *found_at);
};

#endif
