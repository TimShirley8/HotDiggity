/***************************************************************************//**
 *  @file
 *
 *  Parses commands from the serial port and executes them
 *
 *  @copyright 2022-present Meta Platforms, Inc. and affiliates.
 *              Confidential and proprietary.
 *//***************************************************************************/

#include "CmdParser.h"
#include "HotDiggity.h"
#include "HdSerial.h"

/// @brief empty constructor
CmdParser::CmdParser(){
    // do nothing
}

/// @brief establishes connection to the hot_diggity instance
/// @param pointer to the hot_diggity instance
/// @returns true (ideally it would return meaningful status)
bool CmdParser::begin(hot_diggity &hd)
{
    _hd = &hd;
    return true;
}

/// @brief resets the cmd parser (actually does nothing)
void CmdParser::reset(){
    // do some sort of reset thing here
}

/// @brief parses serial commands and executes them
/// @param cmd_str command string to be parsed
void CmdParser::parseCmd(String cmd_str){
    bool cmd_found = false;
    // trim white space and make all caps
    cmd_str.trim();
    cmd_str.toUpperCase();
    // loop through all the commands and look for a valid command
    for(int i = 0; i < hd_cmds::cmd_count; i++){
        if(cmd_str.startsWith(hd_cmds::cmds[i])){
            // we know the command now parse it
            cmd_found = true;
            switch(i){
                case hd_cmds::set_htr:
                    // do the set heater thing
                    _hd->hds.println("set heater");
                    parseRunSetHeater(cmd_str);
                    break;
                case hd_cmds::get_htr:
                    // do the get heater thing
                    _hd->hds.println("get heater");
                    parseRunGetHeater(cmd_str);
                    break;
                case hd_cmds::get_temp:
                    // do the get temp thing
                    _hd->hds.println("get temp");
                    parseRunGetTemp(cmd_str);
                    break;
                case hd_cmds::get_board_info:
                    // get board information
                    _hd->hds.println("get board info");
                    parseRunGetBoard();
                    break;
                case hd_cmds::get_fw_ver:
                    // get the fw version (if any)
                    _hd->hds.println("get fw version");
                    break;
                case hd_cmds::tog_led3:
                    // toggle the led 3
                    parseRunToggleLed3();
                    break;
                case hd_cmds::set_led3:
                    // set the led3 led
                    parseRunsetLed3(cmd_str);
                    break;
                case hd_cmds::set_rgb:
                    // set the rgb led
                    parseRunSetRgb(cmd_str);
                    break;
                case hd_cmds::set_poll:
                    parseRunSetPoll(cmd_str);
                    break;
                case hd_cmds::polling:
                    parseRunPollState(cmd_str);
                    break;
                case hd_cmds::pwm_oe:
                    parseRunPwmState(cmd_str);
                    break;
                case hd_cmds::i2c_scan:
                    parseRunI2cScan();
                    break;
                case hd_cmds::test_all:
                    parseRunTestAll();
                    break;
                case hd_cmds::fb_test:
                    parseRunFbTest();
                    break;
                default:
                    cmd_found = false;
                    break;
            }
        }
    }
    if(cmd_found != true){
        _hd->hds.println("command not found: [" + cmd_str + "]");
    }
}

/// @brief will use the command string to set the heater power level
/// @param cmd_str string containing the heater number and value to set heater to
void CmdParser::parseRunSetHeater(String cmd_str){
    // strip command and parse the rest
    String tmp = stripCmdStr(cmd_str);
    int idx = 0;
    String my_str = getNextArg(tmp, 0, &idx);
    uint8_t htr_num = (uint8_t)my_str.toInt();
    _hd->hds.println("htr #" + String(htr_num));
    my_str = getNextArg(tmp, idx + 1, &idx);
    uint16_t heat = (uint16_t)my_str.toInt();
    _hd->hds.println("pwm_val = " + String(heat));

    // attempt to set the heater
    _hd->setHeaterPower((pwm_info::pwm_sel)htr_num, heat);
}

/// @brief will use the command string to return (via serial) the heater power level
/// @param cmd_str string containing heater number to get power level of
void CmdParser::parseRunGetHeater(String cmd_str){
    String msg = "";
    // strip command and parse the rest
    String tmp = stripCmdStr(cmd_str);
    String val_str = getNextArg(tmp, 0);
    pwm_info::pwm_sel pwm_num = (pwm_info::pwm_sel)val_str.toInt();

    // now get the value
    uint16_t htr_pwr = _hd->getHeaterPower(pwm_num);
    if(htr_pwr != pwm_errors::kPwmValReadError){
        msg += "heater is set to " + String(htr_pwr) + "mW";
        _hd->hds.println(msg);
    }
}

/// @brief will use the command string to return (via serial) the temperature from a sensor
/// @param cmd_str command string that contains the temp sensor number to get temp from
void CmdParser::parseRunGetTemp(String cmd_str){
    String msg = "";
    // strip command and parse the rest
    String tmp = stripCmdStr(cmd_str);
    String temp_num_str = getNextArg(tmp, 0);
    tsense_info::tsense temp_num = (tsense_info::tsense)temp_num_str.toInt();

    float val = _hd->getTemperature(temp_num);
    msg += "temperature = " + String(val) + " C";
    _hd->hds.println(msg);
}

/// @brief will return (via serial) the board id information and a bizarre message
void CmdParser::parseRunGetBoard(){
    _hd->hds.println(_hd->getBoardInfo());
    _hd->hds.println("chicken pot pie?");
}

/// @brief when there is a fw version this will return (via serial) the fw version
void CmdParser::parseRunGetFw(){
    String msg = "";
    // *** TODO

    _hd->hds.println(msg);
}

/// @brief will toggle led3 (3V3 led on port expander) state
void CmdParser::parseRunToggleLed3(){
    _hd->toggleExLed();
}

/// @brief will set led3 to the state requested via the command string
/// @param cmd_str string containg the commadn and ON/OFF to set the led
void CmdParser::parseRunsetLed3(String cmd_str){
    String msg = "";
    // strip command and parse the rest
    String tmp = stripCmdStr(cmd_str);
    String set_to = getNextArg(tmp, 0);
    if(set_to.equals("ON")){
        _hd->setExLed(true);
        msg += "set Ex Led On";
    } else if (set_to.equals("OFF")){
        _hd->setExLed(false);
        msg += "set Ex Led Off";
    } else {
        // invalide command
        msg += "invalid command to set Led";
    }
    _hd->hds.println(msg);
}

/// @brief will set the rgb led (on v_htr) to the command string specified 24b (R, G, B) value
/// @param cmd_str string with 8bit R, G, and B values (space seperator)
void CmdParser::parseRunSetRgb(String cmd_str){
    String msg = "";
    String tmp = stripCmdStr(cmd_str);
    int idx = 0;
    String red_str = getNextArg(tmp, 0, &idx);
    String grn_str = getNextArg(tmp, idx + 1, &idx);
    String blu_str = getNextArg(tmp, idx + 1, &idx);
    uint8_t r_val = (uint8_t)(red_str.toInt());
    uint8_t g_val = (uint8_t)(grn_str.toInt());
    uint8_t b_val = (uint8_t)(blu_str.toInt());
    _hd->setRgbValue(r_val, g_val, b_val);

    _hd->hds.println(msg);
}

/// @brief will set the polling interval for temp sensors
/// @param cmd_str string containing the desired polling rate in msec
void CmdParser::parseRunSetPoll(String cmd_str){
    String tmp = stripCmdStr(cmd_str);
    String rate_str = getNextArg(tmp, 0);
    _hd->setPollRate((uint16_t)rate_str.toInt());
}

/// @brief will set the polling state for temp sensors
/// @param cmd_str string containing the designed state ON or OFF
void CmdParser::parseRunPollState(String cmd_str){
    String tmp = stripCmdStr(cmd_str);
    String action_str = getNextArg(tmp, 0);
    _hd->setPollingState(action_str);
}

/// @brief will set the pwm output enable state
/// @param cmd_str string containing the designed state ON or OFF
void CmdParser::parseRunPwmState(String cmd_str){
    String msg = "";
    // strip command and parse the rest
    String tmp = stripCmdStr(cmd_str);
    String set_to = getNextArg(tmp, 0);
    if(set_to.equals("ON")){
        _hd->setPwmOutEn(true);
        msg += "set Ex PWM OE On";
    } else if (set_to.equals("OFF")){
        _hd->setPwmOutEn(false);
        msg += "set Ex PWM OE Off";
    } else {
        // invalide command
        msg += "invalid command to set PWM OE";
    }
    _hd->hds.println(msg);}

/// @brief scans valid i2c addresses and returns status of each
void CmdParser::parseRunI2cScan(){
    _hd->scanI2cAddresses();
}

/// @brief tests all the heaters and temp sensors
void CmdParser::parseRunTestAll(){
    parseRunFbTest();
}

/// @brief tests all the heaters and temp sensors on flex board
void CmdParser::parseRunFbTest(){
    _hd->fbTest();
}

/// @brief helper function to remove the command bit of the command string
/// @param cmd_str string to have the command removed from
/// @returns string with just the parameters
String CmdParser::stripCmdStr(String in_str){
    int start = 0;
    int found = in_str.indexOf(" ");
    int end  = in_str.length();
    return in_str.substring(found+1, end);
}

/// @brief helper function to get the next argument in the command string
/// @param in_str the string we wish to get an argument from
/// @param start offset in the string to parse from
/// @returns the string containing the next argument (from start)
String CmdParser::getNextArg(String in_str, int start){
    int found = in_str.indexOf(" ", start);
    int total = in_str.length();
    if(-1 == found){
        // return from start to the end
        found = total;
    }
    return(in_str.substring(start, found));
}

/// @brief helper function to get the next in the command string
/// @param in_str the string we wish to get an argument from
/// @param start offset in the string to parse from
/// @param found_at [out] pointer to int; returns the location of the seperator
/// @returns the string containing the next argument (from start)
String CmdParser::getNextArg(String in_str, int start, int *found_at){
    int found = in_str.indexOf(" ", start);
    int total = in_str.length();
    if(-1 == found){
        // return from start to the end
        found = total;
    }
    *found_at = found;
    return(in_str.substring(start, found));
}
