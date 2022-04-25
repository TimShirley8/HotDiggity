#include "cmd_parser.h"
#include "hot_diggity.h"
#include "hd_serial.h"

cmd_parser::cmd_parser(){
    // do nothing
}


bool cmd_parser::begin(hot_diggity &hd)
{
    _hd = &hd;
    return true;
}

/// resets the cmd parser (actually does nothing)
void cmd_parser::reset(){
    // do some sort of reset thing here
}

void cmd_parser::parseCmd(String cmd_str){
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
                    parse_run_set_heater(cmd_str);
                    break;
                case hd_cmds::get_htr:
                    // do the get heater thing
                    _hd->hds.println("get heater");
                    parse_run_get_heater(cmd_str);
                    break;
                case hd_cmds::get_temp:
                    // do the get temp thing
                    _hd->hds.println("get temp");
                    parse_run_get_temp(cmd_str);
                    break;
                case hd_cmds::get_board_info:
                    // get board information
                    _hd->hds.println("get board info");
                    parse_run_get_board();
                    break;
                case hd_cmds::get_fw_ver:
                    // get the fw version (if any)
                    _hd->hds.println("get fw version");
                    break;
                case hd_cmds::tog_led3:
                    // toggle the led 3
                    parse_run_toggle_led3();
                    break;
                case hd_cmds::set_led3:
                    // set the led3 led
                    parse_run_set_led3(cmd_str);
                    break;
                case hd_cmds::set_rgb:
                    // set the rgb led
                    parse_run_set_rgb(cmd_str);
                    break;
                case hd_cmds::set_poll:
                    parse_run_set_poll(cmd_str);
                    break;
                case hd_cmds::polling:
                    parse_run_poll_state(cmd_str);
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

void cmd_parser::parse_run_set_heater(String cmd_str){
    // strip command and parse the rest
    String tmp = strip_cmd_str(cmd_str);
    int idx = 0;
    String my_str = get_next_arg(tmp, 0, &idx);
    uint8_t htr_num = (uint8_t)my_str.toInt();
    _hd->hds.println("htr #" + String(htr_num));
    my_str = get_next_arg(tmp, idx + 1, &idx);
    uint16_t heat = (uint16_t)my_str.toInt();
    _hd->hds.println("pwm_val = " + String(heat));

    // attempt to set the heater
    _hd->setHeaterPower((pwm_info::pwm_sel)htr_num, heat);
}

void cmd_parser::parse_run_get_heater(String cmd_str){
    String msg = "";
    // strip command and parse the rest
    String tmp = strip_cmd_str(cmd_str);
    String val_str = get_next_arg(tmp, 0);
    pwm_info::pwm_sel pwm_num = (pwm_info::pwm_sel)val_str.toInt();

    // now get the value
    uint16_t htr_pwr = _hd->getHeaterPower(pwm_num);
    if(htr_pwr != PWM_VAL_ERROR){
        msg += "heater is set to " + String(htr_pwr) + "mW";
        _hd->hds.println(msg);
    }
}

void cmd_parser::parse_run_get_temp(String cmd_str){
    String msg = "";
    // strip command and parse the rest
    String tmp = strip_cmd_str(cmd_str);
    String temp_num_str = get_next_arg(tmp, 0);
    tsense_info::tsense temp_num = (tsense_info::tsense)temp_num_str.toInt();

    float val = _hd->getTemperature(temp_num);
    msg += "temperature = " + String(val) + " C";
    _hd->hds.println(msg);
}

void cmd_parser::parse_run_get_board(){
    _hd->hds.println(_hd->getBoardInfo());
    _hd->hds.println("chicken pot pie?");
}

void cmd_parser::parse_run_get_fw(){
    String msg = "";
    // *** TODO

    _hd->hds.println(msg);
}

void cmd_parser::parse_run_toggle_led3(){
    _hd->toggleExLed();
}

void cmd_parser::parse_run_set_led3(String cmd_str){
    String msg = "";
    // strip command and parse the rest
    String tmp = strip_cmd_str(cmd_str);
    String set_to = get_next_arg(tmp, 0);
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

void cmd_parser::parse_run_set_rgb(String cmd_str){
    String msg = "";
    String tmp = strip_cmd_str(cmd_str);
    int idx = 0;
    String red_str = get_next_arg(tmp, 0, &idx);
    String grn_str = get_next_arg(tmp, idx + 1, &idx);
    String blu_str = get_next_arg(tmp, idx + 1, &idx);
    uint8_t r_val = (uint8_t)(red_str.toInt());
    uint8_t g_val = (uint8_t)(grn_str.toInt());
    uint8_t b_val = (uint8_t)(blu_str.toInt());
    _hd->setRgbValue(r_val, g_val, b_val);

    _hd->hds.println(msg);
}

void cmd_parser::parse_run_set_poll(String cmd_str){
    String tmp = strip_cmd_str(cmd_str);
    String rate_str = get_next_arg(tmp, 0);
    _hd->setPollRate((uint16_t)rate_str.toInt());
}

void cmd_parser::parse_run_poll_state(String cmd_str){
    String tmp = strip_cmd_str(cmd_str);
    String action_str = get_next_arg(tmp, 0);
    _hd->setPollingState(action_str);
}

String cmd_parser::strip_cmd_str(String in_str){
    int start = 0;
    int found = in_str.indexOf(" ");
    int end  = in_str.length();
    return in_str.substring(found+1, end);
}

String cmd_parser::get_next_arg(String in_str, int start){
    int found = in_str.indexOf(" ", start);
    int total = in_str.length();
    if(-1 == found){
        // return from start to the end
        found = total;
    }
    return(in_str.substring(start, found));
}

String cmd_parser::get_next_arg(String in_str, int start, int *found_at){
    int found = in_str.indexOf(" ", start);
    int total = in_str.length();
    if(-1 == found){
        // return from start to the end
        found = total;
    }
    *found_at = found;
    return(in_str.substring(start, found));
}
