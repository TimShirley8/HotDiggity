// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#include "hd_serial.h"

/// @brief empty constructor
hd_serial::hd_serial(){
    // do nothing
}

/// @brief Constructor that assigns the port for comms
/// @param [in] port to send stuff to, select from ports enum
hd_serial::hd_serial(ports connect_to)
    : _use_port(connect_to)
{}

/// @brief with current hd_serial, assigns port to use
/// @param [in] port to send stuff to, select from ports enum
bool hd_serial::begin(ports connect_to){
    _use_port = connect_to;
}

/// @brief same as println for Serial/SerialUSB
/// @param [in] String to print
/// @returns number of bytes sent to port
size_t hd_serial::println(String to_print){
    size_t out_num = 0;
    if(_use_port == ser0){
        out_num = Serial.println(to_print);
    }
    if(_use_port == serusb){
        out_num = SerialUSB.println(to_print);
    }
    return out_num;
}

/// @brief same as readString for Serial/SerialUSB
/// @returns a string
String hd_serial::readString(){
    String rcvd;
    if(_use_port == ser0){
        rcvd = Serial.readString();
    }
    if(_use_port == serusb){
        rcvd = SerialUSB.readString();
    }
    return rcvd;
}

/// @brief checks to see if there is anything available in Serial/SerialUSB
/// @returns number of bytes available
int hd_serial::available(){
    int in_buff;
    if(_use_port == ser0){
        in_buff = Serial.available();
    }
    if(_use_port == serusb){
        in_buff = SerialUSB.available();
    }
    return in_buff;
}
