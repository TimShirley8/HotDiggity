/***************************************************************************//**
 *  @file
 *
 *  module for serial i/o
 *
 *  @copyright 2022-present Meta Platforms, Inc. and affiliates.
 *              Confidential and proprietary.
 *//***************************************************************************/

#include "HdSerial.h"

/// @brief empty constructor
HdSerial::HdSerial(){
    // do nothing
}

/// @brief Constructor that assigns the port for comms
/// @param [in] connect_to a port to send stuff to, select from ports enum
HdSerial::HdSerial(ports connect_to)
    : _use_port(connect_to)
{}

/// @brief with current HdSerial, assigns port to use
/// @param [in] connect_to port to send stuff to, select from ports enum
bool HdSerial::begin(ports connect_to){
    _use_port = connect_to;
}

/// @brief same as println for Serial/SerialUSB
/// @param [in] to_print string to print
/// @returns number of bytes sent to port
int HdSerial::println(String to_print){
    int out_num = 0;
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
String HdSerial::readString(){
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
int HdSerial::available(){
    int in_buff;
    if(_use_port == ser0){
        in_buff = Serial.available();
    }
    if(_use_port == serusb){
        in_buff = SerialUSB.available();
    }
    return in_buff;
}

/// @brief read a char from Serial/SerialUSB (val -1 is error, else it is a char)
/// @returns an int (-1 error, else char)
int HdSerial::read(){
    int rcvd = -1;
    if(_use_port == ser0){
        rcvd = Serial.read();
    }
    if(_use_port == serusb){
        rcvd = SerialUSB.read();
    }
    return rcvd;
}
