/***************************************************************************//**
 *  @file
 *
 *  module for serial i/o
 *
 *  @copyright 2022-present Meta Platforms, Inc. and affiliates.
 *              Confidential and proprietary.
 *//***************************************************************************/

#ifndef HdSerial_H_
#define HdSerial_H_
#include <Arduino.h>

/// @class HdSerial
/// @brief uses the selected Serial/USB io for comm
class HdSerial{
public:
    enum ports : byte{
        ser0 = 0,
        serusb
    };
    HdSerial(void);
    HdSerial(ports connect_to);
    bool begin(ports connect_to);
    int println(String to_print);
    String readString(void);
    int read(void);
    int available(void);
private:
    ports _use_port;
};

#endif /* HdSerial_H_ */
