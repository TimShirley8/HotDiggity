// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#ifndef hd_serial_h
#define hd_serial_h
#include <Arduino.h>

/// @class hd_serial
/// @brief uses the selected Serial/USB io for comm
class hd_serial{
public:
    enum ports : byte{
        ser0 = 0,
        serusb
    };
    hd_serial(void);
    hd_serial(ports connect_to);
    bool begin(ports connect_to);
    size_t println(String to_print);
    String readString(void);
    int available(void);
private:
    ports _use_port;
};

#endif
