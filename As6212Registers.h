/************************************************************
  Brandon Williams
  AS6212 Register Addresses and Presets
  Creation Date: 07/15/20
  https://github.com/will2055/AS6212-Arduino-Library/src

  This file defines AS6212 registers and includes some configuration
  presets.

  Development environment specifics:
    IDE: Arduino 1.8.12
    Hardware Platform: Arduino Uno

  Special thanks to Madison Chodikov @ SparkFun Electronics
    for code examples from TMP117 Arduino Library
    (https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library)

  Distributed as-is; no warranty is given.

  2022-07-11 adding bit masks for config register - Tim Shirley

************************************************************/

#ifndef __AS6212_Registers_H__
#define __AS6212_Registers_H__

enum As6212Register{

  //Internal Register Addresses
  TVAL      =       0x0,    //Temperature Register
  CONFIG    =       0x1,    //Configuration Register
  TLOW      =       0x2,    //Low Temperature Threshold
  THIGH     =       0x3,    //High Temperature Threshold

  CONV_RATE_MSK =   0x00C0,
  SLEEP_MSK     =   0x0100,
  INTMODE_MSK   =   0x0200,
  POLARITY_MSK  =   0x0400,
  FAILS_MASK    =   0x0C00,
  SS_MASK       =   0x8000,

  //Helpful preset definitions for configuration register
  DEFAULTM  =       0x40A0,   //Default state
  SLEEPMODE =       0x41A0,   //Sleep Mode
  SLEEPSS   =       0xC1A0    //Sleep Mode Single Shot
};



#endif
