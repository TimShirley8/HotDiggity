// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
// Author: Tim Shirley

#include <Arduino.h>
#include <Wire.h>   // gives us the I2C library
#include "hot_diggity.h"
#include "cmd_parser.h"

cmd_parser cp;
hot_diggity hd;

void setup() {
  bool device_init_ok = true;
  // put your setup code here, to run once:
  Wire.begin();         // I2C0
  Wire.setClock(400000);
  Wire1.begin();        // I2C1
  Wire1.setClock(400000);
  Serial.begin(115200);  // serial for I/O
  SerialUSB.begin(2000000); // serial for fast I/O
  while(!SerialUSB);        // wait for it to enumerate
  // select the serial i/o method
  hd.hds.begin(hd_serial::ports::serusb);     // using USB CDC Serial
  //hd.hds.begin(hd_serial::ports::ser0);       // using serial debug port
  if(hd.begin() == false){
    SerialUSB.println("encountered begin issue....");
  } else {
    String msg = "Board id: " + String(hd.getBoardId());
    msg += ", Rev: " + String(hd.getBoardRev()) + " up and running!!!";
    SerialUSB.println(msg);
  }
  cp.begin(hd);
  //Serial.println("hello Tim - you rock!!!!");
  //SerialUSB.println("this should work @ 2Mbaud");

  //hd.hds.println("is this working - SerialUSB");

}

void loop() {
  String tot_cmd = "";
  hd.checkPoll();
  // look for serial input
  if(hd.inputMachine()){
    String cmd = hd.getCommand();
    if(cmd.length() > 0){
      // hd.hds.println("[" + cmd + "]");
      cp.parseCmd(cmd);
    }
  }
  /*
  if(hd.hds.available() > 0){
    String ser_str = hd.hds.readString();
    tot_cmd += ser_str;
    // now see if the string is terminated with cr/lf
    // if(tot_cmd.endsWith("\r\n") || tot_cmd.endsWith("\n\r")){
      // output response to serial input
      unsigned long hd_time = millis();
      cp.parseCmd(tot_cmd);
      tot_cmd = "";
      hd.hds.println("at time: " + String(hd_time));
    // }
  }
  */
  // cp.reset();
}

#if 0
/// @brief will read the serial port until it sees CR, LF
///       it will append all new chars to tot_cmd until CR, LF
/// @returns whether or not a CR, LF is found
String getSerialInput(void){
  static String the_cmd = "";
  static bool new_cmd = true;
  if(hd.hds.available() > 0){
    String ser_str = hd.hds.readString();
    the_cmd += ser_str;
    if ((the_cmd.endsWith('\n')) || (the_cmd.endsWith('\r'))){
      return the_cmd;
    }
  }
  return "";
}

String getSerialChars(void){
  static String baked_cmd = "";
  static bool start_cmd = true;
  if(hd.hds.available() > 0){
    int val = hd.hds.read();
    if(val >= 0){
      if(start_cmd && (val == '\n' || val == '\r')){
        // just toss this (extra \n or \r char)
      } else {
        start_cmd = false;
        baked_cmd += String((char)val);
      }
    }
  }

}
#endif

/* =================================================================================
   *** TODO list *****

   - make all i2c transactions id success or failure
   - add ability to time stamp transactions (get or set)
   - stagger start pwms so that they aren't all switching on at the same time

  *** DONE list ******
   - use the SerialUSB instead of the slow UART/debug connection
   - add a temp scan command? Something that prints all temp sensors periodically
    -- adds a timestamp to each tsense... not with I2C timeouts (no HW) it takes
      nearly 600msec to gather 13 tsense.  So poll rates faster than 1 sec will
      be problematic....
   - fix scan rate (there is a lot of uncertainty in the loop time - almost 1 sec)
   - add SerialUSB or Serial.available() to the hd_serial class
   - cleanup direct call to SerialUSB in pwm_driver (debug thing)


=================================================================================== */
