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
  Wire1.begin();        // I2C1
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
  hd.checkPoll();
  // look for serial input
  if(hd.hds.available() > 0){
    String cmd_str = hd.hds.readString();
    if (cmd_str.length() > 0){
      // output response to serial input
      unsigned long hd_time = millis();
      cp.parseCmd(cmd_str);
      cmd_str = "";
      hd.hds.println("at time: " + String(hd_time));
    }
  }
  cp.reset();
}

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
