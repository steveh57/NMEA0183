/*
 Demo: NMEA0183 library. NMEA0183 -> NMEA2000
   Reads messages from NMEA0183 and forwards them to the N2k bus
   Also forwards all NMEA2000 bus messages to the PC (Serial)

 This example reads NMEA0183 messages from one serial port. It is possible
 to add more serial ports for having NMEA0183 combiner functionality.

 The messages, which will be handled has been defined on NMEA0183Handlers.cpp
 on NMEA0183Handlers variable initialization. So this does not automatically
 handle all NMEA0183 messages. If there is no handler for some message you need,
 you have to write handler for it and add it to the NMEA0183Handlers variable
 initialization. If you write new handlers, please after testing send them to me,
 so I can add them for others use.
*/

#define N2K_SOURCE 15

#include <Arduino.h>
#include <NMEA0183.h>



#define NMEA0183SourceGPSCompass 3
#define NMEA0183SourcePC 1

tNMEA0183 NMEA0183_s1;
tNMEA0183 NMEA0183_pc;

void setup() {

// Set up NMEA0183 serial streams

  NMEA0183_s1.Begin(&Serial,NMEA0183SourceGPSCompass, 4800);
  NMEA0183_pc.Begin(&Serial1, NMEA0183SourcePC, 115200 );

  //Set up low level forwarding

  NMEA0183_s1.SetForward(&NMEA0183_pc);  // Forward all messages received on S1 to PC

}

void loop() {
  NMEA0183_s1.ParseMessages();
  
}



