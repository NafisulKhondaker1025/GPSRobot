// Author: Kevin Gilman, Nafisul Khondaker, Ahmad Eladawy
// Date: May 12, 2022
// Assignment: Project GPS Tracking Robot
// Description: This file implements functions for onboard the GPS Module
//----------------------------------------------------------------------//

#include <Adafruit_GPS.h>
#include "Definitions.h"
#include <GPS.h>


//Setup code adopted from Adafruit GPS library
void setupGPS(Adafruit_GPS gps) {
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  delay(5000);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  gps.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out

  // Request updates on antenna status, comment out to keep quiet
  gps.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}

//Our own code to parse GPS data into lat and lon
// change to receive robot GPS data
// Get and process GPS data
GeoLoc gpsdump(Adafruit_GPS gps,int timer) {
  float flat, flon;
  GeoLoc robotLoc;
  robotLoc.lat = 0;
  robotLoc.lon = 0;
  
  timer = millis();
  while (millis() - timer < 2000) {
    char c = gps.read();

    // if a sentence is received, we can check the checksum, parse it...
    if (gps.newNMEAreceived()) {

      if (!gps.parse(gps.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return robotLoc;  // we can fail to parse a sentence in which case we should just wait for another
    }
  }
    Serial.print("Robot Location: ");
    flat = ((int)gps.latitude / 100) + (((int)gps.latitude % 100) / 60.0) + ((gps.latitude - (int)gps.latitude) / 60.0);
    flon = ((int)gps.longitude / -100) - (((int)gps.longitude % 100) / 60.0) - ((gps.longitude - (int)gps.longitude) / 60.0);
    robotLoc.lat = flat;
    robotLoc.lon = flon;
    Serial.print(flat, 7);
    Serial.print(", ");
    Serial.println(flon, 7);

  return robotLoc;
}
