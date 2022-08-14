// Author: Kevin Gilman, Nafisul Khondaker, Ahmad Eladawy
// Date: May 12, 2022
// Assignment: Project GPS Tracking Robot
// Description: This file implements functions for onboard GPS module
//----------------------------------------------------------------------//

#ifndef GPS_H
#define GPS_H

void setupGPS(Adafruit_GPS gps);
GeoLoc gpsdump(Adafruit_GPS, int);

#endif