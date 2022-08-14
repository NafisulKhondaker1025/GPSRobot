// Author: Kevin Gilman, Nafisul Khondaker, Ahmad Eladawy
// Date: May 12, 2022
// Assignment: Project GPS Tracking Robot
// Description: This file implements I2C Communication with HMC5883L
//----------------------------------------------------------------------//

#ifndef Compass_h
#define Compass_h

#include <avr/io.h>
#include <Arduino.h>

#define SLA 0x3C //HMC5883L write address
#define xMSB 0x03 //Local address locations

void setupCompass();
float* getCompassData();

#endif