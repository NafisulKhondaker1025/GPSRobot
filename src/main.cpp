// Author: Kevin Gilman, Nafisul Khondaker, Ahmad Eladawy
// Date: May 12, 2022
// Assignment: Project GPS Tracking Robot
// Description: This file implements the main code for robot to function and navigate
//----------------------------------------------------------------------//

//macros adopted from Dabble library for Bluetooth comunication with phone
#define CUSTOM_SETTINGS
#define INCLUDE_SENSOR_MODULE

// Imports
#include <Adafruit_GPS.h>
#include <Dabble.h>
#include <SoftwareSerial.h>
#include "Definitions.h"
#include "compass.h"
#include "GPS.h"
#include "i2c.h"
#include "motorControl.h"

// Define GPS Module and ports for RS232 Communication
// Using Adafruit GPS library
SoftwareSerial mySerial(12, 13);
Adafruit_GPS gps(&mySerial);

int timer = millis();

//function to get gps data from phone using Dabble library
GeoLoc get_PhoneGPS_data()
{
  GeoLoc mobileLoc;

  mobileLoc.lat = Sensor.getGPSlatitude();
  mobileLoc.lon = Sensor.getGPSlongitude();

  Serial.print("Longitude: ");
  Serial.print(mobileLoc.lon, 12);
  Serial.print('\t');
  Serial.print("Latitude: ");
  Serial.println(mobileLoc.lat, 12);
  Serial.println();

  return mobileLoc;
}

//Function to calculate bearing from phone and robot coordinates
float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

//Function to calculate distance from phone and robot coordinates
float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

//Function to calculate heading from Compass data from the magnetometer on the robot
float geoHeading() {
  
  float* xyz;

  xyz = getCompassData();
  // Calculate heading
  float heading = atan2(xyz[1], xyz[0]);
  Serial.print("X = "); Serial.print(xyz[0]);
  Serial.print("     Y = "); Serial.println(xyz[1]);

  //Declination Angle for my our latitude and longitude obtained from http://magnetic-declination.com/
  //and converted to radians
  float declinationAngle = (9.0 + (23.0 / 60.0)) / (180 / M_PI); //values are for Tucson
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 
  // Map to -180 - 180
  while (headingDegrees < -180) {
    headingDegrees += 360;
  }
  while (headingDegrees >  180) {
    headingDegrees -= 360;
  }

Serial.print("                       Heading degrees = ");
Serial.println(headingDegrees);

  return headingDegrees;
}

void drive(int distance, float turn) {
  int fullSpeed = 200;
  int stopSpeed = 0;

  // drive to location
  int s = fullSpeed;
  if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
  }
  
  int autoThrottle = constrain(s, stopSpeed, fullSpeed);
  autoThrottle = 250;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("turn: ");
  Serial.println(t);
  
  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t < 0) {
    autoSteerB = t_modifier;
  } else if (t > 0){
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);
  
  setSpeedA(speedA);
  setSpeedB(speedB);

  Serial.print("speedA: "); Serial.println(speedA);
  Serial.print("speedB: "); Serial.println(speedB);
}

void driveTo(struct GeoLoc &loc, int timeout) {
  GeoLoc robotLoc = gpsdump(gps, timer);

  // if (robotLoc.lat != 0 && robotLoc.lon != 0 && enabled) {
  if (robotLoc.lat != 0 && robotLoc.lon != 0) {
    float d = 0;
    //Start move loop here
    do {
      robotLoc = gpsdump(gps, timer);
      
      d = geoDistance(robotLoc, loc);
      float t = geoBearing(robotLoc, loc) - geoHeading();
      
      Serial.print("Distance: ");
      Serial.println(geoDistance(robotLoc, loc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(robotLoc, loc));
      
      
      drive(d, t);
      timeout -= 1;
    } while (d > 3.0 && timeout>0);

    stop();
  }
}

// Start Tracking the Phone
void Track() {
  Serial.println("Received Phone GPS: ");
  GeoLoc phoneLoc = get_PhoneGPS_data();

  driveTo(phoneLoc, 18);
}

//setup function
void setup()
{
  Serial.begin(9600);     //baud rate.

  //Compass
  setupCompass();

  // GPS
  setupGPS(gps);

  //Bluetooth
  Dabble.begin(9600);     //Serial3 pins for Mega.

  //motors
  initMotors();

}

//loop function
void loop()
{ 
  Dabble.processInput(); 
  Track();
}

