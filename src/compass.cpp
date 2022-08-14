// Author: Kevin Gilman, Nafisul Khondaker, Ahmad Eladawy
// Date: May 12, 2022
// Assignment: Project GPS Tracking Robot
// Description: This file implements I2C Communication with HMC5883L
//----------------------------------------------------------------------//

#include "Compass.h"
#include "i2c.h"

void setupCompass() {
  //Wakeup I2C communication
  startI2C_Trans(SLA);
  Serial.println("Started Transmission");
  write(0x01); //Configuration Register B address on SLA
  write(0x20); //Gain=5
  stopI2C_Trans();
  Serial.println("Gain = 5");

  startI2C_Trans(SLA);
  write(0x02); //Mode Register address on SLA
  write(0x00); //Continuous-measurement mode
  Serial.println("Continuous measurement mode");
  stopI2C_Trans();
}

float* getCompassData() {
  unsigned char trueXYZ[6];

  Serial.println("ready to read data");
  startI2C_Trans(SLA);
  write(0x03);
  stopI2C_Trans();
  startI2C_Trans(SLA | 0x01);

  for(int i = 0; i < 6; i++) {
      if (i == 5) {
          trueXYZ[i] = i2c_readNak();
      }
      else {
          trueXYZ[i] = i2c_readAck();
      }
  }
  stopI2C_Trans();
  Serial.println("Done reading data");
  
  float xyz[3];

  // xyz[0] = (float)((trueXYZ[0] << 8) | trueXYZ[1]);
  // xyz[1] = (float)((trueXYZ[2] << 8) | trueXYZ[3]);
  // xyz[2] = (float)((trueXYZ[4] << 8) | trueXYZ[5]);

  xyz[0] = trueXYZ[1] + 120; //Adding offsets to LSB of raw data
  xyz[1] = trueXYZ[3] - 122;
  xyz[2] = trueXYZ[5];

  return xyz;
}