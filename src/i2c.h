// Author: Kevin Gilman, Nafisul Khondaker, Ahmad Eladawy
// Date: May 12, 2022
// Assignment: Project GPS Tracking Robot
// Description: This file implements functions for I2C communication protocol
//----------------------------------------------------------------------//

#ifndef I2C_H
#define I2C_H
#include <avr/io.h>
#include <Arduino.h>

void initI2C();
void startI2C_Trans(unsigned char slave);
void stopI2C_Trans();
void write(unsigned char data);
void read_from(unsigned char slave, unsigned char MEMADDRESS);
unsigned char read_data();
unsigned char i2c_readNak();
unsigned char i2c_readAck();


#endif

