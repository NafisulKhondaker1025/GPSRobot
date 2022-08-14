// Author: Kevin Gilman, Nafisul Khondaker, Ahmad Eladawy
// Date: May 12, 2022
// Assignment: Project GPS Tracking Robot
// Description: This file implements functions for motor control
//----------------------------------------------------------------------//

#include "motorControl.h"
#include <avr/io.h>
#include "Arduino.h"

void initMotors() {
    DDRB |= (1 << PORTB4);          //pin 10 motor A enable pin
    DDRH |= (1 << PORTH6);          //pin 9 for motor B enable pin
    DDRC |= (1 << PORTC7);          //pin 30 for motor B input 1
    DDRA |= (1 << PORTA6);          //pin 28 for motor B input 2
    DDRA |= (1 << PORTA4);          //pin 26 for motor A input 1
    DDRA |= (1 << PORTA2);          //pin 24 for motor A input 2      
}

void setSpeedA(int speed) {
  PORTA &= ~(1 << PORTA4);          //set input 1 to low
  PORTA |= (1 << PORTA2);           //set input 2 to high
  
  // set speed to 200 out of possible range 0~255 for duty cycle
  analogWrite(10, speed);           //set the speed in the enable pin
}

void setSpeedB(int speed) {
  PORTC &= ~(1 << PORTC7);
  PORTA |= (1 << PORTA6);
  
  // set speed to 200 out of possible range 0~255 for duty cycle
  analogWrite(9, speed);            //set the speed in the enable pin
}

void setSpeed(int speed)
{
  // this function will run the motors in both directions at a fixed speed
  // turn on motor A
  setSpeedA(speed);

  // turn on motor B
  setSpeedB(speed);
}

//Stop moving robot
void stop() {
  // now turn off motors
  PORTA &= ~(1 << PORTA4);          //set input 1 to low
  PORTA &= ~(1 << PORTA2);          //set input 2 to low
  PORTC &= ~(1 << PORTC7);          
  PORTA &= ~(1 << PORTA6);
}