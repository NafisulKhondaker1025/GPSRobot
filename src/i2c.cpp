// Author: Kevin Gilman, Nafisul Khondaker, Ahmad Eladawy
// Date: May 12, 2022
// Assignment: Project GPS Tracking Robot
// Description: This file implements functions for I2C communication protocol
//----------------------------------------------------------------------//

#define wait_for_completion while(!(TWCR & (1 << TWINT))); //Defined macro for waiting for I2C completion 

#include "I2C.h"

//Initialize the I2C protocol
void initI2C() {
    PRR0 &= ~(1 << PRTWI); //Wake up I2C module on AT2560 power management register

    TWSR |= (1 << TWPS0); //Set Prescaler power = 1
    TWSR &= ~(1 << TWPS1);

    TWBR = 0xC0; //Bit rate generator = 10k (TWBR = 198)

    TWCR |= (1 << TWINT) | (1 << TWEN); //Enable two wire interface
}

//Initiate start condition and call slave device
void startI2C_Trans(unsigned char SLA) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); //Clear TWINT, initiate start condition and enable

    wait_for_completion;

    TWDR = SLA; //Slave address + write bit '0'

    TWCR = (1 << TWINT) | (1 << TWEN); //Trigger action by clearing flag and enabling TWI

    wait_for_completion;
}

//Initate stop condition for I2C transmission
void stopI2C_Trans() {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); //Clear TWINT, initate stop condition and enable
}


//Load data into I2C data register and transmits data
void write(unsigned char data) {
    TWDR = data; //Load data into TWDR
    TWCR = (1 << TWINT) |(1 << TWEN); //trigger action
    wait_for_completion;
}

//Read data from slave
void read_from(unsigned char SLA, unsigned char MEMADDRESS) {
    startI2C_Trans(SLA); //Start transmission and call SLA

    write(MEMADDRESS); //Access MEMADDRESS within SLA

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); //Restart to switch to read mode
    wait_for_completion;

    TWDR = (SLA << 1) | 0x01; //SLA + read bit '1'

    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); //Trigger with master sending ack
    wait_for_completion;

    TWCR = (1 << TWINT) | (1 << TWEN); //Trigger with master sending nack
    wait_for_completion;

    stopI2C_Trans(); //Stop transmission
}

//Returns TWDR
unsigned char read_data() {
    return TWDR;
}

unsigned char i2c_readAck()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    

    return TWDR;

}

unsigned char i2c_readNak()
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;

}