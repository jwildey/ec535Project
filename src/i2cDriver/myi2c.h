/**
 * \file    myi2c.h
 * \brief   This is the header for the i2c driver
 * \author  Josh Wildey
 * \author  Ian Hogan
 * 
 * Course   ENG EC535
 */

//////////////////////////////////
// I2C Commands to use with ioctl
//////////////////////////////////

// Initialize
// 1 Byte Slave address should be written as the argument
#define I2C_INIT 1

// Write 1 Byte
#define I2C_W1B  2

// Read 1 Byte
#define I2C_R1B  3
