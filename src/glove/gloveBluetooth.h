/**
 * \file    gloveBluetooth.h
 * \brief   This is the header file for gloveBluetooth.c
 * \author  Josh Wildey
 *          Ian Hogan
 *
 * Course   EC535
 *          Final Project
 *
 * Header file to define the Raspberry Pi bluetooth
 * address and function prototype(s) for gloveBluetooth.c.
 */

// Address Definitions
#define RASPBERRY_PI  "B8:27:EB:52:69:11"

// Functions
int btClientInit(void);
