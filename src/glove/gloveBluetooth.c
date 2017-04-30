/**
 * \file    gloveBluetooth.c
 * \brief   This module provides bluetooth initialization
 * \author  Josh Wildey
 *          Ian Hogan
 *
 * Course   EC535
 *          Final Project
 *
 * This file implements a method to initialize a bluetooth
 * connection with the raspberry pi.
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <error.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#include "gloveBluetooth.h"

/**
 * \brief Bluetooth Client Init
 *
 * This function initializes a client connection via bluetooth.
 *
 * \return <0 on error, integer File descriptor of BT connection otherwise
 */
int btClientInit()
{
    // Variables
    struct sockaddr_rc addr;
    int btFd, status;

    char dest[18] = RASPBERRY_PI;
    btFd = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    if (btFd < 0)
    {
        perror("Cannot allocate socket!");
        return -1;
    }

    // set conection params (who to connect to)
    addr.rc_family  = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba(dest, &addr.rc_bdaddr);

    // connect to server
    status = connect(btFd, (struct sockaddr *) &addr, sizeof(addr));

    if (status != 0)
    {
        perror("Bluetooth: cannot connect");
        return -1;
    }

    return btFd;
}

