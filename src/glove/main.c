/**
 * \file    glove.c
 * \brief   This module is the user level program to read the state
 *          of the glove driver and send info to the arm
 * \author  Josh Wildey
 *          Ian Hogan
 *
 * Course   EC535
 *          Final Project
 *
 * User-level program to communicate with a kernel module capable
 * of reading the device file, packing the info into a structure
 * and sending it via bluetooth to the arm and gripper controller.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "gloveBluetooth.h"

#define MAX_MSG_LEN 8

typedef struct data {
    uint8_t x;
    uint8_t y;
    uint8_t z;
    uint8_t g;
}data_t;


/**
 * \brief Main entrance to program
 *
 * This program provides a user level application to
 * interface with /dev/glove and to send info to arm.
 *
 * \param argc - number of arguments
 * \param **argv - pointer to arguments
 * \return <0 is error, 0 success
 */
int main(int argc, char **argv)
{
    // Variables
    int btConn;
    int wr_res;
    FILE *pFile;
    char line[MAX_MSG_LEN];
    data_t gloveData;
    int x, y, z, g;

    // Establish Bluetooth Connection
    btConn = btClientInit();
    if (btConn < 0)
    {
        printf("Error connecting to bluetooth...\n");
        return -1;
    }

    // Loop at desired rate until user stops process
    while (1)
    {
        // Open glove driver file
        pFile = fopen("/dev/gloveDriver", "r");
        if (pFile == NULL)
        {
            fputs("glove driver module isn't loaded\n", stderr);
            return -1;
        }

        // Read glove driver file
        while (fgets(line, MAX_MSG_LEN, pFile) != NULL)
        {
            sscanf(line, "%x %d %d %d", &x, &y, &z, &g);
            gloveData.x = (uint8_t) x;
            gloveData.y = (uint8_t) y;
            gloveData.z = (uint8_t) z;
            gloveData.g = (uint8_t) g;
        }

        printf("X: %d Y: %d Z: %d G: %d\n", gloveData.x, gloveData.y, gloveData.z, gloveData.g);

        // Close file
        fclose(pFile);

        // Send info via bluetooth
        wr_res = write(btConn, &gloveData, sizeof(data_t));
        if (wr_res < 0)
        {
            fputs("error writing data to bluetooth connection...\n", stderr);
        }

        // Sleep for desired rate
        //usleep(100000);
        sleep(1);
    }

    return 0;
}
