/**
 * \file    main.c
 * \brief   This is the source for the Raspberry Pi data routing
 *          software.
 * \author  Josh Wildey
 *          Ian Hogan
 *
 * Course   EC535
 *          Final Project
 *
 * This file implements a bluetooth connection initialization,
 * a USB serial connection and a loop to listen for data on the
 * bluetooth connection which then adds a header and writes said
 * data onto the USB serial connection.
 */

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

// Command Data Structure
typedef struct data {
	uint8_t x;
	uint8_t y;
	uint8_t z;
	uint8_t g;
} data_t;


/**
 * \brief Bluetooth Client Init
 *
 * This function initializes a client connection via bluetooth.
 *
 * \return <0 on error, integer File descriptor of BT connection otherwise
 */
int initBT()
{
    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
	char buf[1024] = { 0 };    
	
	int s;
	int client = -1;
    socklen_t opt = sizeof(rem_addr);

    // allocate socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // bind socket to port 1 of the first available 
    // local bluetooth adapter
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = *BDADDR_ANY;
    loc_addr.rc_channel = (uint8_t) 1;
    bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

    // put socket into listening mode
    listen(s, 1);

    // accept one connection
    client = accept(s, (struct sockaddr *)&rem_addr, &opt);

    ba2str( &rem_addr.rc_bdaddr, buf );
    fprintf(stderr, "accepted connection from %s\n", buf);
    memset(buf, 0, sizeof(buf));

    return client;
}


/**
 * \brief USB Serial Connection Init
 *
 * This function initializes a USB Serial Connection using /dev/ttyACM0
 *
 * \return <0 on error, integer File descriptor of USB Serial Connection
 */
int initUSB()
{
	int fd = -1;

	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);		//Open in non blocking read/write mode
	if (fd == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;

	tcgetattr(fd, &options);

	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);

	return fd;
}


/**
 * \brief Main entrance to program
 *
 * This program provides a means to route data command
 * from the Gumstix controller to the Arduino.
 *
 * \param argc - number of arguments
 * \param **argv - pointer to arguments
 * \return <0 is error, 0 success
 */
int main(int argc, char **argv)
{
	// Variables
	int btConn;
	int usbConn;
	int bytesRead, bytesWritten;
	data_t gloveData;
	uint8_t hdr = 0x0A;
	char buf[1024];
	
	// Set up Bluetooth Connection
	btConn = initBT();
	if (btConn < 0)
	{
		printf("Error connecting to bluetooth...\n");
		return -1;
	}

	// Open USB Serial Port
	usbConn = initUSB();
	if (usbConn < 0)
	{
		printf("Error opening usb serial port...\n");
	}

	while (1)
	{
		// Read data from Bluetooth
		bytesRead = read(btConn, &gloveData, sizeof(data_t));
		if (bytesRead > 0)
		{
			bytesWritten = write(usbConn, &hdr, 1);
			if (bytesWritten < 0)
			{
				printf("Error writing header to usb serial port...\n");
			}

			bytesWritten = write(usbConn, &gloveData, sizeof(data_t));
			if (bytesWritten < 0)
			{
				printf("Error writing glove data to usb serial port...\n");
			}
			
			bytesRead = read(usbConn, buf, sizeof(buf));
			if (bytesRead > 0)
			{
				printf("Received: %s ", buf);
			}
		}

	}


	return 0;
}




