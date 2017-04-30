#include <stdio.h>
#include <stdint.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

typedef struct data {
	uint8_t x;
	uint8_t y;
	uint8_t z;
	uint8_t g;
} data_t;

int uart_init()
{
	//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	int fd = -1;

	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	//uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
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

void uart_close(int uart)
{
	if (uart)
	{
		close(uart);
	}
}


int uart_write(int uart, const void *buf, size_t nbyte)
{
	int count;

	if (uart != -1)
	{

		count = write(uart, buf, nbyte); //Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
		else
		{
			printf("UART wrote %d bytes\n", count);
		}
	}

	return count;
}


int main()
{
	int uartConn;
	int result;
	uint8_t header = 0x0A;
	data_t d;
	int i;

	uint8_t buf[20];

	d.x = 0;
	d.y = 1;
	d.z = 2;
	d.g = 0;


	uartConn = uart_init();

	while (1)
	{

		printf("Send H: %02x x: %02x y: %02x z: %02x g: %02x\n", header, d.x, d.y, d.z, d.g);
		write(uartConn, &header, 1);
		write(uartConn, &d, 4);

		result = read(uartConn, buf, sizeof(buf));
		printf("Received ");
		for (i = 0; i < result; i++)
		{
			printf("%02x ", buf[i]);
		}
		printf("\n");

		d.x = (d.x + 1) % 3;
		d.y = (d.y + 1) % 3;
		d.z = (d.z + 1) % 3;
		d.g = (d.g + 1) % 3;

		sleep(1);
	}

	uart_close(uartConn);

	return 0;
}
