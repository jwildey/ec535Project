##########################################
#
# \file     Makefile
# \brief    Makefile for Lab 1 My Letters
# \author   Josh Wildey
#  Course   ENG EC535
#  Lab 1
#
#  This is the Makefile for Lab 1
#
##########################################

# Compiler and Compile Flags
EC535=/ad/eng/courses/ec/ec535
#CC=$(EC535)/arm-linux/bin/arm-linux-gcc
CC=arm-linux-gcc
CFLAGS=-Wall

# Application Name
SERVER_NAME=server
CLIENT_NAME=client

# Source/Header Files
SERVER_SRCS=bluetoothServer.c
SERVER_HDRS=

CLIENT_SRCS=bluetoothClient.c
CLIENT_HDRS=

# Include current dir for headers
INCS=-I. -I$(EC535)/bluez/include/

# Include necessary libraries
#LIBS=-L$(EC535)/gumstix/bluez/lib -lbluetooth 
LIBS=-L/ad/eng/courses/ec/ec535/gumstix/bluez/lib -lbluetooth 

# Intermediate 0bject files
SERVER_OBJS=$(SERVER_SRCS:.c=.o)
CLIENT_OBJS=$(CLIENT_SRCS:.c=.o)


##################
#  MAKE RULES
##################

# All/Default
all: $(SERVER_NAME) $(CLIENT_NAME)

# Server Application
$(SERVER_NAME): $(SERVER_OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# Server Application
$(CLIENT_NAME): $(CLIENT_OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# Server Intermediate Object Files
$(SERVER_OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(INCS) $< -o $@

# Client Intermediate Object Files
$(CLIENT_OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(INCS) $< -o $@

# Cleanup
clean:
	rm $(SERVER_OBJS) $(CLIENT_OBJS) $(SERVER_NAME) $(CLIENT_NAME)
