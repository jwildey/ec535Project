##########################################
#
# \file     Makefile
# \brief    Makefile for Lab 1 My Letters
# \author   Josh Wildey
#  Course   ENG EC535
#  Project - Glove portion
#
#  This is the Makefile for the glove of
#  the project
#
##########################################

# Compiler and Compile Flags
PROJ_ROOT=.
EC535=/ad/eng/courses/ec/ec535
CC=arm-linux-gcc
CFLAGS=-Wall

# Application Name
BIN_NAME=glove

# Source/Header Files
SRCS=main.c \
	 gloveBluetooth.c
HDRS=gloveBluetooth.h

# Include current dir for headers
INCS=-I$(PROJ_ROOT) \
	 -I$(EC535)/gumstix/bluez/include

# Include Necessary Libraries
LIBS=-L$(EC535)/gumstix/bluez/lib -lbluetooth

# Intermediate 0bject files
OBJS=$(SRCS:.c=.o)


##################
#  MAKE RULES
##################

# All/Default
all: $(BIN_NAME)

# Application
$(BIN_NAME): $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# Intermediate Object Files
$(OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(INCS) $< -o $@

# Cleanup
clean:
	rm $(OBJS) $(BIN_NAME)
