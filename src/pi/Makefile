CC=gcc

CFLAGS=-Wall

# App Name
APPNAME=piRouter

# Source/Header Files
SRCS=main.c 
HDRS=

# Include Directorys
INC=-I.

# Include libs
LIBS=-lbluetooth

# Intermediate Object Files
OBJS=$(SRCS:.c=.o)

# Make Rules

# All/Default
all: $(APPNAME)

# Application
$(APPNAME): $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# Intermediate Files
$(OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $(INC) $< -o $@

# Cleanup
clean:
	rm $(OBJS) $(APPNAME)
