# Wireless Robotic Arm
## Team Fort Teen
## EC 535 Spring 2017
## Authors: Josh Wildey, Ian Hogan

### Directory Structure
* arm/Robotic_Arm: Source code for Arduino
  - Robotic_Arm folder required for Arduino IDE
* glove: Source code and Makefile for User Space program running on Gumstix
* gloveDriver: Source code and Makefile for Kernel Driver running on Gumstix
* i2cDriver: Source code and Makefile for 3rd and final attempt at getting I2C to work
* pi: Source code and Makefile for User Space program that acted as a command router running on the Raspberry Pi

### Build Instructions

Gumstix:

Raspberry Pi:

Arduino:
Connect Arduino Uno via USB to the computer
Load up Robot_Arm code (.ino file) to the arduino software
Select the correct Serial Port and the Board
Press the Upload button
Disconnect the Arduino from the computer and connect the USB to the Raspberry Pi
Connect all Robotic Arm wires to the Arduino, specified prior to the setup function of the Arduino Code
