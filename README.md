# Wireless Robotic Arm
## Team Fort Teen
## EC 535 Spring 2017
## Authors: Josh Wildey, Ian Hogan
https://github.com/jwildey/ec535Project.git

### Directory Structure
* src/arm/Robotic_Arm: Source code for Arduino
  - Robotic_Arm folder required for Arduino IDE
* src/glove: Source code and Makefile for User Space program running on Gumstix
* src/gloveDriver: Source code and Makefile for Kernel Driver running on Gumstix
* src/i2cDriver: Source code and Makefile for 3rd and final attempt at getting I2C to work
* src/pi: Source code and Makefile for User Space program that acted as a command router running on the Raspberry Pi

### Build Instructions

#### Gumstix:
Dependencies: 
- BlueZ
- Arm-Linux-gcc compiler for Gumstix
Build:
1. Kernel Driver:
```
source /ad/eng/courses/ec/ec535/bashrc_ec535
cd src/gloveDriver/
make
```
2. User Space program:
```
source /ad/eng/courses/ec/ec535/bashrc_ec535
cd src/glove/
make
```
3. Transferring program to Gumstix
  - Connect Gumstix to computer via USB serial connection
  - Open minicom
  - Log into Gumstix with 'root' as username and 'gumstix' as the password
  - Type 'rz -bZ' into command prompt
  - Press Ctrl + A + S
  - This will bring up a sub window of the host computer's file heirarchy
  - Using double space to enter folders, navigate to src/glove and select glove by pressing space bar once
  - This should highlight the glove file
  - Navigate to src/gloveDriver and select gloveDriver.ko by pressing space bar once
  - Hit enter to initiate the transfer
4. Make device file for Kernel Driver
```
mknod /dev/gloveDriver c 61 0
```
5. Insert Kernel Driver
```
insmod gloveDriver.ko
```
  

#### Raspberry Pi:
Dependencies: 
- BlueZ
Build:
1. Transfer files to Raspberry Pi either by SCP or connecting Raspberry Pi to internet and cloning Git Repository
2. Navigate to src/pi
3. Make the piRouter
```
make
```

#### Arduino:
1. Connect Arduino Uno via USB to the computer
2. Load up Robot_Arm code (.ino file) to the arduino software
3. Select the correct Serial Port and the Board
4. Press the Upload button
5. Disconnect the Arduino from the computer and connect the USB to the Raspberry Pi
6. Connect all Robotic Arm wires to the Arduino, specified prior to the setup function of the Arduino Code

### Run Instructions
1. Ensure all connections are made according to circuit diagrams provided in Project Report
2. Ensure Arduino has been flashed with the latest code
3. Connect Arduino to Raspberry Pi via USB Cable
4. Run piRouter program
```
./piRouter
```
5. Run glove user space program
```
./glove
```
