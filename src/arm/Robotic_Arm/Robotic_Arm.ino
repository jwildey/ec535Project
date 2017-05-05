// AUTHOR Ian Hogan
// 5-1-2017
#include <Servo.h>// import servo library

// UART Data from the Raspberry Pi //
const int TOTAL_BYTES = 5;  // the total bytes of the message
byte Header = 0;            // initializing byte indicating start of message
byte X = 0;                 // indicates X-axis 0,1,2 nothing, up, down
byte Y = 0;                 // indicates Y-axis 0,1,2 nothing, left, right
byte Z = 0;                 // indicates Z-axis 0,1,2 nothing, up, down
byte GRIPPER = 0;           // indicates gripper motion 0,1 open, close

// !!!Servo.h kills Pins 9 & 10 !!! //
// Due to this use A0 and A1 as digitalOutput

// Gripper requires 5V to run and does not need a motor driver
Servo gripper;        // create servo object
int pos = 140;         // gripper servo position (80 to 140) Start at 140 (closed)
int gripperStatus;    // Global flag to indicate whether gripper is open(0) or closed(1)
                      // This will be deleted for later application

// Potentiometers limiting the movement of two motors
int elbowPot = A4;     
const int elbowHigh = 520;   // upper limit if wrist is down 585
const int elbowLow = 220;    // lower limit 250 
                             // upper limit if wrist is up 870 
int elbowValue = 0;          // variable to store incoming value

int wristPot = A5;    
const int wristLow = 230;      // lower limit   
const int wristHigh = 813;     // upper limit
int wristValue = 0;          // variable to store incoming value

// connect motor controller pins to Arduino digital pins
// enA, in1-4, enB are labelled pins on the controller
// enA and enB require PWM outputs
// Shoulder Controller
int enA_1 = 11;
int in1_1 = 13;
int in2_1 = 12;

// Elbow Controller
int enB_1 = 6;
int in3_1 = 7;
int in4_1 = A0;

// Wrist Controller
int enA_2 = 5;
int in1_2 = A1;
int in2_2 = 8;

void setup() {
  Serial.begin(9600);   // Serial Baud 9600 matches UART RPI

  gripper.attach(3);   // attaches the servo on pin 3 to gripper object
  gripper.write(pos);  // Range of gripper is 80 to 140 (140 being closed)
  gripperStatus = 0;   // Gripper is open

  // set all the motor control pins to outputs
  pinMode(enA_1, OUTPUT);
  pinMode(enB_1, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);
  pinMode(in3_1, OUTPUT);
  pinMode(in4_1, OUTPUT); 
  pinMode(enA_2, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(in2_2, OUTPUT);
}

void loop() {
  if ( Serial.available() >= TOTAL_BYTES){  // listens until entire msg recieved
      Header = Serial.read();         // Read in the header byte
      X = Serial.read();              // Read in X byte
      Y = Serial.read();              // Read in Y byte
      Z = Serial.read();              // Read in Z byte
      GRIPPER = Serial.read();        // Read in GRIPPER byte 

      // Debug //
      Serial.write(Header);
      Serial.write(X);
      Serial.write(Y);
      Serial.write(Z);
      Serial.write(GRIPPER);
  }

  // Control for the X direction (ELBOW)
  if ( X == 1 ) {
      elbowUp();    // move elbow motor in the up direction
  } else if ( X == 2 ) {
      elbowDown();  // move elbow motor in the down direction
  } else {
    // Do nothing
  }
  
  // Control for the Y direction (SHOULDER)
  if ( Y == 1 ) {
      shoulderLeft();    // move shoulder motor in the left direction
  } else if ( Y == 2 ) {
      shoulderRight();  // move shoulder motor in the right direction
  } else {
    // Do nothing
  }
  
  // Control for the Z direction (WRIST)  
  if ( Z == 1 ) {
      wristUp();    // move wrist motor in the up direction
  } else if ( Z == 2 ) {
      wristDown();  // move wrist motor in the down direction
  } else {
    // Do nothing
  }
  
  // Control for the Gripper Servo Motor
  if ( GRIPPER == 1 ) {
      gripClose();    // close gripper
  } else {
      gripOpen();     // open gripper
  }
  
}

//test potentiometers on robotic arm for limits
/*void loop() {
  wristValue = analogRead(wristPot);
  elbowValue = analogRead(elbowPot);
  Serial.print("elbowValue: ");
  Serial.println(elbowValue);
  delay(500);
}*/

// Gripper Range Test
// Used to find the limits of the servo motor
// that controls the servo, used for test
void grip_range_test() {
    for (int i = 80; i <140; i ++){
       Serial.println(i);
        gripper.write(i);
        delay(15);
    }
}

// Min Motor speed is ~100
// Will engage for 50ms and go low/stop
// Calibrations have been made to match a single
// movement between both directions of the motor
// due to the motor favoring 1 direction over the other
void wristUp() {
    wristValue = analogRead(wristPot);
    elbowValue = analogRead(elbowPot);
    // checks if the location of the wrist or elbow is at a limit
    if ( wristValue < 460 && elbowValue > 220 ) {
        digitalWrite(in1_2, HIGH);
        digitalWrite(in2_2, LOW);
        analogWrite(enA_2, 200); 
        delay(50);
        digitalWrite(in1_2, LOW);
        digitalWrite(in2_2, LOW);
    }
}

void wristDown() {
    wristValue = analogRead(wristPot);
    elbowValue = analogRead(elbowPot);
    // checks if the location of the wrist or elbow is at a limit
    if ( wristValue > 230 ) {
        digitalWrite(in1_2, LOW);
        digitalWrite(in2_2, HIGH);
        analogWrite(enA_2, 200);
        delay(50);
        digitalWrite(in1_2, LOW);
        digitalWrite(in2_2, LOW);
    }
}

void elbowUp() {
    wristValue = analogRead(wristPot);
    elbowValue = analogRead(elbowPot);
  // checks if the location of the wrist or elbow is at a limit
    if ( elbowValue < 940 || wristValue < 840 ) {
        digitalWrite(in3_1, HIGH);
        digitalWrite(in4_1, LOW);
        analogWrite(enB_1, 200);
        delay(100);
        digitalWrite(in3_1, LOW);
        digitalWrite(in4_1, LOW);
    //}
}

void elbowDown() {
    elbowValue = analogRead(elbowPot);
    wristValue = analogRead(wristPot);
    // checks if the location of the wrist or elbow is at a limit
    if ( elbowValue > 230 ) {
        digitalWrite(in3_1, LOW);
        digitalWrite(in4_1, HIGH);
        analogWrite(enB_1, 200);
        delay(100);
        digitalWrite(in3_1, LOW);
        digitalWrite(in4_1, LOW);
    }
}

void shoulderLeft() {
    digitalWrite(in1_1, HIGH);
    digitalWrite(in2_1, LOW);
    analogWrite(enA_1, 200);
    delay(100);
    digitalWrite(in1_1, LOW);
    digitalWrite(in2_1, LOW);
}

void shoulderRight() {
    digitalWrite(in1_1, LOW);
    digitalWrite(in2_1, HIGH);
    analogWrite(enA_1, 200);
    delay(100);
    digitalWrite(in1_1, LOW);
    digitalWrite(in2_1, LOW);
}

// Gripper numbers are derived from the gripper test above
// the servos are set at their maximum permitted stepper value
void gripOpen() {
    if (gripperStatus != 0) {
        gripperStatus = 0;
        gripper.write(80);
        delay(15);
    }
}

void gripClose() {
    if (gripperStatus == 0 ) {
        gripperStatus = 1;
        gripper.write(140);
        delay(15);
    }
}

