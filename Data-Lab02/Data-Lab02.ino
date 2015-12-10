/*Data_Lab02.ino
  Modified: Peter Heath and Matthew Schack and Data the robot 12/8/15

***************************************************
  This code uses the IR remote to program the robot *
  to move according to various different patterns   *
***************************************************

  Based on the Arduino IRremote Library and ArduinoRobot Library


  created 3 December 2015
  By P. Heath and M. Schack and Data the robot
*/
// include the necessary libraries
#include <IRremote.h>
#include <ArduinoRobot.h>
#include <math.h>

// Define a few commands from your remote control
#define IR_CODE_UP 16621663      //UP ARROW
#define IR_CODE_DOWN 16625743    //DOWN ARROW
#define IR_CODE_LEFT 16584943   //LEFT ARROW
#define IR_CODE_RIGHT 16601263   //RIGHT ARROW
#define IR_CODE_STOP 16605343  //STOP MODE BUTTON
#define IR_CODE_SETUP 16589023   //SETUP BUTTON
#define IR_CODE_0 16593103 //0 10+ BUTTON
#define IR_CODE_1 16582903 // 1 BUTTON
#define IR_CODE_2 16615543 // 2 BUTTON
#define IR_CODE_3 16599223 // 3 BUTTON
#define IR_CODE_4 16591063 // 4 BUTTON
#define IR_CODE_5 16623703 // 5 BUTTON
#define IR_CODE_6 16607383 // 6 BUTTON
#define IR_CODE_7 16586983 // 7 BUTTON
#define IR_CODE_8 16619623 // 8 BUTTON
#define IR_CODE_9 16603303 // 9 BUTTON
#define IR_CODE_ENTER 16617583 // ENTER BUTTON
#define IR_CODE_BACK 16609423 // back button
// Define calibration values
#define TURN_CALIBRATION 4.7 // time in ms to turn about 1 degree
#define LENGTH_CALIBRATION 800 // time it takes to travel 1 ft in ms
#define CIRCLE_TIME 6600 // time it takes to travel in a full circle in ms


int RECV_PIN = TKD1; // the pin the IR receiver is connected to
IRrecv irrecv(RECV_PIN); // an instance of the IR receiver object
decode_results results; // container for received IR codes
int motor_spd = 200; // normal motor speed
int turn_spd = 150; // speed of motors for turning
int cw = 1; //go clockwise
int ccw = -1; //go counter-clockwise
// global values store current location of robot in space
int currentAngle = 0;
int currentX = 0;
int currentY = 0;

// function to tell the robot to spin to the desired angle given an angle in the global coordinate system
void goToAngle(int destination) {
  int angle = destination - currentAngle; //determines nessecary angle
  Robot.debugPrint(angle, 5, 100);
  delay(1000);
  float delayTime = angle * TURN_CALIBRATION; //find time to turn
  // decides what direction to turn
  if (angle > 0) {
    Robot.motorsWrite(turn_spd, -turn_spd);
    delay(delayTime);
  } else {
    Robot.motorsWrite(-turn_spd, turn_spd);
    delay(-delayTime);
  }
  Robot.motorsStop();
  currentAngle = destination;
  if (currentAngle < 0) {
    currentAngle = currentAngle + 360;
  }
}

// function to drive the robot to a desired global coordinate given a global coordinate
void goToPoint(int destX, int destY) {
  int yLength = destY - currentY;
  int xLength = destX - currentX;
  //determine how far to drive
  float hyp = sqrt(yLength * yLength + xLength * xLength);
  // find angle to travel to
  int angle = (int)(atan2(yLength, xLength) * 180 / M_PI);
  Robot.text(angle, 5, 115);
  //go to angle to achieve straight line travel
  goToAngle(angle);
  delay(100);
  //find how long to drive for
  float delayTime = hyp * LENGTH_CALIBRATION;
  Robot.motorsWrite(motor_spd, motor_spd);
  delay(delayTime);
  Robot.motorsStop();
  currentX = destX;
  currentY = destY;
}

// function to drive robot in a circle given a direction to drive in
void driveCircle(int direct) {
  if (direct == cw) { //clockwise circel
    Robot.motorsWrite(motor_spd, 0.6 * motor_spd);
  } else { //counter clockwise circle
    Robot.motorsWrite(0.6 * motor_spd, motor_spd);
  }
  delay(CIRCLE_TIME);
  Robot.motorsStop();
}

// function to drive robot in a square using the go to point command
void driveSquare(int direc) {
  if (direc == cw) {
    goToPoint(0, 3);
    delay(100);
    goToPoint(3, 3);
    delay(100);
    goToPoint(3, 0);
    delay(100);
    goToPoint(0, 0);
    delay(100);
  } else {
    goToPoint(3, 0);
    delay(100);
    goToPoint(3, 3);
    delay(100);
    goToPoint(0, 3);
    delay(100);
    goToPoint(0, 0);
    delay(100);
  }
}

// Function to drive the robot in a figure eight using the driveCircle
void driveFigureEight() {
  driveCircle(cw);
  driveCircle(ccw);
}

// function which allows user input to be gathered in the form of an integer
int getNumber() {
  int leave = -1;
  int output = 0;
  while (leave) {
    irrecv.resume();
    while (!irrecv.decode(&results)) {
      Robot.text("input a number", 5, 55);
    }
    switch (results.value) {
      case (IR_CODE_0):
        output = output * 10;
        break;
      case (IR_CODE_1):
        output = output * 10 + 1;
        break;
      case (IR_CODE_2):
        output = output * 10 + 2;
        break;
      case (IR_CODE_3):
        output = output * 10 + 3;
        break;
      case (IR_CODE_4):
        output = output * 10 + 4;
        break;
      case (IR_CODE_5):
        output = output * 10 + 5;
        break;
      case (IR_CODE_6):
        output = output * 10 + 6;
        break;
      case (IR_CODE_7):
        output = output * 10 + 7;
        break;
      case (IR_CODE_8):
        output = output * 10 + 8;
        break;
      case (IR_CODE_9):
        output = output * 10 + 9;
        break;
      case (IR_CODE_ENTER):
        leave = 1;
        Robot.text("leaving", 5, 75);
        break;
      case (IR_CODE_BACK):
        output = output * -1;
        break;
    }
    Robot.debugPrint(output, 5, 85);
    if (leave == 1) {
      break;
    }
  }
  return output;
}

// Function to allow user to choose the mode that the robot will be working in
void chooseMode() {
  switch (results.value) {
    //go to angle
    case (IR_CODE_UP):
      {
        Robot.text("Angle mode", 5, 45);
        int angle = getNumber();
        goToAngle(angle);
        Robot.motorsWrite(motor_spd, motor_spd);
        delay(1000);
        Robot.motorsStop();
        Robot.clearScreen();
      }
      break;

    case (IR_CODE_RIGHT):
      {
        Robot.text("Point mode", 5, 45);
        int x = getNumber();
        int y = getNumber();
        goToPoint(x, y);
        Robot.clearScreen();
      }
      break;

    case (IR_CODE_DOWN):
      {
        Robot.text("Square mode", 5, 45);
        int direc = getNumber();
        driveSquare(direc);
        Robot.clearScreen();
      }
      break;

    case (IR_CODE_LEFT):
      {
        Robot.text("Circle mode", 5, 45);
        driveCircle(cw);
        Robot.clearScreen();
      }
      break;

    case (IR_CODE_SETUP):
      {
        Robot.text("Figure eight mode", 5, 45);
        driveFigureEight();
        Robot.clearScreen();
      }
      break;
  }
}



// Begins all of the important functions for the robot
void setup() {
  // initialize the Robot, SD card, display, and speaker
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  irrecv.enableIRIn(); // Start the receiver
}

// Runs the overall robot code
void loop() {
  Robot.text("Up arrow angle", 5, 1);
  Robot.text("Left arrow circle", 5, 9);
  Robot.text("Down arrow square", 5, 17);
  Robot.text("Right arrow point", 5, 26);
  Robot.text("Setup key eight", 5, 35);
  Robot.text(currentX, 5, 130);
  Robot.text(currentY, 5, 140);
  Robot.text(currentAngle, 5, 150);
  if (irrecv.decode(&results)) {
    chooseMode();
    irrecv.resume(); // resume receiver
  }
}


