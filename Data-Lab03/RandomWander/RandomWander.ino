#include <ArduinoRobot.h>

#define WAIT_TIME 500
#define MAX_MOTOR_SPEED 255
#define MAX_FORWARD_TIME 5000
#define MAX_ANGLE 180
#define TURN_CALIBRATION 4.7
#define TURN_SPEED 150

void randomWander() {
  int motorSpeed = random(MAX_MOTOR_SPEED);
  int forwardTime = random(MAX_FORWARD_TIME);
  int angle = random(-MAX_ANGLE, MAX_ANGLE);
  Robot.text(motorSpeed, 5, 5);
  Robot.text(forwardTime, 5, 15);
  Robot.text(angle, 5, 50);
  //change angle
  goToAngle(angle);
  // drive forward
  Robot.motorsWrite(motorSpeed, motorSpeed);
  delay(forwardTime);
  Robot.motorsStop();
}

// function to tell the robot to spin to the desired angle given an angle in the global coordinate system
void goToAngle(int angle) {
  Robot.debugPrint(angle, 5, 100);
  delay(1000);
  float delayTime = angle * TURN_CALIBRATION; //find time to turn
  // decides what direction to turn
  if (angle > 0) {
    Robot.motorsWrite(TURN_SPEED, -TURN_SPEED);
    delay(delayTime);
  } else {
    Robot.motorsWrite(-TURN_SPEED, TURN_SPEED);
    delay(-delayTime);
  }
  Robot.motorsStop();
}

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  randomWander();
  delay(WAIT_TIME);
   Robot.clearScreen();
}
