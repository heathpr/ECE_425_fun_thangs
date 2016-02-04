#include "ArduinoRobot.h"

#define TOL 4
#define RIGHT -1
#define LEFT 1
#define TURN_SPEED 150
#define TURN_TIME 50

void turn(int);

void setup() {
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  while(!Robot.keyboardRead() == BUTTON_MIDDLE){
    // do nothing
  }
  turn(90);
  delay(2000); 

}
void turn(int angle) {
  int direc;
  int currentAngle;
  int desiredAngle;
  int previousAngle;
  currentAngle = Robot.compassRead();
  Robot.debugPrint(currentAngle, 5, 5);
  desiredAngle = currentAngle + angle;
  if(desiredAngle>359){
    desiredAngle=desiredAngle-360;
  }
  Robot.debugPrint(desiredAngle, 5, 15);
  delay(2000);
  while (abs(desiredAngle-currentAngle)>TOL) {
    Robot.clearScreen();
    currentAngle = Robot.compassRead();
    if (currentAngle < desiredAngle) {
      direc = RIGHT;
    } else {
      direc = LEFT;
    }
    Robot.debugPrint(currentAngle,5,25);
    Robot.motorsWrite(direc*TURN_SPEED,-direc*TURN_SPEED);
    delay(TURN_TIME);
  }
  Robot.motorsStop();
}

