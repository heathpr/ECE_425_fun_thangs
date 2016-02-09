#include "ArduinoRobot.h"

//define pins used
#define SONAR_FRONT D1
#define SONAR_LEFT D2
#define SONAR_RIGHT D0
#define SONAR_REAR B_TK1

//define constants for sensor data
#define MIN_SONAR 483
#define NUM_SAMPLES 5

#define LEFT 1
#define RIGHT -1

//directions Data can go
#define UP_DIR 1
#define RIGHT_DIR 2
#define DOWN_DIR 3
#define LEFT_DIR 4

#define WALL_DIST 18

int worldMap [7][7] = {
  {99, 99, 99, 99, 99, 99, 99},
  {99, 99, 99, 99, 99, 99, 99},
  {99, 99, 99, 99, 99, 99, 99},
  {99, 99, 99, 99, 99, 99, 99},
  {99, 99, 99, 99, 99, 99, 99},
  {99, 99, 99, 99, 99, 99, 99},
  {99, 99, 99, 99, 99, 99, 99}
};
int currentMap [4][4] = {
  {9, 7, 15, 11},
  {10, 15, 15, 10},
  {8, 5, 5, 2},
  {14, 15, 15, 14}
};
int pastPath[16] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
bool leftWall = false;
bool rightWall = false;
bool upWall = false;
bool downWall = false;

int orientation = UP_DIR;

int findCellValue(void);
void detectWalls(void);
double checkSonarPin(int);

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
}

void loop() {
  // put your main code here, to run repeatedly:

}

int findCellValue() {
  int value = 15;
  if (leftWall && rightWall && downWall && upWall) {
    value = 15;
  } else if (leftWall && rightWall && downWall && !upWall) {
    value = 14;
  } else if (leftWall && !rightWall && downWall && upWall) {
    value = 13;
  } else if (leftWall && !rightWall && downWall && !upWall) {
    value = 12;
  } else if (leftWall && rightWall && !downWall && upWall) {
    value = 11;
  } else if (leftWall && rightWall && !downWall && !upWall) {
    value = 10;
  } else if (leftWall && !rightWall && !downWall && upWall) {
    value = 9;
  } else if (leftWall && !rightWall && !downWall && !upWall) {
    value = 8;
  } else if (!leftWall && rightWall && downWall && upWall) {
    value = 7;
  } else if (!leftWall && rightWall && downWall && !upWall) {
    value = 6;
  } else if (!leftWall && !rightWall && downWall && upWall) {
    value = 5;
  } else if (!leftWall && !rightWall && downWall && !upWall) {
    value = 4;
  } else if (!leftWall && rightWall && !downWall && upWall) {
    value = 3;
  } else if (!leftWall && rightWall && !downWall && !upWall) {
    value = 2;
  } else if (!leftWall && !rightWall && !downWall && upWall) {
    value = 1;
  } else {
    value = 0;
  }
}

void detectWalls(void) {
  int leftWallDist = checkSonarPin(SONAR_LEFT);
  int rightWallDist = checkSonarPin(SONAR_RIGHT);
  int frontWallDist = checkSonarPin(SONAR_FRONT);
  int rearWallDist = checkSonarPin(SONAR_REAR);
  int tempDir;

  switch (orientation) {
    case DOWN_DIR:
      tempDir = leftWallDist;
      leftWallDist = rightWallDist;
      rightWallDist = tempDir;
      tempDir = frontWallDist;
      frontWallDist = rearWallDist;
      rearWallDist = tempDir;
      break;
    case LEFT_DIR:
      tempDir = leftWallDist;
      leftWallDist = frontWallDist;
      frontWallDist = rightWallDist;
      rightWallDist = rearWallDist;
      rearWallDist = tempDir;
      break;
    case RIGHT_DIR:
      tempDir = rightWallDist;
      rightWallDist = frontWallDist;
      frontWallDist = leftWallDist;
      leftWallDist = rearWallDist;
      rearWallDist = tempDir;
      break;
  }

  if (leftWallDist < WALL_DIST) {
    leftWall = true;
  } else {
    leftWall = false;
  }
  if (rightWallDist < WALL_DIST) {
    rightWall = true;
  } else {
    rightWall = false;
  }
  if (frontWallDist < WALL_DIST) {
    upWall = true;
  } else {
    upWall = false;
  }
  if (rearWallDist < WALL_DIST) {
    downWall = true;
  } else {
    downWall = false;
  }
}

/*
   polls a sonar pin

   pin - the pin that is being polled
*/
double checkSonarPin(int pin) {
  double output = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int value;
    pinMode(pin, OUTPUT);//set the PING pin as an output
    Robot.digitalWrite(pin, LOW);//set the PING pin low first
    delayMicroseconds(2);//wait 2 us
    Robot.digitalWrite(pin, HIGH);//trigger sonar by a 2 us HIGH PULSE
    delayMicroseconds(5);//wait 5 us
    Robot.digitalWrite(pin, LOW);//set pin low first again
    pinMode(pin, INPUT);//set pin as input with duration as reception time
    value = pulseIn(pin, HIGH);//measures how long the pin is high

    value = .0111 * value - 0.8107;
    output += value;
    delay(5);
  }
  output = output / NUM_SAMPLES;
  if (output <= 0) {
    output = 9998; // low output is set to a very large distance
  }
  return output;
}


