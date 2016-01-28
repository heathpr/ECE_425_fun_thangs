#include "ArduinoRobot.h"

//define pins used
#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D2
#define SONAR_RIGHT D0

//define constants for sensor data
#define MIN_SONAR 483
#define MAX_IR 595
#define NUM_SAMPLES 5

#define STRAIGHT_HALLWAY 0
#define RIGHT_CORNER 1
#define LEFT_CORNER 2
#define DEAD_END 3
#define LEFT_HALL 4
#define RIGHT_HALL 5
#define BOTH_HALL 6
#define T_JUNCTION 7

#define DIFFERENCE_THRESHOLD 5 // threshold from front sonar and side sonar to go into straight wall behavior

// PD control constants
#define KP 15
#define KD 2

#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 125
#define TURN_SPEED 150
#define MOTOR_SPEED 200
#define MOVE_TIME 200

#define TURN_TIME 423

#define FORWARD_WALL 0
#define FORWARD_BLIND 1
#define TURN_LEFT 2
#define TURN_RIGHT 3
#define STOP 4

//dead band where the robot does not try to correct error
#define LOW_BAND 1
#define HIGH_BAND 1

#define WALL_DIST 12

#define LEFT 1
#define RIGHT -1

#define UP_DIR 1
#define RIGHT_DIR 2
#define DOWN_DIR 3
#define LEFT_DIR 4

#define BUTTON_TIME 100

bool leftWall = false;
bool rightWall = false;
bool frontWall = false;
int previousValue = 0;
int startX = 0;
int startY = 0;
int goalX = 0;
int goalY = 0;
int path[16] = {0};


int identifyState(void);
void setPath(void);
void createPath( int x, int y, int iter);
int inputMap[4][4] = {
  {0, 99, 99, 0},
  {0, 0, 0, 0},
  {0, 99, 99, 0},
  {0, 99, 0, 0}
};


void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
  setPath();
  Robot.clearScreen();
  Robot.text("map", 5, 1);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Robot.debugPrint(inputMap[j][i], 15 * (1 + i), (j + 1) * 9);
    }
  }
  int i = 0;
  int x = startX;
  int y = startY;
//  for(int j = 0;j<16;j++){
//    Robot.debugPrint(path[j],5*j,60);
//  }
//  while(Robot.keyboardRead()!=BUTTON_MIDDLE);
  while (i < 16) {
    Robot.debugPrint(1, 5 * (x + 1), 50 + 9 * (y + 1));
//    Robot.debugPrint(i, 5, 150);
//
//    while (Robot.keyboardRead() != BUTTON_MIDDLE);
//    Robot.clearScreen();
    if (path[i] == 0) {
      break;
    }
    switch (path[i]) {
      case UP_DIR:
        y--;
        break;
      case DOWN_DIR:
        y++;
        break;
      case LEFT_DIR:
        x--;
        break;
      case RIGHT_DIR:
        x++;
        break;
    }
    i++;
  }
  while (Robot.keyboardRead() != BUTTON_MIDDLE);

}

void loop() {
  delay(500);
}

void turn(int direc) {
  goStraight();
  goStraight();
  Robot.motorsWrite(direc * TURN_SPEED, -direc * TURN_SPEED);
  delay(TURN_TIME);
  Robot.motorsStop();
  goStraight();
  goStraight();
  goStraight();
}

void goStraight() {
  Robot.motorsWrite(MOTOR_SPEED, MOTOR_SPEED);
  delay(MOVE_TIME);
  Robot.motorsStop();
}

void setPath(void) {
  Robot.text("Enter start x", 5, 1);
  while (Robot.keyboardRead() != BUTTON_MIDDLE) {
    if (Robot.keyboardRead() == BUTTON_UP) {
      startX++;
    } else if (Robot.keyboardRead() == BUTTON_DOWN) {
      startX--;
    }
    delay(BUTTON_TIME);
    Robot.debugPrint(startX, 5, 9);
  }
  delay(BUTTON_TIME);
  Robot.clearScreen();
  Robot.text("Enter start y", 5, 1);
  while (Robot.keyboardRead() != BUTTON_MIDDLE) {
    if (Robot.keyboardRead() == BUTTON_UP) {
      startY++;
    } else if (Robot.keyboardRead() == BUTTON_DOWN) {
      startY--;
    }
    delay(BUTTON_TIME);
    Robot.debugPrint(startY, 5, 9);
  }
  delay(BUTTON_TIME);
  Robot.clearScreen();
  Robot.text("Enter goal x", 5, 1);
  while (Robot.keyboardRead() != BUTTON_MIDDLE) {
    if (Robot.keyboardRead() == BUTTON_UP) {
      goalX++;
    } else if (Robot.keyboardRead() == BUTTON_DOWN) {
      goalX--;
    }
    delay(BUTTON_TIME);
    Robot.debugPrint(goalX, 5, 9);
  }
  delay(BUTTON_TIME);
  Robot.clearScreen();
  Robot.text("Enter goal y", 5, 1);
  while (Robot.keyboardRead() != BUTTON_MIDDLE) {
    if (Robot.keyboardRead() == BUTTON_UP) {
      goalY++;
    } else if (Robot.keyboardRead() == BUTTON_DOWN) {
      goalY--;
    }
    delay(BUTTON_TIME);
    Robot.debugPrint(goalY, 5, 9);
  }

  createMap(goalX - 1, goalY, 0);
  createMap(goalX + 1, goalY, 0);
  createMap(goalX, goalY + 1, 0);
  createMap(goalX, goalY - 1, 0);
  Robot.clearScreen();
  createPath(startX, startY, 0);
}

void createMap(int x, int y, int currentNum) {
  if (x > 3 || x < 0 || y > 3 || y < 0 || inputMap[y][x] == 99 || (x == goalX && y == goalY)) {
    return;
  }
  if (inputMap[y][x] == 0) {
    inputMap[y][x] = ++currentNum;
  } else {
    return;
  }

  //  Robot.debugPrint(x, 5, 1);
  //  Robot.debugPrint(y, 5, 9);
  //  Robot.debugPrint(currentNum, 5, 17);
  //  while (Robot.keyboardRead() != BUTTON_MIDDLE);
  //  Robot.clearScreen();

  createMap(x - 1, y, currentNum);
  createMap(x + 1, y, currentNum);
  createMap(x, y - 1, currentNum);
  createMap(x, y + 1, currentNum);
  return;
}

void createPath( int x, int y, int iter) {
  if (x > 3 || x < 0 || y > 3 || y < 0 || inputMap[y][x] == 99 || inputMap[y][x] == 0) {
    return;
  }

  int up = 99, down = 99, left = 99, right = 99;
  if (x + 1 < 4) {
    right = inputMap[y][x + 1];
  }
  if (x - 1 > -1) {
    left = inputMap[y][x - 1];
  }
  if (y + 1 < 4) {
    down = inputMap[y + 1][x];
  }
  if (y - 1 > -1) {
    up = inputMap[y - 1][x];
  }

  //  Robot.debugPrint(x, 5, 1);
  //  Robot.debugPrint(y, 5, 9);
  //  Robot.debugPrint(iter, 5, 17);
  //  Robot.debugPrint(up, 5, 50);
  //  Robot.debugPrint(down, 5, 59);
  //  Robot.debugPrint(left, 5, 68);
  //  Robot.debugPrint(right, 5, 77);


  if (up  <= down && up  <= left && up  <= right) {
    path[iter] = UP_DIR;
    //    Robot.text("going up", 5, 40);
    //    while (Robot.keyboardRead() != BUTTON_MIDDLE);
    //    Robot.clearScreen();
    createPath(x, y - 1, iter + 1);
  } else if (down  <= left && down  <= right) {
    path[iter] = DOWN_DIR;
    //    Robot.text("going down", 5, 40);
    //    while (Robot.keyboardRead() != BUTTON_MIDDLE);
    //    Robot.clearScreen();
    createPath( x, y + 1, iter + 1);
  } else if (left <= right) {
    path[iter] = LEFT_DIR;
    //    Robot.text("going left", 5, 40);
    //    while (Robot.keyboardRead() != BUTTON_MIDDLE);
    //    Robot.clearScreen();
    createPath( x - 1, y, iter + 1);
  } else {
    path[iter] = RIGHT_DIR;
    //    Robot.text("going right", 5, 40);
    //    while (Robot.keyboardRead() != BUTTON_MIDDLE);
    //    Robot.clearScreen();
    createPath(x + 1, y, iter + 1);
  }

  return;
}


