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

#define DIFFERENCE_THRESHOLD 5 // threshold from front sonar and side sonar to go into straight wall behavior

// PD control constants
#define KP 15
#define KD 2

#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 125
#define TURN_SPEED 150
#define MOTOR_SPEED 200
#define MOVE_TIME 200

#define TURN_TIME 650
#define CELL_TIME 1900

//dead band where the robot does not try to correct error
#define LOW_BAND 1
#define HIGH_BAND 1

#define LEFT 1
#define RIGHT -1

//movemnts the robot can do
#define UP_DIR 1
#define RIGHT_DIR 2
#define DOWN_DIR 3
#define LEFT_DIR 4

#define BUTTON_TIME 100

int previousValue = 0;

//path planning variables
int startX = 0;
int startY = 0;
int goalX = 0;
int goalY = 0;
int path[16] = {0};
int iter = 0;
int inputMap[4][4] = {
  {0, 99, 99, 0},
  {0, 0, 0, 0},
  {0, 99, 99, 0},
  {0, 99, 0, 0}
};

int orientation = DOWN_DIR; //starting orientation


int identifyState(void);
void setPath(void);
void createPath( int x, int y, int iter);

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
  setPath();
  //create the map
  createMap(goalX - 1, goalY, 1);
  createMap(goalX + 1, goalY, 1);
  createMap(goalX, goalY + 1, 1);
  createMap(goalX, goalY - 1, 1);
  Robot.clearScreen();
  
  //create the path
  createPath(startX, startY, 0);
  Robot.clearScreen();

  //print out the map
  Robot.text("map", 5, 1);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Robot.debugPrint(inputMap[j][i], 15 * (1 + i), (j + 1) * 9);
    }
  }

  //print out the path
  int i = 0;
  int x = startX;
  int y = startY;
  while (i < 16) {
    Robot.debugPrint(1, 5 * (x + 1), 50 + 9 * (y + 1));
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
  Robot.clearScreen();
  //switch based off of the movements in the path
  switch (path[iter]) {
    case UP_DIR://go up
      switch (orientation) {//turn to the up direction
        case UP_DIR:
          break;
        case DOWN_DIR:
          turn90(LEFT);
          turn90(LEFT);
          break;
        case LEFT_DIR:
          turn90(RIGHT);
          break;
        case RIGHT_DIR:
          turn90(LEFT);
          break;
      }
      moveCell();
      orientation = path[iter];
      break;

    case DOWN_DIR://go down
      switch (orientation) {//turn to the down direction
        case UP_DIR:
          turn90(LEFT);
          turn90(LEFT);
          break;
        case DOWN_DIR:

          break;
        case LEFT_DIR:
          turn90(LEFT);
          break;
        case RIGHT_DIR:
          turn90(RIGHT);
          break;
      }
      moveCell();
      orientation = path[iter];
      break;

    case LEFT_DIR://go left
      switch (orientation) {//turn to the left direction
        case UP_DIR:
          turn90(LEFT);
          break;
        case DOWN_DIR:
          turn90(RIGHT);
          break;
        case LEFT_DIR:
          break;
        case RIGHT_DIR:
          turn90(LEFT);
          turn90(LEFT);
          break;
      }
      moveCell();
      orientation = path[iter];
      break;

    case RIGHT_DIR://go right
      switch (orientation) {//turn to the right direction
        case UP_DIR:
          turn90(RIGHT);
          break;
        case DOWN_DIR:
          turn90(LEFT);
          break;
        case LEFT_DIR:
          turn90(RIGHT);
          turn90(RIGHT);
          break;
        case RIGHT_DIR:
          break;
      }
      moveCell();
      orientation = path[iter];
      break;
  }
  iter++;
  delay(1000);
}

/*
   asks for the user to input x and y coordinates for the goal and start
*/
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
}


/*
 * recursive algorithm that creates a wavefront propogation map
 * 
 * x - x coordinate
 * y - y coordinate
 * currentNum - current value to put into the map
 */
void createMap(int x, int y, int currentNum) {
  //end case
  if (x > 3 || x < 0 || y > 3 || y < 0 || inputMap[y][x] == 99 || (x == goalX && y == goalY)) {
    return;
  }
  //if the spot does not have a wavefront value give it a value
  if (inputMap[y][x] == 0) {
    inputMap[y][x] = currentNum++;
  } else {
    return;
  }

  //iterate through all the other positions (bad cases are handled at the beginging of the createMap()
  createMap(x - 1, y, currentNum);
  createMap(x + 1, y, currentNum);
  createMap(x, y - 1, currentNum);
  createMap(x, y + 1, currentNum);
  
  return;
}

/*
 * recursively creates a path using the wavefront propagation map
 * x- x coordinate
 * y - y coordinate
 * iter - iterator that goes through the path array
 */
void createPath( int x, int y, int iter) {
  //end cases
  if (x > 3 || x < 0 || y > 3 || y < 0 || inputMap[y][x] == 99 || inputMap[y][x] == 0) {
    return;
  }

  //populate each direction with their wavefront value
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

  //choose the direction with the lowest value and iterate
  if (up  <= down && up  <= left && up  <= right) {
    path[iter] = UP_DIR;
    createPath(x, y - 1, iter + 1);
  } else if (down  <= left && down  <= right) {
    path[iter] = DOWN_DIR;
    createPath( x, y + 1, iter + 1);
  } else if (left <= right) {
    path[iter] = LEFT_DIR;
    createPath( x - 1, y, iter + 1);
  } else {
    path[iter] = RIGHT_DIR;
    createPath(x + 1, y, iter + 1);
  }

  return;
}

/*
 * turns 90 degrees in the inputted direction, unless it decides not to.
 */
void turn90(int direc) {
  if (direc == LEFT) {
    Robot.text("turn left", 5, 85);
  } else {
    Robot.text("turn right", 5, 85);
  }
  Robot.motorsWrite(direc * TURN_SPEED, -direc * TURN_SPEED);
  delay(TURN_TIME);
  Robot.motorsStop();
  delay(500);
}

/*
 * moves one cell worth of distance foward
 */
void moveCell() {
  Robot.text("driving forward", 5, 80);
  Robot.motorsWrite(MOTOR_SPEED, MOTOR_SPEED);
  delay(CELL_TIME);
  Robot.motorsStop();
  delay(500);
}

