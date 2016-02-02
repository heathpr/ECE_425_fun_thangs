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

#define TURN_TIME 650
#define CELL_TIME 1450

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
int magicIterator = -1;
int iter = 0;
int orientation = DOWN_DIR;
int inputMap[4][4] = {
  {11, 15, 15, 11},
  {10, 13, 1, 2},
  {8, 1, 6, 10},
  {15, 14, 15, 14}
};
int visited[4][4] = {0};
int waveFrontMap[4][4] = {
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99}
};


int identifyState(void);
void setPath(void);
void createPath( int x, int y, int iter);
void findMoves(int x, int y, int moves[]);
void createMap(int x, int y, int i);



void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
  getStartAndGoal();
  Robot.clearScreen();
  //findPath(startX, startY);
  createMap(goalX, goalY, 0);
  Robot.text("map", 5, 1);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Robot.debugPrint(waveFrontMap[j][i], 15 * (1 + i), (j + 1) * 9);
    }
  }
  while (Robot.keyboardRead() != BUTTON_MIDDLE);
  createPath(startX, startY, 0);

  int i = 0, x = startX, y = startY;
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

  for (int i = 0; i < 16; i++) {
    Robot.debugPrint(path[i], 5 * (i + 1), 150);
  }
  while (Robot.keyboardRead() != BUTTON_MIDDLE);


}

void loop() {
  Robot.clearScreen();
  switch (path[iter]) {
    case UP_DIR:
      switch (orientation) {
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
    case DOWN_DIR:
      switch (orientation) {
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
    case LEFT_DIR:
      switch (orientation) {
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
    case RIGHT_DIR:

      switch (orientation) {
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

void getStartAndGoal() {
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

void createPath( int x, int y, int iter) {
  if (x > 3 || x < 0 || y > 3 || y < 0 || waveFrontMap[y][x] == 99 || waveFrontMap[y][x] == 0) {
    return;
  }

  int up = 99, down = 99, left = 99, right = 99, current = waveFrontMap[y][x];

  int moves[4] = {0};
  findMoves(x, y, moves);

  for (int i = 0; i < 4; i++) {
    bool leave = false;
    switch (moves[i]) {
      case RIGHT_DIR:
        right = waveFrontMap[y][x + 1];
        break;
      case LEFT_DIR:
        left = waveFrontMap[y][x - 1];
        break;
      case UP_DIR:
        up = waveFrontMap[y - 1][x];
        break;
      case DOWN_DIR:
        down = waveFrontMap[y + 1][x];
        break;
      default:
        leave = true;
        break;
    }
    if (leave) {
      break;
    }
  }

  if (current - 1 == up) {
    path[iter] = UP_DIR;
    createPath(x, y - 1, iter + 1);
  } else if (current - 1 == down) {
    path[iter] = DOWN_DIR;
    createPath( x, y + 1, iter + 1);
  } else if (current - 1 == left) {
    path[iter] = LEFT_DIR;
    createPath( x - 1, y, iter + 1);
  } else {
    path[iter] = RIGHT_DIR;
    createPath(x + 1, y, iter + 1);
  }
  return;
}

void createMap(int x, int y, int iter) {
  if (x > 3 || x < 0 || y > 3 || y < 0 || inputMap[y][x] == 15 || visited[y][x] == 1) {
    return;
  }
  if (waveFrontMap[y][x] > iter) {
    waveFrontMap[y][x] = iter;
  } else {
    return;
  }
  int moves[4] = {0};
  findMoves(x, y, moves);
  visited[y][x] = 1;

  for (int i = 0; i < 4; i++) {
    bool leave = false;
    switch (moves[i]) {
      case RIGHT_DIR:
        createMap(x + 1, y, iter + 1);
        break;
      case LEFT_DIR:
        createMap(x - 1, y, iter + 1);
        break;
      case UP_DIR:
        createMap(x, y - 1, iter + 1);
        break;
      case DOWN_DIR:
        createMap(x, y + 1, iter + 1);
        break;
      default:
        leave = true;
        break;
    }
    if (leave) {
      break;
    }
  }
  return;
}

void findMoves(int x, int y, int moves[]) {
  if (inputMap[y][x] == 15) {
    return;
  }
  int i = 0;
  if (inputMap[y][x] != 2 && inputMap[y][x] != 3 && inputMap[y][x] != 6 &&
      inputMap[y][x] != 10 && inputMap[y][x] != 11 && inputMap[y][x] != 14) {
    moves[i++] = RIGHT_DIR;
  }

  if (inputMap[y][x] != 8 && inputMap[y][x] != 9 && inputMap[y][x] != 10 &&
      inputMap[y][x] != 11 && inputMap[y][x] != 12 && inputMap[y][x] != 13 && inputMap[y][x] != 14) {
    moves[i++] = LEFT_DIR;
  }
  if (inputMap[y][x] != 1 && inputMap[y][x] != 3 && inputMap[y][x] != 5 &&
      inputMap[y][x] != 7 && inputMap[y][x] != 9 && inputMap[y][x] != 11 && inputMap[y][x] != 13) {
    moves[i++] = UP_DIR;
  }

  if (inputMap[y][x] != 4 && inputMap[y][x] != 5 && inputMap[y][x] != 6 &&
      inputMap[y][x] != 7 && inputMap[y][x] != 12 && inputMap[y][x] != 13 && inputMap[y][x] != 14) {
    moves[i++] = DOWN_DIR;
  }
  return;
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

void moveCell() {
  Robot.text("driving forward", 5, 80);
  Robot.motorsWrite(MOTOR_SPEED, MOTOR_SPEED);
  delay(CELL_TIME);
  Robot.motorsStop();
  delay(500);
}







