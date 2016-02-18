#include "ArduinoRobot.h"

//define pins used
#define SONAR_FRONT D1
#define SONAR_LEFT D2
#define SONAR_RIGHT D0
#define SONAR_BACK D3

//define constants for sensor data
#define MIN_SONAR 483
#define NUM_SAMPLES 5

#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 125
#define TURN_SPEED 150
#define MOTOR_SPEED 200

#define MOVE_TIME 200
#define TURN_TIME 750
#define CELL_TIME 1550

#define LEFT 1
#define RIGHT -1

//directions Data can go
#define UP_DIR 1
#define RIGHT_DIR 2
#define DOWN_DIR 3
#define LEFT_DIR 4

#define WALL_DIST 17
#define BUTTON_TIME 100

int path [16] = {0};
int iter = 0;
int worldMap[7][7] = {
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
int wavefrontMap[4][4] = {
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99}
};
int x = 3, y = 3;
int startVal;
int goalX = 0, goalY = 0;

bool localized = false;

int orientation = UP_DIR;

bool localize(int x, int y);
void addMap(int x, int y);
void detectWalls(int wallDist[], bool walls[]);
void findMoves(int val, int moves[]);
double checkSonarPin(int);

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
  getEndPoint();
  printMap(worldMap);
  bool walls[4] = {0};
  addMap(x, y);
  localized = localize(x, y);
  if (localized) {
    Robot.clearScreen();
    Robot.text("I have localized", 1, 1);
    Robot.debugPrint(x, 1, 10);
    Robot.debugPrint(y, 10, 10);
    while (Robot.keyboardRead() != BUTTON_MIDDLE);
  }
}

void loop() {
  Robot.clearScreen();
  //switch based off of the movements in the path
  int moves[4] = { 0};
  findMoves(worldMap[y][x], moves);
  int dir;
  if (!localized) {
    dir = moves[0];
  } else {
    dir = path[iter++];
  }
  if(dir==0){
    Robot.clearScreen();
    Robto.text("arrived at goal");
    while(1);
  }
  switch (dir) {
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
      y--;
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
      y++;
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
      x--;
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
      x++;
      break;
  }
  moveCell();
  orientation = dir;
  if (!localized) {
    printMap(worldMap);
    path[iter++] = dir;
    addMap(x, y);
    localized = localize(x, y);
    if (localized) {
      Robot.clearScreen();
      Robot.text("I have localized", 1, 1);
      Robot.debugPrint(x, 1, 10);
      Robot.debugPrint(y, 10, 10);
      while (Robot.keyboardRead() != BUTTON_MIDDLE);
      for (int i = 0; i < 16; i++) {
        path[i] = 0;
      }
      iter = 0;
      createMap(goalX, goalY, 0);
      createPath(x, y, 0);
      Robot.clearScreen();

      //print out the map
      Robot.text("map", 5, 1);
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          Robot.debugPrint(wavefrontMap[j][i], 15 * (1 + i), (j + 1) * 9);
        }
      }
      while (Robot.keyboardRead() != BUTTON_MIDDLE);
    }
  }
}
/*
   recursively creates the wavefront map
   x - x coordinate
   y - y coordinate
   iter - wavefront value
*/
void createMap(int x, int y, int iter) {
  //break case
  if (x > 3 || x < 0 || y > 3 || y < 0 || currentMap[y][x] == 15 || wavefrontMap[y][x] < iter) {
    return;
  }
  //if the current wavefront value is greater than iter set the wavefront value to iter
  //(the map initializes at 99)
  if (wavefrontMap[y][x] > iter) {
    wavefrontMap[y][x] = iter;
  } else {
    return;
  }

  //find valid moves
  int moves[4] = {0};
  findMoves(currentMap[y][x], moves);

  //recursively call it for every valid move
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

/*
   recursively creates the path the robot is going to follow
   x - x coordinate
   y - y coordinate
   iter - where in the path array the direction goes
*/
void createPath( int x, int y, int iter) {
  //break case
  if (x > 3 || x < 0 || y > 3 || y < 0 || wavefrontMap[y][x] == 99 || wavefrontMap[y][x] == 0) {
    return;
  }

  int up = 99, down = 99, left = 99, right = 99, current = wavefrontMap[y][x];

  int moves[4] = {0};
  findMoves(currentMap[y][x], moves);//only check directions where there is not a wall

  for (int i = 0; i < 4; i++) {
    bool leave = false;
    //find stored wavefront values for valid directions
    switch (moves[i]) {
      case RIGHT_DIR:
        right = wavefrontMap[y][x + 1];
        break;
      case LEFT_DIR:
        left = wavefrontMap[y][x - 1];
        break;
      case UP_DIR:
        up = wavefrontMap[y - 1][x];
        break;
      case DOWN_DIR:
        down = wavefrontMap[y + 1][x];
        break;
      default:
        leave = true;
        break;
    }
    if (leave) {
      break;
    }
  }

  //go in the directions that is exactly one less than the current wavefront value
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

/*
   asks for the user to input x and y coordinates for the goal and start
*/
void getEndPoint(void) {
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
  Robot.clearScreen();
}

/*
   moves one cell worth of distance foward
*/
void moveCell() {
  Robot.text("driving forward", 5, 80);
  Robot.motorsWrite(MOTOR_SPEED, MOTOR_SPEED);
  delay(CELL_TIME);
  Robot.motorsStop();
  delay(500);
}

/*
   turns 90 degrees in the inputted direction, unless it decides not to.
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

int defineSquare(bool walls[]) {
  bool upWall = walls[0];
  bool downWall = walls[1];
  bool leftWall = walls[2];
  bool rightWall = walls[3];
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
  return value;
}

/*
   detects the walls around Data
   wallDist =  [upDist,downDist,leftDist,rightDist]
   walls[] = [up,down,left,right]
*/
void detectWalls(int wallDist[], bool walls[]) {
  int upWallDist = wallDist[0];
  int downWallDist = wallDist[1];
  int leftWallDist = wallDist[2];
  int rightWallDist = wallDist[3];

  if (leftWallDist < WALL_DIST) {
    walls[2] = true;
  } else {
    walls[2] = false;
  }
  if (rightWallDist < WALL_DIST) {
    walls[3] = true;
  } else {
    walls[3] = false;
  }
  if (upWallDist < WALL_DIST) {
    walls[0] = true;
  } else {
    walls[0] = false;
  }
  if (downWallDist < WALL_DIST) {
    walls[1] = true;
  } else {
    walls[1] = false;
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

int findValue(bool walls[]) {
  int up = checkSonarPin(SONAR_FRONT);
  int left = checkSonarPin(SONAR_LEFT);
  int right = checkSonarPin(SONAR_RIGHT);
  int down = checkSonarPin(SONAR_BACK);

  int tempDir;

  //convert relative sensor info into global orientation
  switch (orientation) {
    case DOWN_DIR:
      tempDir = left;
      left = right;
      right = tempDir;
      tempDir = up;
      up = down;
      down = tempDir;
      break;
    case LEFT_DIR:
      tempDir = left;
      left = up;
      up = right;
      right = down;
      down = tempDir;
      break;
    case RIGHT_DIR:
      tempDir = right;
      right = up;
      up = left;
      left = down;
      down = tempDir;
      break;
  }
  int wallDist[4] = {up, down, left, right};
  detectWalls(wallDist, walls);
  return defineSquare(walls);
}


void addMap(int x, int y) {

  bool walls[4] = {0};
  int val = findValue(walls);
  if (x != 0 && walls[2]) {
    worldMap[y][x - 1] = 15;
  }
  if (x != 7 && walls[3]) {
    worldMap[y][x + 1] = 15;
  }
  if (y != 0 && walls[0]) {
    worldMap[y - 1][x] = 15;
  }
  if (y != 7 && walls[1]) {
    worldMap[y + 1][x] = 15;
  }

  worldMap[y][x] = val;
  return;

}

bool localize(int xIn, int yIn) {
  int possPos[32] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
  int numPos = 0;
  int iter2 = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (currentMap[j][i] == worldMap[yIn][xIn]) {
        possPos[iter2++] = i;
        possPos[iter2++] = j;
        numPos++;
        Robot.debugPrint(1, 15 * (i + 1), 9 * (j + 1));
      } else {
        Robot.debugPrint(0, 15 * (i + 1), 9 * (j + 1));
      }
    }
  }
  while (Robot.keyboardRead() != BUTTON_MIDDLE);
  Robot.clearScreen();
  iter2 = iter - 1;
  while (numPos > 1 && iter2 >= 0) {
    for (int i = 0; i < 32; i += 2) {
      if (possPos[i] == -1) {
        continue;
      }
      int xy[2];
      backTrace(xy, iter2);
      Robot.debugPrint(currentMap[possPos[i] + xy[1]][possPos[i + 1] + xy[0]]);
      Robot.debugPrint(worldMap[yIn + xy[1]][xIn + xy[0]], 1, 10);
      while (Robot.keyboardRead() != BUTTON_MIDDLE);
      Robot.clearScreen();
      if (currentMap[possPos[i] + xy[1]][possPos[i + 1] + xy[0]] != worldMap[yIn + xy[1]][xIn + xy[0]]) {
        possPos[i] = -1;
        possPos[i + 1] = -1;
        numPos--;
      }
    }
    iter2--;
  }
  if (numPos == 1) {
    int i = 0;
    while (possPos[i++] == -1);
    x = possPos[i - 1];
    y = possPos[i];
    return true;
  }
  return false;
}

void backTrace(int out[], int iter2) {
  int y = 0, x = 0;
  for (int i = iter; i >= iter2; i--) {
    switch (path[i]) {
      case UP_DIR://go up
        y++;
        break;
      case DOWN_DIR://go down
        y--;
        break;
      case LEFT_DIR://go left
        x++;
        break;
      case RIGHT_DIR://go right
        x--;
        break;
    }
  }
  out[0] = x;
  out[1] = y;
}


bool validDir(int x, int y, int dir) {
  switch (dir) {
    case UP_DIR:
      return worldMap[y - 1][x] != 15 && worldMap[y - 1][x] != 99;
      break;
    case DOWN_DIR:
      return worldMap[y + 1][x] != 15 && worldMap[y + 1][x] != 99;
      break;
    case LEFT_DIR:
      return worldMap[y][x - 1] != 15 && worldMap[y][x - 1] != 99;
      break;
    case RIGHT_DIR:
      return worldMap[y][x + 1] != 15 && worldMap[y][x + 1] != 99;
      break;
    default:
      return true;
      break;
  }

}

/*
  finds all the valid moves that can be performed at the input x and y coordinate
  x - x coordinate
  y - y coordinate
  moves - the array that the moves are added to
*/
void findMoves(int val, int moves[]) {
  if (val == 15) {
    return;
  }
  int i = 0;
  if (val != 2 && val != 3 && val != 6 &&
      val != 10 && val != 11 && val != 14) {
    moves[i++] = RIGHT_DIR;
  }

  if (val != 8 && val != 9 && val != 10 &&
      val != 11 && val != 12 && val != 13 && val != 14) {
    moves[i++] = LEFT_DIR;
  }
  if (val != 1 && val != 3 && val != 5 &&
      val != 7 && val != 9 && val != 11 && val != 13) {
    moves[i++] = UP_DIR;
  }

  if (val != 4 && val != 5 && val != 6 &&
      val != 7 && val != 12 && val != 13 && val != 14) {
    moves[i++] = DOWN_DIR;
  }
  return;
}
void printMap(int input[][7]) {
  Robot.clearScreen();

  //print out the map
  Robot.text("map", 5, 1);
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7; j++) {
      Robot.debugPrint(input[j][i], 15 * (1 + i), (j + 1) * 9);
    }
  }
  while (Robot.keyboardRead() != BUTTON_MIDDLE);
  Robot.clearScreen();
}

