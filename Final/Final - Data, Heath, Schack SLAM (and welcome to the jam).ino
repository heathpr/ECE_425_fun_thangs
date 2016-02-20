/* Final Peter Heath, Matthew Schack, and Data
    last edited 2/20/16

    Impliments SLAM algorithms for a 4x4 maze.
    After the maze is completely mapped it uses wavefront propagation to move to a goal point.

    Based on the ArduinoRobot.h library for the Arduino Robot
*/
#include "ArduinoRobot.h"

//define pins used
#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D2
#define SONAR_RIGHT D0
#define SONAR_BACK D3

//define constants for sensor data
#define MIN_SONAR 483
#define NUM_SAMPLES 5

//motor speeds for movement
#define TURN_SPEED 150
#define MOTOR_SPEED 200

//max distance where data sees a wall
#define WALL_DIST 17

//movement times for 90 degree turns and 1 cell of movement
#define TURN_TIME 750
#define CELL_TIME 1550

//constants denoting which way to turn
#define LEFT 1
#define RIGHT -1

//movemnts the robot can do
#define UP_DIR 1
#define RIGHT_DIR 2
#define DOWN_DIR 3
#define LEFT_DIR 4

//delay for reading button pushes
#define BUTTON_TIME 100

//map making variables
int goalX = 0;
int goalY = 0;
int x = 2, y = 2; //assume we start at 2,2

//path planning variables
int path[16] = {0};
int iter = 0;

//topological map to be populated
int topoMap[4][4] = {
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99}
};

//wavefront map to be populated
int wavefrontMap[4][4] = {
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99}
};

int orientation = DOWN_DIR; //starting orientation
int nextSquare;
bool mapComplete = false;

int identifyState(void);
void getXandY(void);
void createPath( int x, int y, int iter);
void createMap(int x, int y, int i);
void printMap(int input[][4]);
void addMap();

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);

  getXandY();
  Robot.clearScreen();
  addMap();
  bool visited[4][4] = {0};
  chooseDirection(x, y, visited);
  printMap(topoMap);
}


void loop() {
  Robot.clearScreen();
  if (nextSquare == 0) {
    Robot.text("I have arrived at the goal", 1, 1);
    while (1);
  }
  //switch based off of the movements in the path
  switch (nextSquare) {
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
  orientation = nextSquare;
  
  if (!mapComplete) {
    bool visited[4][4] = {0};
    addMap();
    if (!chooseDirection(x, y, visited)) {//if choose directions returns false there is no more spots to map
      mapComplete = true;
      createMap(goalX, goalY, 0);
      createPath(x, y, 0);
      printMap(wavefrontMap);
      iter = 0;
      nextSquare = path[iter];
    }
  } else {
    nextSquare = path[++iter];
  }
  printMap(topoMap);
}

/*
   recursively chooses the direction to go to map the entire area
   if there is more than one direction to map we choose to go in the direction that is closer to the border
   we do this because the ones closer to the border are more likely to be dead ends so Data can map the maze quicker

   x - x coordinate
   y - y coordinate
   visited - an array of what locations have been previously visited by other recursive calls
*/
bool chooseDirection(int x, int y, bool visited[][4]) {
  //break case
  if (x < 0 || y < 0 || x > 3 || y > 3 || visited[y][x] || topoMap[y][x] == 15) {
    return false;
  }

  visited[y][x] = true;

  //find unmapped directions directly ajasent
  bool up = 0, down = 0, left = 0, right = 0;
  if (y > 0 && topoMap[y - 1][x] == 99) {
    up = 1;
  }
  if (y < 3 && topoMap[y + 1][x] == 99) {
    down = 1;
  }
  if (x > 0 && topoMap[y][x - 1] == 99) {
    left = 1;
  }
  if (x < 3 && topoMap[y][x + 1] == 99) {
    right = 1;
  }

  //if there are no unmapped directions directly ajasent
  if (!up && !down && !left && ! right) {
    //recursively call the function on each direction
    up = chooseDirection(x, y - 1, visited);
    down = chooseDirection(x, y + 1, visited);
    left = chooseDirection(x - 1, y, visited);
    right = chooseDirection(x + 1, y, visited);
  }
  //if there is more than one valid direction set the ones farther away from the border to false
  if (up) {
    if (down) {
      if (3 - y > y) {
        down = false;
      } else {
        up = false;
      }
    }
    if (left) {
      if (x > y) {
        left = false;
      } else {
        up = false;
      }
    }
    if (right) {
      if (3 - x > y) {
        right = false;
      } else {
        up = false;
      }
    }
  }

  if (down) {
    if (left) {
      if (x > 3 - y) {
        left = false;
      } else {
        down = false;
      }
    }
    if (right) {
      if (3 - x > 3 - y) {
        right = false;
      } else {
        down = false;
      }
    }
  }
  if (left) {
    if (right) {
      if (3 - x > x) {
        right = false;
      } else {
        left = false;
      }
    }
  }

  //set the next movement to the valid direction and return true
  if (up) {
    nextSquare = UP_DIR;
    return true;
  } else if (down) {
    nextSquare = DOWN_DIR;
    return true;
  } else if (left) {
    nextSquare = LEFT_DIR;
    return true;
  } else if (right) {
    nextSquare = RIGHT_DIR;
    return true;
  }

  //if at the end there is still no valid direction return false
  return false;
}

/*
   prints out the map and then waits for a button press to return

   intput - the map to print
*/
void printMap(int input[][4]) {
  Robot.clearScreen();

  //print out the map
  Robot.text("map", 5, 1);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Robot.debugPrint(input[j][i], 15 * (1 + i), (j + 1) * 9);
    }
  }
  while (Robot.keyboardRead() != BUTTON_MIDDLE);
}

/*
   asks for the user to input x and y coordinates for the goal and start
*/
void getXandY(void) {
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
 * adds a point to the topoMap
 */
void addMap() {
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
  bool walls[4] = {0};
  detectWalls(wallDist, walls);

  //if there is an opening where a outside border should be
  //shift the entire map and location
  if (x == 0 && !walls[2]) {
    shiftMap(RIGHT_DIR);
    Robot.text("shifting right", 1, 1);
    while (Robot.keyboardRead() != BUTTON_MIDDLE);
    x++;
  }
  if (x == 3 && !walls[3]) {
    shiftMap(LEFT_DIR);
    Robot.text("shifting left", 1, 1);
    while (Robot.keyboardRead() != BUTTON_MIDDLE);
    x--;
  }
  if (y == 0 && !walls[0]) {
    shiftMap(DOWN_DIR);
    Robot.text("shifting down", 1, 1);
    while (Robot.keyboardRead() != BUTTON_MIDDLE);
    y++;
  }
  if (y == 3 && !walls[1]) {
    shiftMap(UP_DIR);
    Robot.text("shifting up", 1, 1);
    while (Robot.keyboardRead() != BUTTON_MIDDLE);
    y--;
  }


  if (x != 0 && walls[2]) {
    topoMap[y][x - 1] = 15;
  }
  if (x != 3 && walls[3]) {
    topoMap[y][x + 1] = 15;
  }
  if (y != 0 && walls[0]) {
    topoMap[y - 1][x] = 15;
  }
  if (y != 3 && walls[1]) {
    topoMap[y + 1][x] = 15;
  }

  topoMap[y][x] = defineSquare(walls);

  return;
}

/*
 * shifts the topological map in the given direction
 */
void shiftMap(int dir) {
  int tempVal[4];
  for (int i = 0; i < 16; i++) {
    switch (dir) {
      case UP_DIR:
        if (i >= 12) {
          topoMap[i / 4][i % 4] = 99;
        } else {
          topoMap[i / 4][i % 4] = topoMap[1 + i / 4][i % 4];
        }
        break;
      case DOWN_DIR:
        if (i >= 12) {
          topoMap[3 - i / 4][i % 4] = 99;
        } else {
          topoMap[3 - i / 4][i % 4] = topoMap[3 - (1 + i / 4)][i % 4];
        }
        break;
      case LEFT_DIR:
        if (i >= 12) {
          topoMap[i % 4][i / 4] = 99;
        } else {
          topoMap[i % 4][i / 4] = topoMap[i % 4][1 + i / 4];
        }
        break;
      case RIGHT_DIR:
        if (i >= 12) {
          topoMap[i % 4][3 - i / 4] = 99;
        } else {
          topoMap[i % 4][3 - i / 4] = topoMap[i % 4][3 - (1 + i / 4)];
        }
        break;
    }
  }
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
   returns the topological representation of square

   walls - a boolean array of whether there are walls in each direction
*/
int defineSquare(bool walls[]) {
  bool upWall = walls[0];
  bool downWall = walls[1];
  bool leftWall = walls[2];
  bool rightWall = walls[3];
  int value;
  
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
   finds all the valid moves that can be performed at the input x and y coordinate
   x - x coordinate
   y - y coordinate
   moves - the array that the moves are added to
*/
void findMoves(int x, int y, int moves[]) {
  if (topoMap[y][x] == 15) {
    return;
  }
  int i = 0;
  if (topoMap[y][x] != 2 && topoMap[y][x] != 3 && topoMap[y][x] != 6 &&
      topoMap[y][x] != 10 && topoMap[y][x] != 11 && topoMap[y][x] != 14) {
    moves[i++] = RIGHT_DIR;
  }

  if (topoMap[y][x] != 8 && topoMap[y][x] != 9 && topoMap[y][x] != 10 &&
      topoMap[y][x] != 11 && topoMap[y][x] != 12 && topoMap[y][x] != 13 && topoMap[y][x] != 14) {
    moves[i++] = LEFT_DIR;
  }
  if (topoMap[y][x] != 1 && topoMap[y][x] != 3 && topoMap[y][x] != 5 &&
      topoMap[y][x] != 7 && topoMap[y][x] != 9 && topoMap[y][x] != 11 && topoMap[y][x] != 13) {
    moves[i++] = UP_DIR;
  }

  if (topoMap[y][x] != 4 && topoMap[y][x] != 5 && topoMap[y][x] != 6 &&
      topoMap[y][x] != 7 && topoMap[y][x] != 12 && topoMap[y][x] != 13 && topoMap[y][x] != 14) {
    moves[i++] = DOWN_DIR;
  }
  return;
}


/*
   recursively creates the wavefront map
   x - x coordinate
   y - y coordinate
   iter - wavefront value
*/
void createMap(int x, int y, int iter) {
  //break case
  if (x > 3 || x < 0 || y > 3 || y < 0 || topoMap[y][x] == 15 || wavefrontMap[y][x] < iter) {
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
  findMoves(x, y, moves);

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
  findMoves(x, y, moves);//only check directions where there is not a wall

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
