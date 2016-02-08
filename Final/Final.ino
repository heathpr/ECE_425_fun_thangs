/* Final Peter Heath, Matthew Schack, and Data
    last edited 2/7/16

    Based on the ArduinoRobot.h library for the Arduino Robot
*/
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
int x, y;
int path[16] = {0};
int iter = 0;
int topoMap[4][4] = {
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99},
  {99, 99, 99, 99}
};

int orientation = DOWN_DIR; //starting orientation
int nextSquare;


int identifyState(void);
void getXandY(void);
void createPath( int x, int y, int iter);
void printMap();

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
  getXandY();
  Robot.clearScreen();

  //create the path
  createPath(startX, startY, 0);
  x = startX;
  y = startY;
  int walls[4] = {0};
  addMap(x, y,walls);
  chooseDirection(walls);
}


void loop() {
  Robot.clearScreen();
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

  int walls[4] = {0};
  addMap(x,y,walls);
  chooseDirection(walls);
  delay(1000);
}

void chooseDirection(int walls[]){
  int[4] toGo ={99};
  for(int i=0;i<4;i++){
    if(!walls[i]){
      if(i<=1){
        toGo = 3-y;
      }else{
        toGo = 3-x;
      }
    }
  }
  nextSquare = UP_DIR;
  int dist = toGo[0];
  
}

void printMap() {
  Robot.clearScreen();

  //print out the map
  Robot.text("map", 5, 1);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Robot.debugPrint(inputMap[j][i], 15 * (1 + i), (j + 1) * 9);
    }
  }
}

/*
   asks for the user to input x and y coordinates for the goal and start
*/
void getXandY(void) {
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


void addMap(int x, int y, int walls[]) {
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
      tempDir = front;
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

  int wallsDist[4] = {up, down.left, right};
  detectWalls(wallDist, walls);
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
   detects the walls around Data
   wallDist =  [upDist,downDist,leftDist,rightDist]
   walls[] = [up,down,left,right]
*/
void detectWalls(int wallDist[], bool walls[]) {
  int upWallDist = wallDist[0]
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
  if (frontWallDist < WALL_DIST) {
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
    output = 99999998; // low output is set to a very large distance
  }
  return output;
}
