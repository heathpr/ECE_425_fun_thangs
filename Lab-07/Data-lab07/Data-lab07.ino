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

//dead band where the robot does not try to correct error
#define LOW_BAND 1
#define HIGH_BAND 1

#define WALL_DIST 12

bool leftWall = false;
bool rightWall = false;
bool frontWall = false;
int previousValue = 0;
char path[10];

int identifyState(void);

void setup() {
  // put your setup code here, to run once:
  identifyState();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void setPath(void){
  int i=0;
  while(1){
    if(Robot.keyboardRead()==BUTTON_LEFT){
      path[i]='L';
      delay(75);
      i++;
    }else if(Robot.keyboardRead()==BUTTON_RIGHT){
      path[i]='R';
      delay(75);
      i++;
    }else if(Robot.keyboardRead()==BUTTON_UP){
      path[i]='F';
      delay(75);
      i++;
    }else if(Robot.keyboardRead()==BUTTON_MIDDLE){
      path[i]='S';
      break;
    }
    if(i==9){
      break;
    }
  }
}

void detectWalls(void) {
  double leftWallDist = checkSonarPin(SONAR_LEFT);
  double rightWallDist = checkSonarPin(SONAR_RIGHT);
  double frontWallDist = checkSonarPin(SONAR_FRONT);

  if (leftWallDist < WALL_DIST) {
    leftWall = true;
  }
  if (rightWallDist < WALL_DIST) {
    rightWall = true;
  }
  if (frontWallDist < WALL_DIST) {
    frontWall = true;
  }
}

int identifyState(void) {
  int state = -1;
  if (leftWall && rightWall && !frontWall) {
    state = STRAIGHT_HALLWAY;
  } else if ( !leftWall && rightWall && frontWall) {
    state = RIGHT_CORNER;
  } else if (leftWall && !rightWall && frontWall) {
    state = LEFT_CORNER;
  } else if (leftWall && rightWall && frontWall) {
    state = DEAD_END;
  } else if (!leftWall && rightWall && !frontWall) {
    state = LEFT_HALL;
  } else if (leftWall && !rightWall && !frontWall) {
    state = RIGHT_HALL;
  } else if (!leftWall && !rightWall && !frontWall) {
    state = BOTH_HALL;
  } else {
    state = T_JUNCTION;
  }
  return state;
}

void bothWallFollowing() {
  //poll sensors
  double front_left_wall = checkIRPin(IR_FRONT_LEFT) * sqrt(2) / 2;;
  double front_right_wall = checkIRPin(IR_FRONT_RIGHT) * sqrt(2) / 2;;
  double left_wall = checkSonarPin(SONAR_LEFT);
  double right_wall = checkSonarPin(SONAR_RIGHT);


  //find the distance on either side of the walls
  double avg_dist = left_wall + right_wall;
  int divider = 2;

  //make sure that if we are taking the front side sensors that they are reading a value close to the sonar sensor
  if (abs(front_left_wall - left_wall) <= DIFFERENCE_THRESHOLD) {
    avg_dist += front_left_wall;
    divider++;
  }

  if (abs(front_right_wall - right_wall) <= DIFFERENCE_THRESHOLD) {
    avg_dist += front_right_wall;
    divider++;
  }

  avg_dist /= divider;
  movement(right_wall, avg_dist);
  previousValue = right_wall;
}

/*
   moves the robot using PD control depending on inputs

   wall_distance - distance from Data to the wall
   dir - direction to the wall (-1 left, 1 right)
   correct_distance - distance Data should be to the wall
*/
void movement(double wall_distance, double correct_distance) {
  //base speed
  double move_spd_L = MOTOR_SPEED;
  double move_spd_R = MOTOR_SPEED;

  Robot.debugPrint(wall_distance, 5, 50);
  Robot.debugPrint(correct_distance, 5, 59);

  //if robot is too close turn away
  if (wall_distance < correct_distance - LOW_BAND) {
    move_spd_L -= (wall_distance - (correct_distance)) * KP  - (wall_distance - previousValue) * KD ;
    move_spd_R += (wall_distance - (correct_distance)) * KP  + (wall_distance - previousValue) * KD ;
  }
  else if (wall_distance > correct_distance + HIGH_BAND) { // if robot is too far turn towards
    move_spd_L += ((correct_distance) - wall_distance) * KP  + (wall_distance - previousValue) * KD ;
    move_spd_R -= ((correct_distance) - wall_distance) * KP  - (wall_distance - previousValue) * KD ;
  }

  //uppper and lower bounds for motor speed
  if (move_spd_L > MAX_MOTOR_SPEED) {
    Robot.text("left too fast", 5, 100);
    move_spd_L = MAX_MOTOR_SPEED;
  } else if (move_spd_L < MIN_MOTOR_SPEED) {
    Robot.text("left too slow", 5, 100);
    move_spd_L = MIN_MOTOR_SPEED;
  }

  if (move_spd_R > MAX_MOTOR_SPEED) {
    move_spd_R = MAX_MOTOR_SPEED;
    Robot.text("right too fast", 5, 109);
  } else if (move_spd_R < MIN_MOTOR_SPEED) {
    move_spd_R = MIN_MOTOR_SPEED;
    Robot.text("right too slow", 5, 109);
  }

  //move command for the robot
  Robot.motorsWrite(move_spd_L, move_spd_R);
  Robot.debugPrint(move_spd_L, 5, 26);
  Robot.debugPrint(move_spd_R, 5, 35);
  delay(MOVE_TIME);
  Robot.motorsStop();
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

/*
   polls an IR pin

   pin - the pin to be polled
*/
double checkIRPin(int pin) {
  double output = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int value;
    value = Robot.analogRead(pin);

    value = 2517.3 / value - 1.7528;
    output += value;
    delay(5);
  }
  output = output / NUM_SAMPLES;
  if (output <= 0) {
    output = 99999998; // low output is set to a very large distance
  }
  return output + 1; // since IR sensors are about an inch more foward than the sonar pins add 1 to the final output
}

