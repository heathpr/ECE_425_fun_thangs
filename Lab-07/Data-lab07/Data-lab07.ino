#include "ArduinoRobot.h"
#define STRAIGHT_HALLWAY 0
#define RIGHT_CORNER 1
#define LEFT_CORNER 2
#define DEAD_END 3
#define LEFT_HALL 4
#define RIGHT_HALL 5
#define BOTH_HALL 6
#define T_JUNCTION 7

bool leftWall = false;
bool rightWall = false;
bool frontWall = false;


int identifyState(void);

void setup() {
  // put your setup code here, to run once:
  identifyState();
}

void loop() {
  // put your main code here, to run repeatedly:

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
  movement(right_wall, RIGHT_WALL, avg_dist); //movement if switch case doesn't trigger
  previousValue = right_wall;
}

/*
   moves the robot using PD control depending on inputs

   wall_distance - distance from Data to the wall
   dir - direction to the wall (-1 left, 1 right)
   correct_distance - distance Data should be to the wall
*/
void movement(double wall_distance, int dir, double correct_distance) {
  //base speed
  double move_spd_L = MOTOR_SPEED;
  double move_spd_R = MOTOR_SPEED;

  Robot.debugPrint(wall_distance, 5, 50);
  Robot.debugPrint(correct_distance, 5, 59);

  //if robot is too close turn away
  if (wall_distance < correct_distance - LOW_BAND) {
    move_spd_L -= (wall_distance - (correct_distance)) * KP * dir - (wall_distance - previousValue) * KD * dir;
    move_spd_R += (wall_distance - (correct_distance)) * KP * dir + (wall_distance - previousValue) * KD * dir;
  }
  else if (wall_distance > correct_distance + HIGH_BAND) { // if robot is too far turn towards
    move_spd_L += ((correct_distance) - wall_distance) * KP * dir + (wall_distance - previousValue) * KD * dir;
    move_spd_R -= ((correct_distance) - wall_distance) * KP * dir - (wall_distance - previousValue) * KD * dir;
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

