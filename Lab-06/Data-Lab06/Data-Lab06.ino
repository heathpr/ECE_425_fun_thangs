/* Lab04 Peter Heath, Matthew Schack, and Data
    last edited 1/9/16

    Impliments wall following behavior based on PD control from sensor input
    Can follow left wall, right wall, and both walls

    Based on the ArduinoRobot.h library for the Arduino Robot
*/


//grab header files
#include <ArduinoRobot.h>


//define pins used
#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D2
#define SONAR_RIGHT D0
#define PHOTO_LEFT B_TK2
#define PHOTO_RIGHT B_TK4

//define constants for sensor data
#define MIN_SONAR 483
#define MAX_IR 595
#define NUM_SAMPLES 5

// PD control constants
#define KP 15
#define KD 2

//define multipliers for potential fields method
#define KPLIGHT 5

//distance from the wall the robot should follow
#define CORRECT_DISTANCE 5

//dead band where the robot does not try to correct error
#define LOW_BAND 1
#define HIGH_BAND 1

#define DIFFERENCE_THRESHOLD 5 // threshold from front sonar and side sonar to go into straight wall behavior

#define INSIDE_THRESHOLD 7  // how close the robot gets to a inside corner before it turns
#define INSIDE_ANGLE 125 // angle to turn for inside corners

#define BACK_THRESHOLD 2  // threashold to back up

#define TURN_CALIBRATION 4.7 // time in ms to turn about 1 degree
#define MAX_ANGLE 180 // biggest angle robot can turn

#define LIGHT_THRESHOLD 20
#define DOCKING_THRESHOLD 10
#define WALL_THRESHOLD 10
#define TURN_THRESHOLD 10

//movement constants
#define WAIT_TIME 50
#define MAX_FORWARD_TIME 5000
#define MOVE_TIME 200

#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 125
#define TURN_SPEED 150
#define MOTOR_SPEED 200

#define LIGHT_SPEED 125

//wall constants
#define RIGHT_WALL 1
#define LEFT_WALL -1

// possible states the robot can be in
#define WALL 0
#define LIGHT 1
#define DOCK 2
#define RETURN 3
#define TURN_WALL 4


double checkSonarPin(int pin);
double checkIRPin(int pin);
void goToAngle(int destination);
void movement(double wall_distance, int wall, double correct_distance);
void selectMode(void);
void wallFollowing(int wall);
void lightFollow(void);
void dock(void);
void returnToWall(void);
void turnWall(int wall);


int iterator = 0;
int state = WALL; //start in wall mode
int wall;
int previousValue = 0; // previous value used for derivative control

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);


  double left = checkSonarPin(SONAR_LEFT);
  double right = checkSonarPin(SONAR_RIGHT);

  if (left < right) {
    wall = LEFT_WALL;
  } else {
    wall = RIGHT_WALL;
  }

}

void loop() {
  selectMode();
  Robot.clearScreen();
}


/*
   selects the mode Data is in
   choices are: following single wall, both walls, or random wander
   default case is following left or right walls
*/
void selectMode() {

  switch (state) {
    case WALL:
      wallFollowing(wall);
      break;
      
    case LIGHT:
      lightFollow();
      break;

    case DOCK:
      dock();
      break;

    case RETURN:
      returnToWall();
      break;
    case TURN_WALL:
    
      turnWall(wall);
      break;
  }

}


/*
   single wall following

   will follow 1 wall a maximum of 2*(CORRECT_DISTANCE+HIGH_BAND)

   wall - the side the wall is on (-1 for left 1 for right)
*/

void wallFollowing(int wall) {
  //poll sensors
  double wallside_distance;
  double wallside_front_distance;
  double front_distance = checkSonarPin(SONAR_FRONT);
  double opposite_side;

  //choose which sensors to poll depending on input
  if (wall == RIGHT_WALL) {
    wallside_distance = checkSonarPin(SONAR_RIGHT);
    wallside_front_distance = checkIRPin(IR_FRONT_RIGHT) * sqrt(2) / 2;
    opposite_side = checkSonarPin(SONAR_LEFT);
    Robot.text("Following right wall", 5, 1);
  } else {
    wallside_distance = checkSonarPin(SONAR_LEFT);
    wallside_front_distance = checkIRPin(IR_FRONT_LEFT) * sqrt(2) / 2;
    opposite_side = checkSonarPin(SONAR_RIGHT);
    Robot.text("Following left wall", 5, 1);
  }


  // choose what to do depending on what is approaching
  if (front_distance <= INSIDE_THRESHOLD) {//approaching a wall (inside corner case)
    goToAngle(INSIDE_ANGLE * wall);
    straight(1);
  } else if (abs(wallside_front_distance - wallside_distance) <= DIFFERENCE_THRESHOLD) {
    //following the wall (normal wall follow state)
    double wallDistance;
    if (wallside_front_distance < wallside_distance) { //if Data is angled towards the wall we want to turn away harder
      wallDistance = wallside_front_distance;
    } else { //otherwise average the 2 values
      wallDistance = (wallside_front_distance + wallside_distance) / 2;
    }
    movement(wallDistance, wall, CORRECT_DISTANCE);
    previousValue = wallDistance;
    Robot.text("both sensors close", 5, 9);
  } else if (wallside_front_distance <= 2 * (CORRECT_DISTANCE + HIGH_BAND) &&
             wallside_distance > 2 * (CORRECT_DISTANCE + HIGH_BAND)) {
    //if only the IR sensor sees the wall use just the IR sensor (ending an outside turn)
    movement(wallside_front_distance, wall, CORRECT_DISTANCE);
    previousValue = wallside_front_distance * wall;
    Robot.text("Following IR", 5, 9);
  }
  else { // only sonar sees the wall or Data sees nothing (begining an outside turn or in the middle of an outside turn)
    movement(wallside_distance, wall, CORRECT_DISTANCE);
    movement(wallside_distance, wall, CORRECT_DISTANCE);
    previousValue = wallside_distance * wall;
    Robot.text("Following Sonar", 5, 9);
  }


  int leftLight = readPhotoSensor(PHOTO_LEFT);
  int rightLight = readPhotoSensor(PHOTO_RIGHT);

  leftLight = 108.215 - 0.120 * leftLight;
  rightLight = 154.948 - 0.167 * rightLight;

  //switch case for mode changing
  if (leftLight < LIGHT_THRESHOLD || rightLight < LIGHT_THRESHOLD ) {
    state = LIGHT;
    previousValue = 0;
  }
}

/*
   Potential fields method to perform lightFollow

*/
void lightFollow() {

  double front, left, right;

  //motor speeds
  int leftSpeed;
  int rightSpeed;

  //distance obtained by light sensors
  double rightDist = -1;
  double leftDist = -1;

  front = checkSonarPin(SONAR_FRONT);


  //get uncalibrated values
  right = readPhotoSensor(PHOTO_RIGHT);
  left = readPhotoSensor(PHOTO_LEFT);

  //calibrate them differently depending on the side
  rightDist = 154.948 - 0.167 * right;
  leftDist = 108.215 - 0.120 * left;


  rightSpeed = -rightDist * KPLIGHT + LIGHT_SPEED;
  leftSpeed = -leftDist * KPLIGHT + LIGHT_SPEED;

  //check to make sure nothing went out of bounds
  rightSpeed = checkSpeed(rightSpeed);
  leftSpeed = checkSpeed(leftSpeed);

  //print values and move
  Robot.debugPrint(rightSpeed, 5, 20);
  Robot.debugPrint(leftSpeed, 5, 80);
  Robot.motorsWrite(leftSpeed, rightSpeed);

  if (front < DOCKING_THRESHOLD) {
    state = DOCK;
  }
}

void dock() {
  goToAngle(180);
  delay(5000);
  state = RETURN;
}

void returnToWall() {
  straight(1);
  double front, front_left, front_right;
  front = checkSonarPin(SONAR_FRONT);
  front_left = sqrt(2) / 2 * checkIRPin(IR_FRONT_LEFT);
  front_right = sqrt(2) / 2 * checkIRPin(IR_FRONT_RIGHT);

  if (front < WALL_THRESHOLD || front_left < WALL_THRESHOLD || front_right < WALL_THRESHOLD) {
    state = TURN_WALL;
  }
}

void turnWall(int wall) {
  double side;
  if (wall == LEFT_WALL) {
    side = checkSonarPin(SONAR_LEFT);
  } else {
    side = checkSonarPin(SONAR_RIGHT);
  }

  goToAngle(10);
  if (side < TURN_THRESHOLD) {
    state = WALL;
  }
}

int checkSpeed(int motorSpeed) {
  if (abs(motorSpeed) > MAX_MOTOR_SPEED) {
    if (motorSpeed > 0) {
      return MAX_MOTOR_SPEED;
    } else {
      return -MAX_MOTOR_SPEED;
    }
  } else if (abs(motorSpeed) < MIN_MOTOR_SPEED) {
    if (motorSpeed > 0) {
      return MIN_MOTOR_SPEED;
    } else {
      return -MIN_MOTOR_SPEED;
    }
  }

  return motorSpeed;
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




/*
   moves Data in a straight line

   dir - forward or backwards (1 is forward -1 is backwards
*/
void straight(int dir) {
  Robot.motorsWrite(MOTOR_SPEED * dir, MOTOR_SPEED * dir);
  delay(MOVE_TIME);
  Robot.motorsStop();
}


/*
   move Data to an angle

   angle - angle to trun to
*/
void goToAngle(int angle) {
  Robot.motorsStop();
  float delayTime = angle * TURN_CALIBRATION; //find time to turn
  // decides what direction to turn
  if (angle > 0) {
    Robot.motorsWrite(TURN_SPEED, -TURN_SPEED);
    delay(delayTime);
  } else {
    Robot.motorsWrite(-TURN_SPEED, TURN_SPEED);
    delay(-delayTime);
  }
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


/*
   reads the photo sensor at the given pin and returns the uncalibrated value
   pin- the name of the pin to read
*/
double readPhotoSensor(int pin) {
  double value;
  value = Robot.analogRead(pin);
  return value;
}




