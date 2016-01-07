#include <ArduinoRobot.h>

#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D2
#define SONAR_RIGHT D0

#define MIN_SONAR 483
#define MAX_IR 595
#define NUM_SAMPLES 5

#define KP 15
#define KD 2

#define DIFFERENCE_THRESHOLD 5

#define INSIDE_ANGLE 125

#define CORRECT_DISTANCE 5
#define LOW_BAND 1
#define HIGH_BAND 1
#define INSIDE_THRESHOLD 7
#define BACK_THRESHOLD 2

#define TURN_CALIBRATION 4.7 // time in ms to turn about 1 degree

#define WAIT_TIME 50
#define MAX_FORWARD_TIME 5000
#define MOVE_TIME 200

#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 125
#define TURN_SPEED 150
#define MOTOR_SPEED 200

#define MAX_ANGLE 180

#define INCREMENT_FORWARD 50
#define NUMBER_ITERATIONS 2
#define RANDOM_SPEED 150


// possible modes the robot can be in
#define RIGHT_WALL 1
#define LEFT_WALL -1
#define BOTH_WALLS 0
#define RANDOM 2


double checkSonarPin(int pin);
double checkIRPin(int pin);
void goToAngle(int destination);
void movement(double wall_distance, double correct_distance);
void randomWander(void);
void selectMode(void);
void wallFollowing(int wall);
void bothWallFollowing(void);

int iterator = 0;
int wallFollow = RANDOM; //start in random mode
int previousValue = 0;

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
}

void loop() {
  selectMode();
  //delay(500);
  Robot.clearScreen();
}

void selectMode() {
  switch (wallFollow) {
    case RANDOM:
      randomKid();
      iterator++;
      break;

    case BOTH_WALLS:
      Robot.text("folowing both walls", 5, 1);
      bothWallFollowing();
      iterator = 0;
      break;

    default:
      wallFollowing(wallFollow);
      iterator = 0;
      break;
  }

}

void randomKid() {
  int front_distance = checkSonarPin(SONAR_FRONT);
  int front_left_distance = checkIRPin(IR_FRONT_LEFT);
  int front_right_distance = checkIRPin(IR_FRONT_RIGHT);
  int left_distance = checkSonarPin(SONAR_LEFT);
  int right_distance = checkSonarPin(SONAR_RIGHT);

  if (( left_distance < 2 * (CORRECT_DISTANCE + HIGH_BAND)) && ( right_distance < 2 * (CORRECT_DISTANCE + HIGH_BAND))) {
    wallFollow = BOTH_WALLS;
    Robot.motorsStop();
  } else if (front_left_distance < 2 * (CORRECT_DISTANCE + HIGH_BAND) || left_distance < 2 * (CORRECT_DISTANCE + HIGH_BAND)) {
    wallFollow = LEFT_WALL;
    Robot.motorsStop();
  } else if (front_right_distance < 2 * (CORRECT_DISTANCE + HIGH_BAND) || right_distance < 2 * (CORRECT_DISTANCE + HIGH_BAND)) {
    wallFollow = RIGHT_WALL;
    Robot.motorsStop();
  } else {
    Robot.text("Random Wander", 5, 1);
    randomWander(1 / front_distance);
  }

}

void randomWander(int avoid) {
  if (iterator % NUMBER_ITERATIONS == 0) {
    int angle = 0;
    angle = random(-MAX_ANGLE, MAX_ANGLE);
    goToAngle(angle);
  }
  // drive forward
  Robot.motorsWrite(RANDOM_SPEED + avoid, RANDOM_SPEED - avoid);
  delay(INCREMENT_FORWARD);
}

void bothWallFollowing() {
  double front_left_wall = checkIRPin(IR_FRONT_LEFT) * sqrt(2) / 2;;
  double front_right_wall = checkIRPin(IR_FRONT_RIGHT) * sqrt(2) / 2;;
  double left_wall = checkSonarPin(SONAR_LEFT);
  double right_wall = checkSonarPin(SONAR_RIGHT);

  double avg_dist = left_wall + right_wall;
  int divider = 2;
  if (abs(front_left_wall - left_wall) <= DIFFERENCE_THRESHOLD) {
    avg_dist += front_left_wall;
    divider++;
  }

  if (abs(front_right_wall - right_wall) <= DIFFERENCE_THRESHOLD) {
    avg_dist += front_right_wall;
    divider++;
  }

  avg_dist /= divider;

  if (left_wall > 3 * (CORRECT_DISTANCE + HIGH_BAND)) {
    previousValue = 0;
    wallFollow = RIGHT_WALL;
  } else if (right_wall > 3 * (CORRECT_DISTANCE + HIGH_BAND)) {
    previousValue = 0;
    wallFollow = LEFT_WALL;
  } else {
    movement(right_wall, avg_dist);
    previousValue = right_wall;
  }


}

void wallFollowing(int wall) {
  double wallside_distance;
  double wallside_front_distance;
  double front_distance = checkSonarPin(SONAR_FRONT);
  double opposite_side;

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

  if (opposite_side < CORRECT_DISTANCE + HIGH_BAND) {
    previousValue = 0;
    wallFollow = BOTH_WALLS;
  }



  if (front_distance <= INSIDE_THRESHOLD) {//approaching a wall (inside corner case)
    goToAngle(INSIDE_ANGLE * wall);
    straight(1);
  } else if (abs(wallside_front_distance - wallside_distance) <= DIFFERENCE_THRESHOLD) {//following the wall (normal wall follow state)
    double wallDistance;
    if (wallside_front_distance < wallside_distance) { //if Data is angled towards the wall we want to turn away harder
      wallDistance = wallside_front_distance;
    } else { //otherwise average the 2 values
      wallDistance = (wallside_front_distance + wallside_distance) / 2;
    }
    movement(wallDistance * wall, CORRECT_DISTANCE);
    previousValue = wallDistance;
    Robot.text("both sensors close", 5, 9);
  } else if (wallside_front_distance <= 2 * (CORRECT_DISTANCE + HIGH_BAND) &&
             wallside_distance > 2 * (CORRECT_DISTANCE + HIGH_BAND)) {
    //if only the IR sensor sees the wall use just the IR sensor (ending an outside turn)
    movement(wallside_front_distance * wall, CORRECT_DISTANCE);
    previousValue = wallside_front_distance * wall;
    Robot.text("Following IR", 5, 9);
  }
  else { // only sonar sees the wall or Data sees nothing (begining an outside turn or in the middle of a turn)
    movement(wallside_distance * wall, CORRECT_DISTANCE);
    movement(wallside_distance * wall, CORRECT_DISTANCE);
    previousValue = wallside_distance * wall;
    Robot.text("Following Sonar", 5, 9);
  }

  if (wallside_distance > 2 * (CORRECT_DISTANCE + HIGH_BAND) && previousValue > 2 * (CORRECT_DISTANCE + HIGH_BAND)) {
    wallFollow = RANDOM;
    previousValue = 0;
  }
}

void movement(double wall_distance, double correct_distance) {
  double move_spd_L = MOTOR_SPEED;
  double move_spd_R = MOTOR_SPEED;

  Robot.debugPrint(wall_distance, 5, 50);
  Robot.debugPrint(correct_distance, 5, 59);

  double mag = abs(wall_distance);
  int dir = wall_distance / mag;



  if (mag < correct_distance - LOW_BAND) {
    move_spd_L -= (mag - (correct_distance)) * KP * dir - (wall_distance - previousValue) * KD;
    move_spd_R += (mag - (correct_distance)) * KP * dir + (wall_distance - previousValue) * KD;
  }
  else if (mag > correct_distance + HIGH_BAND) {
    move_spd_L += ((correct_distance) - mag) * KP * dir + (wall_distance - previousValue) * KD;
    move_spd_R -= ((correct_distance) - mag) * KP * dir - (wall_distance - previousValue) * KD;
  }

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
  Robot.motorsWrite(move_spd_L, move_spd_R);
  Robot.debugPrint(move_spd_L, 5, 26);
  Robot.debugPrint(move_spd_R, 5, 35);
  delay(MOVE_TIME);
  Robot.motorsStop();
}

void straight(int dir) {
  Robot.motorsWrite(MOTOR_SPEED * dir, MOTOR_SPEED * dir);
  delay(MOVE_TIME);
  Robot.motorsStop();
}

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
    output = 99999998;
  }
  return output;
}

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
    output = 99999998;
  }
  return output + 1;
}
