#include <ArduinoRobot.h>

#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D2
#define SONAR_RIGHT D0

#define MIN_SONAR 483
#define MAX_IR 595
#define NUM_SAMPLES 5

#define KP 11
#define KD 1

#define MIN_DISTANCE 4
#define MAX_DISTANCE 6
#define DIFFERENCE_THRESHOLD 3

#define INSIDE_ANGLE 125

#define TURN_CALIBRATION 4.7 // time in ms to turn about 1 degree

#define WAIT_TIME 50
#define MAX_FORWARD_TIME 5000
#define MOVE_TIME 25

#define MAX_MOTOR_SPEED 175
#define MIN_MOTOR_SPEED 100
#define TURN_SPEED 150
#define MOTOR_SPEED 125
#define TURN_CONSTANT 2

#define MAX_ANGLE 180

#define INCREMENT_FORWARD 50
#define NUMBER_ITERATIONS 2
#define RANDOM_SPEED 150

#define INSIDE_THRESHOLD 7
#define BACK_THRESHOLD 2

#define LOW_BAND 3
#define HIGH_BAND 3


// possible modes the robot can be in
#define RIGHT_WALL 1
#define LEFT_WALL -1
#define BOTH_WALLS 0
#define RANDOM 2


double checkSonarPin(int pin);
double checkIRPin(int pin);
void goToAngle(int destination);
void movement(double wall_distance, int wall);
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
  //  delay(500);
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

  Robot.debugPrint(front_distance, 5, 120);
  Robot.debugPrint(front_left_distance, 5, 129);
  Robot.debugPrint(front_right_distance, 5, 129);
  Robot.debugPrint(left_distance, 5, 138);
  Robot.debugPrint(right_distance, 5, 147);

  if (( left_distance < 2 * MAX_DISTANCE) && ( right_distance < 2 * MAX_DISTANCE)) {
    wallFollow = BOTH_WALLS;
    Robot.motorsStop();
  } else if (front_left_distance < 2 * MAX_DISTANCE || left_distance < 2 * MAX_DISTANCE) {
    wallFollow = LEFT_WALL;
    Robot.motorsStop();
  } else if (front_right_distance < 2 * MAX_DISTANCE || right_distance < 2 * MAX_DISTANCE) {
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



  if (left_wall > 3 * MAX_DISTANCE) {
    previousValue = 0;
    wallFollow = RIGHT_WALL;
    Robot.motorsStop();
  } else if (right_wall > 3 * MAX_DISTANCE) {
    previousValue = 0;
    wallFollow = LEFT_WALL;
    Robot.motorsStop();
  } else {
    Robot.debugPrint(right_wall, 5, 9);
    Robot.debugPrint(avg_dist, 5, 17);
    movementdouble(avg_dist, right_wall);
    previousValue = right_wall;
  }


}

void wallFollowing(int wall) {
  double wallside_distance;
  double wallside_front_distance;
  double front_distance = checkSonarPin(SONAR_FRONT);
  double opposite_side;
  Robot.debugPrint(front_distance, 5, 9);
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

  if (opposite_side < 2 * MAX_DISTANCE) {
    previousValue = 0;
    wallFollow = BOTH_WALLS;
    Robot.motorsStop();
    return;
  }



  if (front_distance <= INSIDE_THRESHOLD) {
    goToAngle(INSIDE_ANGLE * wall);
    movement((MAX_DISTANCE + MIN_DISTANCE) / 2, 1);
  } else if (front_distance <= BACK_THRESHOLD || wallside_front_distance <= BACK_THRESHOLD) {
    straight(-1);
  } else if (wallside_front_distance - wallside_distance <= DIFFERENCE_THRESHOLD) {
    double wallDistance  = (wallside_front_distance + wallside_distance) / 2; // average of the 2 sensors
    movement(wallDistance, wall);
    previousValue = wallDistance;
  } else if (wallside_front_distance <= 2 * MAX_DISTANCE && wallside_distance > 2 * MAX_DISTANCE) {
    movement(wallside_front_distance, wall);
    previousValue = wallside_front_distance;
  } else {
    movement(wallside_distance + TURN_CONSTANT, wall);
    movement(wallside_distance + TURN_CONSTANT, wall);
    previousValue = wallside_distance;
  }

  if (wallside_distance > 3 * MAX_DISTANCE && previousValue > 3 * MAX_DISTANCE) {
    wallFollow = RANDOM;
    previousValue = 0;
    Robot.motorsStop();
  } else if (wallside_distance > 2 * MAX_DISTANCE) {
    movement(wallside_distance + TURN_CONSTANT, wall);
    straight(1);
  }
}

void movement(double wall_distance, int wall) {
  double move_spd_L = MOTOR_SPEED;
  double move_spd_R = MOTOR_SPEED;

  if (wall_distance < MIN_DISTANCE) {
    move_spd_L -= ((wall_distance - MIN_DISTANCE) * KP - (wall_distance - previousValue) * KD) * wall;
    move_spd_R += ((wall_distance - MIN_DISTANCE) * KP + (wall_distance - previousValue) * KD) * wall;
  }
  else if (wall_distance > MAX_DISTANCE) {
    move_spd_L += ((MAX_DISTANCE - wall_distance) * KP + (wall_distance - previousValue) * KD) * wall;
    move_spd_R -= ((MAX_DISTANCE - wall_distance) * KP - (wall_distance - previousValue) * KD) * wall;
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
  //Robot.motorsStop();
}


void movementdouble(double avg_dist, double wall_distance) {
  double move_spd_L = MOTOR_SPEED;
  double move_spd_R = MOTOR_SPEED;

  if (wall_distance < avg_dist - LOW_BAND) {
    move_spd_L -= ((wall_distance - avg_dist - LOW_BAND) * KP - (wall_distance - previousValue) * KD);
    move_spd_R += ((wall_distance - avg_dist - LOW_BAND) * KP + (wall_distance - previousValue) * KD);
  }
  else if (wall_distance > avg_dist + HIGH_BAND) {
    move_spd_L += ((avg_dist + HIGH_BAND - wall_distance) * KP + (wall_distance - previousValue) * KD);
    move_spd_R -= ((avg_dist + HIGH_BAND - wall_distance) * KP - (wall_distance - previousValue) * KD);
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
  //Robot.motorsStop();
}

void straight(int dir) {
  Robot.motorsWrite(MOTOR_SPEED * dir, MOTOR_SPEED * dir);
  delay(MOVE_TIME);
  //Robot.motorsStop();
}

void goToAngle(int angle) {
  Robot.debugPrint(angle, 5, 100);
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
    //Robot.debugPrint(value, 5, 40);
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
    //Robot.debugPrint(value, 5, 40);
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
