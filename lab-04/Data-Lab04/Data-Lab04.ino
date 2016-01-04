#include <ArduinoRobot.h>

#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D2
#define SONAR_RIGHT D0

#define MIN_SONAR 483
#define MAX_IR 595
#define NUM_SAMPLES 5

#define KP 20
#define KD 42

#define MIN_DISTANCE 4
#define MAX_DISTANCE 6
#define DIFFERENCE_THRESHOLD 1

#define TURN_CALIBRATION 4.7 // time in ms to turn about 1 degree

#define WAIT_TIME 50
#define MAX_FORWARD_TIME 5000
#define MOVE_TIME 250

#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 125
#define TURN_SPEED 150
#define MOTOR_SPEED 200

#define MAX_ANGLE 180

#define INCREMENT_FORWARD 50
#define NUMBER_ITERATIONS 2
#define RANDOM_SPEED 150

#define DEBUG_TIME 2000


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

int iterator = 0;
int wallFollow = RANDOM; //start in random mode

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
  //delay(DEBUG_TIME);
  Robot.clearScreen();
}

void selectMode() {
  switch (wallFollow) {
    case RANDOM:
      randomKid();
      iterator++;
      break;

    case BOTH_WALLS:
      Robot.text("both walls", 5, 1);
      wallFollow = RANDOM;
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

  if ((front_left_distance < 2 * MAX_DISTANCE || left_distance < 2 * MAX_DISTANCE) && (front_right_distance < 2 * MAX_DISTANCE || right_distance < 2 * MAX_DISTANCE)) {
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

void wallFollowing(int wall) {
  double wallside_distance;
  double wallside_front_distance;
  double front_distance = checkSonarPin(SONAR_FRONT);
  Robot.debugPrint(front_distance, 5, 9);
  if (wall == RIGHT_WALL) {
    wallside_distance = checkSonarPin(SONAR_RIGHT);
    wallside_front_distance = checkIRPin(IR_FRONT_RIGHT) * sqrt(2) / 2;
    Robot.text("Following right wall", 5, 1);
  } else {
    wallside_distance = checkSonarPin(SONAR_LEFT);
    wallside_front_distance = checkIRPin(IR_FRONT_LEFT) * sqrt(2) / 2;
    Robot.text("Following left wall", 5, 1);
  }



  if (front_distance <= (MAX_DISTANCE + MIN_DISTANCE) / 2) {
    goToAngle(110 * wall);
    movement((MAX_DISTANCE+MIN_DISTANCE)/2,1);
  } else if (wallside_front_distance - wallside_distance <= DIFFERENCE_THRESHOLD || wallside_front_distance != 0) {
    double wallDistance  = (wallside_front_distance + wallside_distance) / 2; // average of the 2 sensors
    movement(wallDistance, wall);
  } else {
    movement(wallside_distance, wall);
  }

  if (wallside_distance > 2 * MAX_DISTANCE) {
    wallFollow = RANDOM;
  }

}

void movement(double wall_distance, int wall) {
  double move_spd_L = MOTOR_SPEED;
  double move_spd_R = MOTOR_SPEED;

  if (wall_distance < MIN_DISTANCE) {
    move_spd_L -= (wall_distance - MIN_DISTANCE) * KP * wall;
    move_spd_R += (wall_distance - MIN_DISTANCE) * KP * wall;
  }
  else if (wall_distance > MAX_DISTANCE) {
    move_spd_L += (MAX_DISTANCE - wall_distance) * KP * wall;
    move_spd_R -= (MAX_DISTANCE - wall_distance) * KP * wall;
  }

  if (move_spd_L > MAX_MOTOR_SPEED) {
    Robot.text("left too fast", 5, 9);
    move_spd_L = MAX_MOTOR_SPEED;
  } else if (move_spd_L < MIN_MOTOR_SPEED) {
    Robot.text("left too slow", 5, 9);
    move_spd_L = MIN_MOTOR_SPEED;
  }

  if (move_spd_R > MAX_MOTOR_SPEED) {
    move_spd_R = MAX_MOTOR_SPEED;
    Robot.text("right too fast", 5, 17);
  } else if (move_spd_R < MIN_MOTOR_SPEED) {
    move_spd_R = MIN_MOTOR_SPEED;
    Robot.text("right too slow", 5, 17);
  }
  Robot.motorsWrite(move_spd_L, move_spd_R);
  Robot.debugPrint(move_spd_L, 5, 26);
  Robot.debugPrint(move_spd_R, 5, 35);
  delay(MOVE_TIME);
  Robot.motorsStop();
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
    output = 99999999;
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
    output = 99999999;
  }
  return output;
}
