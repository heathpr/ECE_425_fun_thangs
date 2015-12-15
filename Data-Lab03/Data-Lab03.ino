#include <ArduinoRobot.h>
#include <math.h>

#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D2
#define SONAR_RIGHT D0

#define MIN_SONAR 483
#define MAX_IR 595

#define NUM_SAMPLES 10

#define ANGLE_CALIBRATION 10
#define TURN_CALIBRATION 4.7 // time in ms to turn about 1 degree
#define MOVEMENT_CALIBRATION .3  //mutiliplier for run away run away movement
#define WAIT_TIME 50
#define MAX_MOTOR_SPEED 255
#define MAX_FORWARD_TIME 5000
#define MAX_ANGLE 180
#define TURN_SPEED 150
#define MOTOR_SPEED 200
#define MOVE_TIME 250
#define TRIGGER_RANGE 5
#define MIN_MOTOR_SPEED 125
#define INCREMENT_FORWARD 50
#define NUMBER_ITERATIONS 20

double checkSonarPin(int pin);
double checkIRPin(int pin);
void goToAngle(int destination);
void runAway();
void movement(double mag, double y);
void randomWander(void);
void selectMode(void);
void agressiveKid(void);


void setup() {
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
}

void loop() {
  selectMode();
  delay(WAIT_TIME);

}


void runAway() {
  double front_distance = checkSonarPin(SONAR_FRONT);
  if (front_distance > TRIGGER_RANGE) {
    front_distance = 0;
  }
  Robot.text("Front", 5, 50);
  Robot.debugPrint(front_distance, 50, 50);
  double front_left_distance = checkIRPin(IR_FRONT_LEFT);
  if (front_left_distance > TRIGGER_RANGE) {
    front_left_distance = 0;
  }
  Robot.text("Front left", 5, 59);
  Robot.debugPrint(front_left_distance, 100, 59);
  double front_right_distance = checkIRPin(IR_FRONT_RIGHT);
  if (front_right_distance > TRIGGER_RANGE) {
    front_right_distance = 0;
  }
  Robot.text("Front right", 5, 68);
  Robot.debugPrint(front_right_distance, 100, 68);
  double left_distance = checkSonarPin(SONAR_LEFT);
  if (left_distance > TRIGGER_RANGE) {
    left_distance = 0;
  }
  Robot.text("left", 5, 77);
  Robot.debugPrint(left_distance, 50, 77);
  double right_distance = checkSonarPin(SONAR_RIGHT);
  if (right_distance > TRIGGER_RANGE) {
    right_distance = 0;
  }
  Robot.text("right", 5, 86);
  Robot.debugPrint(right_distance, 50, 86);

  double y_vector = -(right_distance - left_distance + front_right_distance * sqrt(2) / 2 - front_left_distance * sqrt(2) / 2);
  double x_vector = -(front_distance + front_right_distance * sqrt(2) / 2 + front_left_distance * sqrt(2) / 2);
  Robot.debugPrint(x_vector, 5, 9);
  Robot.debugPrint(y_vector, 5, 17);

  double mag = sqrt(x_vector * x_vector + y_vector * y_vector);
  Robot.text("running away", 5, 1);
  if (mag < TRIGGER_RANGE && mag != 0) {
    movement(mag, y_vector);
  } else {
    Robot.debugPrint(mag, 5, 26);
  }

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
    value = .0071 * value - 1.4278;
    output += value;
    delay(5);
  }
  output = output / NUM_SAMPLES;
  if (output < 0) {
    output = 0;
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
  if (output < 0) {
    output = 0;
  }
  return output;
}


void movement(double mag, double y) {
  float move_spd = (5 - mag) * MOTOR_SPEED * MOVEMENT_CALIBRATION;
  Robot.motorsWrite(-move_spd + y * ANGLE_CALIBRATION, -move_spd - y * ANGLE_CALIBRATION);
  Robot.debugPrint(-move_spd + y * ANGLE_CALIBRATION, 5, 26);
  Robot.debugPrint(-move_spd - y * ANGLE_CALIBRATION, 5, 35);
  delay(MOVE_TIME);
  Robot.motorsStop();
}

void randomWander(int angleIterations) {
  int angle = 0;
  if (angleIterations % NUMBER_ITERATIONS == 0) {
    angle = random(-MAX_ANGLE, MAX_ANGLE);
    //change angle
    goToAngle(angle);
  }
  // drive forward
  Robot.motorsWrite(MOTOR_SPEED, MOTOR_SPEED);
  delay(INCREMENT_FORWARD);

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

void selectMode() {
  Robot.text("Up for shy", 5, 5);
  Robot.text("Left for aggressive", 5, 15);
  Robot.text("Down for random wander", 5, 25);
  Robot.text("Right for random obstacle", 5, 35);
  int button = Robot.keyboardRead();
  int i = 0;
  switch (button) {
    case BUTTON_UP:
      while (1) {
        runAway();
        Robot.clearScreen();
        button = Robot.keyboardRead();
        if (button == BUTTON_MIDDLE) {
          Robot.motorsStop();
          Robot.clearScreen();
          break;
        }
      }
      break;
    case BUTTON_LEFT:
      Robot.clearScreen();
      while (1) {
        agressiveKid();
        int button = Robot.keyboardRead();
        if (button == BUTTON_MIDDLE) {
          Robot.motorsStop();
          Robot.clearScreen();
          break;
        }
      }
      break;
    case BUTTON_DOWN:
      i = 0;
      while (1) {
        Robot.clearScreen();
        randomWander(i);
        button = Robot.keyboardRead();
        if (button == BUTTON_MIDDLE) {
          Robot.motorsStop();
          Robot.clearScreen();
          break;
        }
        i = i + 1;
      }
      break;
    case BUTTON_RIGHT:
      while (1) {
        Robot.clearScreen();
        //randomObstacle();
      }
      break;
  }

}

void agressiveKid() {
  Robot.text("agressive kid", 5, 5);
  bool isClear = false;
  int range = checkSonarPin(SONAR_FRONT);
  if (range <= TRIGGER_RANGE) {
    if (range == 0) {
      isClear = true;
    } else {
      isClear = false;
    }
  } else {
    isClear = true;
  }

  Robot.motorsWrite(isClear * MOTOR_SPEED, isClear * MOTOR_SPEED);
  range = checkSonarPin(SONAR_FRONT);
  delay(WAIT_TIME);
  Robot.clearScreen();

}

