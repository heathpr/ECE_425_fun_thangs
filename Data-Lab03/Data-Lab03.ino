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




int motor_spd = 200; // normal motor speed
int turn_spd = 150; // speed of motors for turning
int move_time = 250;

double checkSonarPin(int pin);
double checkIRPin(int pin);
void goToAngle(int destination);
void runAway();
void movement(double mag, double y);


void setup() {
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
}

void loop() {
  runAway();
  //checkIRPin(IR_FRONT_LEFT, &FRONT_LEFT_LOCATION, FRONT_LEFT_SAMPLES);
  //delay(50);
  Robot.clearScreen();

}


void runAway() {
  double front_distance = checkSonarPin(SONAR_FRONT);
  if (front_distance > 5) {
    front_distance = 0;
  }
  Robot.text("Front", 5, 50);
  Robot.debugPrint(front_distance, 50, 50);
  double front_left_distance = checkIRPin(IR_FRONT_LEFT);
  if (front_left_distance > 5) {
    front_left_distance = 0;
  }
  Robot.text("Front left", 5, 59);
  Robot.debugPrint(front_left_distance, 100, 59);
  double front_right_distance = checkIRPin(IR_FRONT_RIGHT);
  if (front_right_distance > 5) {
    front_right_distance = 0;
  }
  Robot.text("Front right", 5, 68);
  Robot.debugPrint(front_right_distance, 100, 68);
  double left_distance = checkSonarPin(SONAR_LEFT);
  if (left_distance > 5) {
    left_distance = 0;
  }
  Robot.text("left", 5, 77);
  Robot.debugPrint(left_distance, 50, 77);
  double right_distance = checkSonarPin(SONAR_RIGHT);
  if (right_distance > 5) {
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
  if (mag < 5 && mag != 0) {
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
    //    if (value > MIN_SONAR) {
    //    Robot.debugPrint(value, 5, 9);
    value = .0071 * value - 1.4278;
    //    Robot.debugPrint(value, 5, 1);
    //    } else {
    //      value = 0;
    //    }
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
    //    if (value < MAX_IR) {
    value = 2517.3 / value - 1.7528;
    //    } else {
    //      value = 0;
    //    }
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
  float move_spd = (5 - mag) * motor_spd * MOVEMENT_CALIBRATION;
  //  if (sqrt(y * y) < 1) {
  //    Robot.motorsWrite(-move_spd, -move_spd);
  //    Robot.debugPrint(-move_spd, 5, 26);
  //    Robot.debugPrint(-move_spd, 5, 35);
  //  } else if (y > 0) {
  //
  //    Robot.motorsWrite(-move_spd * MOVEMENT_CALIBRATION * (5-y) * ANGLE_THING, -move_spd * MOVEMENT_CALIBRATION / (ANGLE_THING * (5-y)));
  //    Robot.debugPrint(-move_spd  * MOVEMENT_CALIBRATION * y * ANGLE_THING, 5, 26);
  //    Robot.debugPrint(-move_spd * MOVEMENT_CALIBRATION / (y * ANGLE_THING), 5, 35);
  //  } else {
  //    y=-y;
  //    Robot.motorsWrite(-move_spd * MOVEMENT_CALIBRATION / (ANGLE_THING * (5-y)), -move_spd * MOVEMENT_CALIBRATION * (5-y) * ANGLE_THING);
  //    Robot.debugPrint(-move_spd * MOVEMENT_CALIBRATION / (ANGLE_THING *(5-y)), 5, 26);
  //    Robot.debugPrint(-move_spd * MOVEMENT_CALIBRATION * (5-y) * ANGLE_THING, 5, 35);
  //  }
  Robot.motorsWrite(-move_spd + y * ANGLE_CALIBRATION, -move_spd - y * ANGLE_CALIBRATION);
  Robot.debugPrint(-move_spd + y * ANGLE_CALIBRATION, 5, 26);
  Robot.debugPrint(-move_spd - y * ANGLE_CALIBRATION, 5, 35);
  delay(move_time);
  Robot.motorsStop();
}



