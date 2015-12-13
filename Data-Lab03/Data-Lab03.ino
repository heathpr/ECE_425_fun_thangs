#include <ArduinoRobot.h>
#include <math.h>

#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D10
#define SONAR_RIGHT D7

#define MIN_SONAR 483
#define MIN_IR 595

#define NUM_SAMPLES 10

#define TURN_CALIBRATION 4.7 // time in ms to turn about 1 degree
#define MOVEMENT_CALIBRATION .5  //mutiliplier for run away run away movement


int motor_spd = 200; // normal motor speed
int turn_spd = 150; // speed of motors for turning

double checkSonarPin(int pin, int location, double* array);
double checkIRPin(int pin, int location, double* array);
void goToAngle(int destination);
void runAway();
void moveForward(double mag);

double FRONT_SAMPLES[NUM_SAMPLES]  = {0};
double FRONT_LEFT_SAMPLES[NUM_SAMPLES]  = {0};
double LEFT_SAMPLES[NUM_SAMPLES]  = {0};
double FRONT_RIGHT_SAMPLES[NUM_SAMPLES]  = {0};
double RIGHT_SAMPLES[NUM_SAMPLES] = {0};

int FRONT_LOCATION = 0;
int FRONT_LEFT_LOCATION = 0;
int LEFT_LOCATION = 0;
int FRONT_RIGHT_LOCATION = 0;
int RIGHT_LOCATION = 0;


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
  Robot.clearScreen();
}


void runAway() {
  double front_distance = checkSonarPin(SONAR_FRONT, FRONT_LOCATION, FRONT_SAMPLES);
  double front_left_distance = checkIRPin(IR_FRONT_LEFT, FRONT_LEFT_LOCATION,FRONT_LEFT_SAMPLES);
  double front_right_distance = checkIRPin(IR_FRONT_RIGHT, FRONT_RIGHT_LOCATION,FRONT_RIGHT_SAMPLES);
  double left_distance = checkSonarPin(SONAR_LEFT, LEFT_LOCATION,LEFT_SAMPLES);
  double right_distance = checkIRPin(SONAR_RIGHT, RIGHT_LOCATION,RIGHT_SAMPLES);

  double x_vector = -(right_distance-left_distance + front_right_distance*sqrt(2)/2-front_left_distance*sqrt(2)/2);
  double y_vector = -(front_distance + front_right_distance*sqrt(2)/2+front_left_distance*sqrt(2)/2);

  double mag = sqrt(x_vector*x_vector+y_vector*y_vector);
  int angle =(int)(atan2(y_vector, x_vector) * 180 / M_PI);
  Robot.text("running away",5,1);
  Robot.debugPrint(mag,5,9);
  Robot.debugPrint(angle,5,17);

  goToAngle(angle);
  moveForward(mag);
}

double checkSonarPin(int pin, int location, double* array) {
  int value;
  pinMode(pin, OUTPUT);//set the PING pin as an output
  Robot.digitalWrite(pin, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  Robot.digitalWrite(pin, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  Robot.digitalWrite(pin, LOW);//set pin low first again
  pinMode(pin, INPUT);//set pin as input with duration as reception time
  value = pulseIn(pin, HIGH);//measures how long the pin is high
  if (value > MIN_SONAR) {
    value = .0071 * value - 1.4278;
  } else {
    value = 0;
  }
  array[location] = value;
  location++;
  double output;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    output += array[i];
  }
  output = output / NUM_SAMPLES;
  return output;
}

double checkIRPin(int pin, int location, double* array) {
  int value;
  value = Robot.analogRead(pin);
  if (value > MIN_IR) {
    value = .0004 / value + .0007;
  } else {
    value = 0;
  }
  array[location] = value;
  location++;
  double output=0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    output += array[i];
  }
  output = output / NUM_SAMPLES;
  return output;
}

void goToAngle(int angle) {
  float delayTime = angle * TURN_CALIBRATION; //find time to turn
  // decides what direction to turn
  if (angle > 0) {
    Robot.motorsWrite(turn_spd, -turn_spd);
    delay(delayTime);
  } else {
    Robot.motorsWrite(-turn_spd, turn_spd);
    delay(-delayTime);
  }
  Robot.motorsStop();
}

void moveForward(double mag){
  float delayTime = mag*MOVEMENT_CALIBRATION;
  Robot.motorsWrite(motor_spd,motor_spd);
  delay(delayTime);
  Robot.motorsStop();
}



