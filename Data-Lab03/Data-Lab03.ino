/* Lab03 Matthew Schack, Peter Heath, and Data
    last edited 12/19/15

    Impliments driving behaviors based on sensor input including shy kid agressive kid
    random wander and random wander with obstacle avoidance behaviours.

    Based on the ArduinoRobot.h library for the the Arduino Robot
*/


// Grab nessecary header files
#include <ArduinoRobot.h>
#include <math.h>

//pins the sensors ar attached to
#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D2
#define SONAR_RIGHT D0

// min distances where good data can be read
#define MIN_SONAR 483
#define MAX_IR 595

//number of samples in a burst
#define NUM_SAMPLES 10

//multipliers for shy kid behavior
#define ANGLE_CALIBRATION 10
#define MOVEMENT_CALIBRATION .3

#define TURN_CALIBRATION 4.7 // time in ms to turn about 1 degree

#define WAIT_TIME 50 // time inbetween each main loop when choosing mode

// speeds for basic behaviours
#define TURN_SPEED 150
#define MOTOR_SPEED 200

// time to back up before another sensor read in obstacle ovoidence mode
#define MOVE_TIME 250

// distance that breaks out of random wander and begins obstacle avoidance
#define TRIGGER_RANGE 5

//constants for random behavior
#define MAX_FORWARD_TIME 5000
#define MAX_ANGLE 180
#define INCREMENT_FORWARD 50
#define NUMBER_ITERATIONS 2
#define RANDOM_SPEED 150
#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 125

// preallocate functions
void selectMode();
void randomWander(void);
void randomKid(int* i);
void runAway(int def);
void agressiveKid(void);
double checkSonarPin(int pin);
double checkIRPin(int pin);
void movement(double mag, double y);
void goToAngle(int destination);



// sets up initial parameters for the robot including LCD and speaker
void setup() {
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.beginSpeaker();
}

// Main function that drives the robots behaviors
void loop() {
  selectMode();
  delay(WAIT_TIME);

}

/*
    Function which uses LCD and robot keyboard to let the user
    decide what they would like the robot to do.  in addition it
    also provides an environment for the behaviours to run in.
    lastly it also provides the user a break function for the user by
    pressing the center button.
*/
void selectMode() {
  Robot.text("Up for shy", 5, 5);
  Robot.text("Left for aggressive", 5, 15);
  Robot.text("Down for random wander", 5, 25);
  Robot.text("Right for random obstacle", 5, 35);
  int button = Robot.keyboardRead();
  int i = 0;
  switch (button) {
    case BUTTON_UP: //shy kid
      while (1) {
        runAway(0);
        Robot.clearScreen();
        button = Robot.keyboardRead();

        if (button == BUTTON_MIDDLE) { // break out of mode
          Robot.motorsStop();
          Robot.clearScreen();
          break;
        }
      }
      break;
    case BUTTON_LEFT: //aggressive kid
      Robot.clearScreen();
      while (1) {
        agressiveKid();
        button = Robot.keyboardRead();

        if (button == BUTTON_MIDDLE) { //break out of mode
          Robot.motorsStop();
          Robot.clearScreen();
          break;
        }
      }
      break;
    case BUTTON_DOWN: //random wander
      i = 0;
      while (1) {
        Robot.clearScreen();

        randomWander(i);
        button = Robot.keyboardRead();
        i = i + 1;

        if (button == BUTTON_MIDDLE) { //break out of mode
          Robot.motorsStop();
          Robot.clearScreen();
          break;
        }
      }
      break;
    case BUTTON_RIGHT: //random wander with obsicle avoidance
      i = 0;
      while (1) {
        Robot.clearScreen();

        randomKid(&i);
        button = Robot.keyboardRead();
        i = i + 1;

        if (button == BUTTON_MIDDLE) { //break out of mode
          Robot.motorsStop();
          Robot.clearScreen();
          break;
        }
      }
      break;
  }

}

/*
    random wander behavior

    takes an iteration number so that it knows how long it has been running and when it can turn
    otherwise keep driving straight based on the random speed given

*/
void randomWander(int angleIterations) {
  if (angleIterations % NUMBER_ITERATIONS == 0) { // if we have iterated a predefined number of times then turn
    int angle = random(-MAX_ANGLE, MAX_ANGLE);
    goToAngle(angle);
  }

  // drive forward
  Robot.motorsWrite(RANDOM_SPEED, RANDOM_SPEED); // random speed is just a slower speed NOT a random number for speed
  delay(INCREMENT_FORWARD);

}
/*
  random wander with obstacle avoidance behavior

  very similar to the random wander behaviour but incorporates an obstacle avoidance check
  every iteration it is called.  if an obstacle is detected it then calls the shy kid behaviour
  to avoid the obstacle.

*/
void randomKid(int* i) {
  Robot.text("running away", 5, 1);
  //poll sensors
  double front_distance = checkSonarPin(SONAR_FRONT);
  double front_left_distance = checkIRPin(IR_FRONT_LEFT);
  double front_right_distance = checkIRPin(IR_FRONT_RIGHT);

  if (front_distance > TRIGGER_RANGE  && front_distance > TRIGGER_RANGE  && front_right_distance > TRIGGER_RANGE) { // if all of the sensors are outside the trigger range then random wander
    Robot.debugPrint(*i, 5, 1);
    randomWander(*i);
  } else { // otherwise change i to force an angle change next iteration and run shy kid behavior
    *i = NUMBER_ITERATIONS - 1; // -1 because after this look 1 will be added
    Robot.motorsStop();
    Robot.beep(BEEP_SIMPLE); // beep to let us know that we are avoiding an obsicle
    runAway(1);
  }

}

/*
    shy kid behavior

    def is whether the code is being run alone or in conjunction with random wander.
    implements shy kid behavior to run away from nearby obstacles.

*/
void runAway(int def) {
  Robot.text("running away", 5, 1);

  //poll all sensors and if they are greater than the TRIGGER_RAGE just set them to 0 (ignore the sensor)
  double front_distance = checkSonarPin(SONAR_FRONT);
  double right_distance = checkSonarPin(SONAR_RIGHT);
  double front_left_distance = checkIRPin(IR_FRONT_LEFT);
  double front_right_distance = checkIRPin(IR_FRONT_RIGHT);
  double left_distance = checkSonarPin(SONAR_LEFT);

  if (front_distance > TRIGGER_RANGE) {
    front_distance = 0;
  }
  if (front_left_distance > TRIGGER_RANGE) {
    front_left_distance = 0;
  }
  if (front_right_distance > TRIGGER_RANGE) {
    front_right_distance = 0;
  }
  if (left_distance > TRIGGER_RANGE) {
    left_distance = 0;
  }
  if (right_distance > TRIGGER_RANGE) {
    right_distance = 0;
  }

  // vectors pushing away per where the obstacles are
  double y_vector = -(right_distance - left_distance + front_right_distance * sqrt(2) / 2 - front_left_distance * sqrt(2) / 2);
  double x_vector = -(front_distance + front_right_distance * sqrt(2) / 2 + front_left_distance * sqrt(2) / 2);

  //magnitude of vectors added together
  double mag = sqrt(x_vector * x_vector + y_vector * y_vector);

  if ((mag < TRIGGER_RANGE && mag != 0) || def) { // if magnitude is less than the trigger range and not 0 (all sensored ignored) OR we are running this with another function
    movement(mag, y_vector); // move the motors
  } else {
    Robot.debugPrint(mag, 5, 26); // print the magnitude if it is too low to do anything
  }
}

/*
    aggressive kid behavior

    runs towards an object and stops when the front sensor is reading closer than the trigger range
*/
void agressiveKid() {
  Robot.text("agressive kid", 5, 5);

  bool isClear = false;
  int range = checkSonarPin(SONAR_FRONT);
  if (range <= TRIGGER_RANGE && range != 0) { // if within trigger range set value to stop
    isClear = false;
  } else {
    isClear = true; // set value to go
  }
  Robot.motorsWrite(isClear * MOTOR_SPEED, isClear * MOTOR_SPEED); // will be 0 if false
  delay(WAIT_TIME);
  Robot.clearScreen();
}

/*
      polls the given pin a set number of times and averages the result
      then returns the average distance read by the sonar sensor
*/
double checkSonarPin(int pin) {
  double output = 0;
  // gather all samples and process through calibration equation as well as
  //  use delays to avoid false triggering of the sensor
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


/*
      polls the given pin a set number of times and averages the result
      then returns the average distance read by the infrared sensor
*/
double checkIRPin(int pin) {
  double output = 0;
  // gather all samples and process through calibration equation
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
/*
    moves according to the potential fields method with the given magnitude and y vector
    only moves backwards.  uses a PD controler to acurately back away from an object
*/
void movement(double mag, double y) {

  float move_spd = (TRIGGER_RANGE - mag) * MOTOR_SPEED * MOVEMENT_CALIBRATION; // P control for the base speed

  // keep the movement speed within bounds and high enough to move
  if (move_spd < MIN_MOTOR_SPEED) {
    move_spd = MIN_MOTOR_SPEED;
  }
  if (move_spd > MAX_MOTOR_SPEED) {
    move_spd = MOTOR_SPEED;
  }
  Robot.motorsWrite(-move_spd + y * ANGLE_CALIBRATION, -move_spd - y * ANGLE_CALIBRATION); // differential for turning
  delay(MOVE_TIME);
  Robot.motorsStop();
}

/*
    goes to the given angle based on an open loop command using a timer
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
