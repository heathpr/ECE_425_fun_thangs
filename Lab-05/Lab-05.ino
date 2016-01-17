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

//define motor constants
#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 0

//define multipliers for potential fields method
#define KP 2000
#define MOTOR_MULT 5


//define cases as Enums
#define LOVE 0
#define FEAR 1
#define AGGRESSION 2
#define EXPLORE 3


double readPhotoSensor(int pin);
void behaviour(int, bool);
void selectMode(bool);

void setup() {
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
}

/* 
 *  main loop function for Data prompts the user to decide if ostacle avoidance is nessecary
 *  then executes the selectMode function where the normal behaiors begin
*/ 
void loop() {
  bool obstacle;
  Robot.text("Up for obstacles", 5, 5);
  Robot.text("down for no obstacles", 5, 15);
  while (1) {
    int button = Robot.keyboardRead();
    if (button == BUTTON_UP) {
      obstacle = true;
      break;
    } else if (button == BUTTON_DOWN) {
      obstacle = false;
      break;
    }
  }
  Robot.clearScreen();
  delay(250);
  selectMode(obstacle);
  Robot.clearScreen();

}


/*
   selects the behavior the robot impliments then execute 
   that behaviours associated function.
   Data will stay in this function until program is reset
   obstacle - whether or not obstacle avoidence is running
*/
void selectMode(bool obstacle) {
  while (1) {
    Robot.text("Up for aggresion", 5, 5);
    Robot.text("Left for fear", 5, 15);
    Robot.text("Down for love", 5, 25);
    Robot.text("Right for explore", 5, 35);
    int button = Robot.keyboardRead();
    int i = 0;
    switch (button) {
      case BUTTON_UP: //aggression
        while (1) {
          behaviour(AGGRESSION, obstacle);
          Robot.clearScreen();
          button = Robot.keyboardRead();

          if (button == BUTTON_MIDDLE) { // break out of mode
            Robot.motorsStop();
            Robot.clearScreen();
            break;
          }
        }
        break;
      case BUTTON_LEFT: //FEAR
        Robot.clearScreen();
        while (1) {
          behaviour(FEAR, obstacle);
          button = Robot.keyboardRead();

          if (button == BUTTON_MIDDLE) { //break out of mode
            Robot.motorsStop();
            Robot.clearScreen();
            break;
          }
        }
        break;
      case BUTTON_DOWN: //love
        i = 0;
        while (1) {
          Robot.clearScreen();
          behaviour(LOVE, obstacle);
          button = Robot.keyboardRead();


          if (button == BUTTON_MIDDLE) { //break out of mode
            Robot.motorsStop();
            Robot.clearScreen();
            break;
          }
        }
        break;
      case BUTTON_RIGHT: //explore
        i = 0;
        while (1) {
          Robot.clearScreen();
          behaviour(EXPLORE, obstacle);
          button = Robot.keyboardRead();


          if (button == BUTTON_MIDDLE) { //break out of mode
            Robot.motorsStop();
            Robot.clearScreen();
            break;
          }
        }
        break;
    }
  }

}


/*
   Potential fields method to perform every behavior
   mode - behavior to impliment
   obstacle - whether obstacle avoidence is on
*/
void behaviour(int mode, bool obstacle) {

  double front, left, right, front_left, front_right, sonar_left, sonar_right;

  //motor speeds
  int leftSpeed;
  int rightSpeed;

  //distance obtained by light sensors
  double rightDist = -1;
  double leftDist = -1;

  if (obstacle) {//only ping non photosensors if obstacle avoidence is on
    front = checkSonarPin(SONAR_FRONT);
    front_left = checkIRPin(IR_FRONT_LEFT);
    front_right = checkIRPin(IR_FRONT_RIGHT);
    sonar_left = checkSonarPin(SONAR_LEFT);
    sonar_right = checkSonarPin(SONAR_RIGHT);
  }

  //get uncalibrated values
  right = readPhotoSensor(PHOTO_RIGHT);
  left = readPhotoSensor(PHOTO_LEFT);

  //calibrate them differently depending on the side
  rightDist = 154.948 - 0.167 * right;
  leftDist = 108.215 - 0.120 * left;


  //depending on the mode set the motor speeds depending on photo sensor values
  if (mode == LOVE) {
    rightSpeed = 1 / rightDist * KP + MIN_MOTOR_SPEED;
    leftSpeed = 1 / leftDist * KP + MIN_MOTOR_SPEED;
  } else if (mode == FEAR) {
    rightSpeed = -1 / rightDist * KP + MIN_MOTOR_SPEED;
    leftSpeed = -1 / leftDist * KP + MIN_MOTOR_SPEED;
  } else if (mode == AGGRESSION) {
    rightSpeed = 1 / leftDist * KP + MIN_MOTOR_SPEED;
    leftSpeed = 1 / rightDist * KP + MIN_MOTOR_SPEED;
  } else if (mode == EXPLORE) {
    rightSpeed = -1 / leftDist * KP + MIN_MOTOR_SPEED;
    leftSpeed = -1 / rightDist * KP + MIN_MOTOR_SPEED;
  }

  //if obstacle avoidence is on then add in the non-photo sensor values
  if (obstacle) {
    leftSpeed += KP / MOTOR_MULT * (1 / sonar_left - (sqrt(2) / 2) / front_left - 1 / front);
    rightSpeed += KP / MOTOR_MULT * (1 / sonar_right - (sqrt(2) / 2) / front_right - 1 / front);
  }

  //check to make sure nothing went out of bounds
  if (abs(rightSpeed) > MAX_MOTOR_SPEED) {
    if (rightSpeed > 0) {
      rightSpeed = MAX_MOTOR_SPEED;
    } else {
      rightSpeed = -MAX_MOTOR_SPEED;
    }
  } else if (abs(rightSpeed) < MIN_MOTOR_SPEED) {
    if (rightSpeed > 0) {
      rightSpeed = MIN_MOTOR_SPEED;
    } else {
      rightSpeed = -MIN_MOTOR_SPEED;
    }
  }

  if (abs(leftSpeed) > MAX_MOTOR_SPEED) {
    if (leftSpeed > 0) {
      leftSpeed = MAX_MOTOR_SPEED;
    } else {
      leftSpeed = -MAX_MOTOR_SPEED;
    }
  } else if (abs(leftSpeed) < MIN_MOTOR_SPEED) {
    if (leftSpeed > 0) {
      leftSpeed = MIN_MOTOR_SPEED;
    } else {
      leftSpeed = -MIN_MOTOR_SPEED;
    }
  }

  //print values and move
  Robot.debugPrint(rightSpeed, 5, 20);
  Robot.debugPrint(leftSpeed, 5, 80);
  Robot.motorsWrite(leftSpeed, rightSpeed);
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

