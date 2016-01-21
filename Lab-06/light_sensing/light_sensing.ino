#include <ArduinoRobot.h>


//define pins used
#define SONAR_FRONT D1
#define IR_FRONT_RIGHT M1
#define IR_FRONT_LEFT M3
#define SONAR_LEFT D2
#define SONAR_RIGHT D0
#define PHOTO_LEFT B_TK2
#define PHOTO_RIGHT B_TK4
#define KPLIGHT 150
#define LIGHT_SPEED 150

void setup() {
  // put your setup code here, to run once:
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);

}

void loop() {
  double front, left, right;

  //motor speeds
  int leftSpeed;
  int rightSpeed;

  //distance obtained by light sensors
  double rightDist = -1;
  double leftDist = -1;


  //get uncalibrated values
  right = readPhotoSensor(PHOTO_RIGHT);
  left = readPhotoSensor(PHOTO_LEFT);

  
  Robot.clearScreen();
  Robot.debugPrint(left, 5, 20);
  Robot.debugPrint(right, 5, 80);


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

