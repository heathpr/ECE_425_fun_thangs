#include <ArduinoRobot.h>


#define PHOTO_LEFT B_TK2
#define PHOTO_RIGHT B_TK4
#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 0
#define KP 3000
int right = -1;
int left = -1;
int readPhotoSensor(int pin);
void attraction(void);

void setup() {
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
}

void loop() {
  attraction();
  delay(15);
  Robot.clearScreen();

}

int readPhotoSensor(int pin) {
  int value;
  value = Robot.analogRead(pin);
  return value;
}

void attraction() {
  int rightDist = -1;
  int leftDist = -1;
  int leftSpeed;
  int rightSpeed;
  right = readPhotoSensor(PHOTO_RIGHT);
  left = readPhotoSensor(PHOTO_LEFT);
  rightDist = 154.948 - 0.167 * right;
  leftDist = 108.215 - 0.120 * left;
  rightSpeed = 1 / rightDist * KP + MIN_MOTOR_SPEED;
  leftSpeed = 1 / leftDist * KP + MIN_MOTOR_SPEED;
  if (rightSpeed > MAX_MOTOR_SPEED) {
    rightSpeed = MAX_MOTOR_SPEED;
  }
  if (leftSpeed > MAX_MOTOR_SPEED) {
    leftSpeed = MAX_MOTOR_SPEED;
  }
  Robot.debugPrint(rightSpeed, 5, 20);
  Robot.debugPrint(leftSpeed, 5, 80);
  Robot.motorsWrite(leftSpeed, rightSpeed);
}

