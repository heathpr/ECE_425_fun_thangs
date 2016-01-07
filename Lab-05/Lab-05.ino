#include <ArduinoRobot.h>


#define PHOTO_LEFT B_TK2
#define PHOTO_RIGHT B_TK4

int right = -1;
int left = -1;
int readPhotoSensor(int pin);

void setup() {
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
}

void loop() {
  right=readPhotoSensor(PHOTO_RIGHT);
   delay(1000);
  Robot.clearScreen();
  left=readPhotoSensor(PHOTO_LEFT);
  
  delay(1000);
  Robot.clearScreen();

}

int readPhotoSensor(int pin) {
  int value;
  value = Robot.analogRead(pin);
  Robot.debugPrint(value,5,30);
  return value;
}

