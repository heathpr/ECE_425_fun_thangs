#include <ArduinoRobot.h>

#define ftSonarPin D1

void setup() {
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  long value;
  pinMode(ftSonarPin, OUTPUT);//set the PING pin as an output
  Robot.digitalWrite(ftSonarPin, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  Robot.digitalWrite(ftSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  Robot.digitalWrite(ftSonarPin, LOW);//set pin low first again
  pinMode(ftSonarPin, INPUT);//set pin as input with duration as reception time
  value = pulseIn(ftSonarPin, HIGH);//measures how long the pin is high
  Robot.debugPrint(value);
  delay(1000);

}
