/* Lab07 Peter Heath, Matthew Schack, and Data
    last edited 1/26/16

    Impliments wall following behavior based on PD control from sensor input
    as well implements behaviour to deal with various landmarks and to 
    navigate the given path by the user.
    
    Based on the ArduinoRobot.h library for the Arduino Robot
*/

//include header files
#include "ArduinoRobot.h"


//setup robot path and basic functionallity
void setup() {
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  Robot.stroke(0, 0, 0);
}

// main loop for the robot makes implements the finite state machine
void loop() {
  Robot.debugPrint(Robot.compassRead());
  delay(100); 

}


