/* Robot_Remote.ino
  Modified: Peter Heath and Matthew Schack and Data the robot
            12/3/15
            
*******************************************************
This code will use a remote control to create motion  *
and behavior on the Arduino robot                     *
*******************************************************

 ************************************************************************
 ***This example code is in an experimental state.                      *
 ***You are welcome to try this with your robot,                        *
 ***and no harm will come to it. We will provide a                      *
 ***detailed description of an updated version of this                  *
 ***in a future update                                                  *
 *** For this example to work you need:                                 *
 *** - download and install the IR-Remote library by Ken Shirriff       *
 ***   to be found at https://github.com/shirriff/Arduino-IRremote      *
 *** - get a Sony remote control                                        *
 *** This example will be updated soon, come back to the Robot          *
 *** page on the Arduino server for updates!!                           *
 ************************************************************************
 If you connect a IR receiver to the robot,                             
 you can control it like you control a TV set. 
 Using a Sony compatiable remote control, 
 map some buttons to different actions. 
 You can make the robot move around without 
 even touching it!

 Circuit: 
 * Arduino Robot
 * Connect the IRreceiver to TKD1
 * Sony compatible remote control

 based on the IRremote library
 by Ken Shirriff
 http://arcfn.com
 
 created 1 May 2013
 by X. Yang
 modified 12 May 2013
 by D. Cuartielles
 modified 3 December 2015
 by P. Heath and M. Schack
 
 This example is in the public domain
 ************************************************************************ 
 */

// include the necessary libraries
#include <IRremote.h>
#include <ArduinoRobot.h>

// Define a few commands from your remote control
#define IR_CODE_FORWARD 16621663      //UP ARROW
#define IR_CODE_BACKWARD 16625743    //DOWN ARROW
#define IR_CODE_SPIN_RIGHT 16584943   //LEFT ARROW
#define IR_CODE_SPIN_LEFT 16601263   //RIGHT ARROW
#define IR_CODE_TURN_RIGHT 16605343  //STOP MODE BUTTON
#define IR_CODE_TURN_LEFT 16589023   //SETUP BUTTON


int RECV_PIN = TKD1; // the pin the IR receiver is connected to
IRrecv irrecv(RECV_PIN); // an instance of the IR receiver object
decode_results results; // container for received IR codes
int move_time = 500;
int turn_time = 250;
int motor_spd = 200;

void setup() {
  // initialize the Robot, SD card, display, and speaker 
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("Remote Control Code:", 5, 5);
  Robot.text("Command:", 5, 26);
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {

    if (irrecv.decode(&results)) {
      Robot.text("code received",5,100);
      processResult(); // if there is an IR command, process it
      irrecv.resume(); // resume receiver
    }
}   


// this function takes the IR signals and then figures out which case this command relates to
// then executes that case.  
void processResult() {
  unsigned long res = results.value;
  // print the value to the screen
  Robot.debugPrint(res, 5, 15);
  //Serial.println(res,HEX);
  if(res == IR_CODE_FORWARD || res == IR_CODE_BACKWARD || res == IR_CODE_TURN_LEFT || res == IR_CODE_TURN_RIGHT) 
  {
    Robot.fill(255, 255, 255);
    Robot.stroke(255, 255, 255);
    Robot.rect(5, 36, 55, 10);
  }
  switch(results.value){
    //move forward
    case (IR_CODE_FORWARD):
      Robot.stroke(0, 0, 0);
      Robot.text("Forward", 5, 36);
      Robot.motorsWrite(motor_spd, motor_spd);
      delay(move_time);
      Robot.motorsStop();
      break;

    //move backward
    case (IR_CODE_BACKWARD):
      Robot.stroke(0, 0, 0);
      Robot.text("Backwards", 5, 36);
      Robot.motorsWrite(-motor_spd, -motor_spd);
      delay(move_time);
      Robot.motorsStop();
      break;

    // soin left
    case IR_CODE_SPIN_LEFT:
      Robot.stroke(0, 0, 0);
      Robot.text("Left", 5, 36);
      Robot.motorsWrite(-motor_spd, motor_spd); 
      delay(turn_time);
      Robot.motorsStop();
      break;
      
    // spin right
    case IR_CODE_SPIN_RIGHT:
      Robot.stroke(0, 0, 0);
      Robot.text("Right", 5, 36);
      Robot.motorsWrite(motor_spd, -motor_spd); 
      delay(turn_time);
      Robot.motorsStop();
      break;

    // turn left
    case IR_CODE_TURN_LEFT:
      Robot.stroke(0, 0, 0);
      Robot.text("Left", 5, 36);
      Robot.motorsWrite(motor_spd, motor_spd/2); 
      delay(turn_time);
      Robot.motorsStop();
      break;

    // turn right
    case IR_CODE_TURN_RIGHT:
      Robot.stroke(0, 0, 0);
      Robot.text("Right", 5, 36);
      Robot.motorsWrite(motor_spd/2, motor_spd); 
      delay(turn_time);
      Robot.motorsStop();
      break;
  }
}

