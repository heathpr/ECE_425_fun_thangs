#include "ArduinoRobot.h"
#define STRAIGHT_HALLWAY 0
#define RIGHT_CORNER 1
#define LEFT_CORNER 2
#define DEAD_END 3
#define LEFT_HALL 4
#define RIGHT_HALL 5
#define BOTH_HALL 6
#define T_JUNCTION 7

bool leftWall = false;
bool rightWall = false;
bool frontWall = false;


int identifyState(void);

void setup() {
  // put your setup code here, to run once:
  identifyState();
}

void loop() {
  // put your main code here, to run repeatedly:

}

int identifyState(void){
  int state=-1;
  if(leftWall && rightWall && !frontWall){
    state=STRAIGHT_HALLWAY;
  }else if( !leftWall && rightWall && frontWall){
    state=RIGHT_CORNER;
  }else if(leftWall && !rightWall && frontWall){
    state=LEFT_CORNER;
  }else if(leftWall && rightWall && frontWall){
    state=DEAD_END;
  }else if(!leftWall && rightWall && !frontWall){
    state=LEFT_HALL;
  }else if(leftWall && !rightWall && !frontWall){
    state=RIGHT_HALL;
  }else if(!leftWall && !rightWall && !frontWall){
    state=BOTH_HALL;
  }else{
    state=T_JUNCTION;
  }
  return state;
}

