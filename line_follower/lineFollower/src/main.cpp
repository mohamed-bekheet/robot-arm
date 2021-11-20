#include <Arduino.h>
#include "motorMove.h"
#include "defines.h"
#include "states.h" //describe where is the robot on the line
#include "Control.h"//using Pid to handle the error
#include "motorMove.h"
void setup() {
  Serial.begin(9600);
  init_motor_pins();
  init_State();
  initPID(60,30,0.01);//kp,kd,ki
 
}

void loop() {
  updateSensors();
  updateState();
  calculate_error();
  updatePID();
  movementHandler();
  //printReadings();//used for debuging
  
}