#include <Arduino.h>
#include "defines.h"//for pins and sensors
#include "states.h" //describe where is the robot on the line
#include "Control.h"//using Pid to handle the error
#include "motorMove.h"

void setup() {
  #if debug
  Serial.begin(9600);
  #endif
  init_motor_pins();
  init_State();
  initPID(40,0,0);//kp,kd,ki
  delay(2000);
}

void loop() {
  
  updateSensors();
  updateState(); 
  calculate_error();
  updatePID();
  movementHandler();
  
  #if debug
  //used for debuging
  printReadings();
  printPID_out();
  printSpeeds();
  #endif
}