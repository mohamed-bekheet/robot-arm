#include <Arduino.h>
#include "defines.h"
#include "states.h" //describe where is the robot on the line
#include "Control.h"//using Pid to handle the error
#include "motorMove.h"

void setup() {
  Serial.begin(9600);
  init_motor_pins();
  init_State();
  initPID(50,0,0);//kp,kd,ki
  delay(2000);
}

void loop() {
  
  updateSensors();
  updateState(); 
  calculate_error();
  updatePID();
  movementHandler();
  
  delay(70);
  Robot_stop();
  delay(70);

  /*
  printReadings();//used for debuging
  printPID_out();
  printSpeeds();
  */
}