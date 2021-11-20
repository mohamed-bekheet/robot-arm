#include <Arduino.h>
#include "Control.h"
#ifndef _motorMove_H_
#define _motorMove_H_

static int maxSpeed =200;//255
static int minSpeed = 75;//to avoid vibrations in motor without moving

extern int speedR = 120;
extern int speedL = 120;
extern int speedT = 100;//total all wheels
extern int speedC = 20;//speed for curve

void init_motor_pins()
{
    pinMode(pwm_r, OUTPUT);
    pinMode(in_r1, OUTPUT);
    pinMode(in_r2, OUTPUT);
    pinMode(pwm_l, OUTPUT);
    pinMode(in_l3, OUTPUT);
    pinMode(in_l4, OUTPUT);
}

void check_Boundries(){
  if(speedR >=  maxSpeed || speedL >=  maxSpeed ) {speedR = maxSpeed;speedL = maxSpeed;}
  if(speedR <=  minSpeed || speedL <=  minSpeed ) {speedR = minSpeed;speedL = minSpeed;}
}

void movePID(){

  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);

  check_Boundries();
  analogWrite(pwm_r, speedR+controlOUT);
  analogWrite(pwm_l, speedL-controlOUT);
}

void moveF() {
  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);

  analogWrite(pwm_r, speedT);
  analogWrite(pwm_l, speedT);
}
void moveB() {
  digitalWrite(in_r1, LOW);
  digitalWrite(in_r2, HIGH);

  digitalWrite(in_l3, LOW);
  digitalWrite(in_l4, HIGH);

  analogWrite(pwm_r, speedT);
  analogWrite(pwm_l, speedT);
}
void moveR() {
  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);
  analogWrite(pwm_r, speedT+speedC);
  analogWrite(pwm_l, speedT-speedC);
}
void moveL() {
  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);
  analogWrite(pwm_r, speedT-speedC);
  analogWrite(pwm_l, speedT+speedC);
}
void Robot_stop() {
  analogWrite(pwm_r, 0);
  analogWrite(pwm_l, 0);

}


void movementHandler(){
  
      switch(state){
        case Finished://stop PID calculations for 10 seconds
            E=0;
            Robot_stop();
            delay(10000);//10sec then start anew loop
            moveF(); 
            break;
        case Centered:
            moveF();
            break;
        case OutOfPath://try to find a path or stop the robot
           moveF();
           moveR();
            break;
        default:
             movePID();//apply PID here
            break;
    }
  
}
#endif