#include <Arduino.h>
#include "defines.h"
#ifndef _MoveDC_H_
#define _MoveDC_H_

//right motor pins connected to arduino uno
#define pwm_r 5
#define in_r1 3
#define in_r2 4
//leftt motor pins connected to arduino uno
#define pwm_l 6
#define in_l3 7
#define in_l4 8

#define forward 1
#define backward -1

static int maxSpeed = 255; // 140; //255
static int minSpeed = 0;  // to avoid vibrations in the motor without moving

extern int speedR = 0;
extern int speedL = 0;

extern int speedT = 200; // total all wheels
extern int speedC = 20;  // speed for curve movment

int outR;
int outL;

void init_motor_pins()
{
  //right motor pins
  pinMode(pwm_r, OUTPUT);
  pinMode(in_r1, OUTPUT);
  pinMode(in_r2, OUTPUT);
  //left motor pins
  pinMode(pwm_l, OUTPUT);
  pinMode(in_l3, OUTPUT);
  pinMode(in_l4, OUTPUT);
}

void directionR(int dir)
{
  // if positive or 0 move forward
  if (dir >= 0)
  {
    digitalWrite(in_r1, HIGH);
    digitalWrite(in_r2, LOW);
  }
  else
  {
    digitalWrite(in_r1, LOW);
    digitalWrite(in_r2, HIGH);
  }
}

void directionL(int dir)
{
  // if positive or 0 move forward
  if (dir >= 0)
  {
    digitalWrite(in_l3, HIGH);
    digitalWrite(in_l4, LOW);
  }
  else
  {
    digitalWrite(in_l3, LOW);
    digitalWrite(in_l4, HIGH);
  }
}

void check_Boundries()
{
  if (outR >= maxSpeed) outR = maxSpeed;
  //if (outR <= -maxSpeed) outR = maxSpeed;
  if (outL >= maxSpeed) outL = maxSpeed;
  //if (outL <= -maxSpeed) outL = maxSpeed; 
}
/*
int check_Boundries( int _outR,int _outL)
{
  if (_outR >= maxSpeed) _outR = maxSpeed;
  if (_outR <= -maxSpeed) _outR = minSpeed;
  if (_outL >= maxSpeed) _outL = maxSpeed;
  if (_outL <= -maxSpeed) _outL = minSpeed; 
  return outR,outL;
}
*/
void movePID(float PID_out)
{
  //get control signal from PID controller
  int _outR,_outL;
  directionR(PID_out);
  directionL(PID_out);


 // check_Boundries();
  analogWrite(pwm_r, _outR);
  analogWrite(pwm_l, _outL);
}
void moveF()
{
  directionL(1);
  directionR(1);
  analogWrite(pwm_r, speedT);
  analogWrite(pwm_l, speedT);
}
void moveB()
{
  directionL(-1);
  directionR(-1);
  analogWrite(pwm_r, speedT);
  analogWrite(pwm_l, speedT);
}
void moveR()
{
  directionL(-1);
  directionR(1);
  analogWrite(pwm_r, speedT + speedC);
  analogWrite(pwm_l, speedT - speedC);
}
void moveL()
{
  directionL(-1);
  directionR(1);
  analogWrite(pwm_r, speedT - speedC);
  analogWrite(pwm_l, speedT + speedC);
}
void Robot_stop()
{
  analogWrite(pwm_r, 0);
  analogWrite(pwm_l, 0);
}
void printSpeeds()
{
  Serial.print("SpeedR: ");
  Serial.println(outR);
  Serial.print("SpeedL: ");
  Serial.println(outL);
  Serial.println("");

  delay(500);
}

#endif
