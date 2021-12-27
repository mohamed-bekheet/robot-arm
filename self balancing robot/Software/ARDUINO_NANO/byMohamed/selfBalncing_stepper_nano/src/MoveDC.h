#include <Arduino.h>
#include "defines.h"
#ifndef _MoveDC_H_
#define _MoveDC_H_

static int maxSpeed = 160; // 140; //255
static int minSpeed = 60;  // to avoid vibrations in the motor without moving

extern int speedR = 0;
extern int speedL = 0;

extern double speedT = 0; // total all wheels
extern int speedC = 20;  // speed for curve movment
extern int speedControlR = 0;
extern int speedControlL = 0;
int outR;
int outL;

void init_motor_pins()
{
  pinMode(pwm_r, OUTPUT);
  pinMode(in_r1, OUTPUT);
  pinMode(in_r2, OUTPUT);
  pinMode(pwm_l, OUTPUT);
  pinMode(in_l3, OUTPUT);
  pinMode(in_l4, OUTPUT);
}

void check_Boundries()
{
  if (outR >= maxSpeed) outR = maxSpeed;
  if (outR <= -maxSpeed) outR = maxSpeed; // 0;
  if (outL >= maxSpeed) outL = maxSpeed;
  if (outL <= -maxSpeed) outL = maxSpeed; // 0;
}
void check_BoundriesOUT(float controlR,float controlL)
{
  if (controlR >= maxSpeed) controlR = maxSpeed;
  if (controlR <= -maxSpeed) controlR = maxSpeed; // 0;
  if (controlL >= maxSpeed) controlL = maxSpeed;
  if (controlL <= -maxSpeed) controlL = maxSpeed; // 0;
}
void directionR(int dir)
{
  // if positive or 0 move forward
  if (dir >= 0)
  {
    digitalWrite(in_r1, LOW);
    digitalWrite(in_r2, HIGH);
  }
  else
  {
    digitalWrite(in_r1, HIGH);
    digitalWrite(in_r2, LOW);
  }
}

void directionL(int dir)
{
  // if positive or 0 move forward
  if (dir >= 0)
  {
    digitalWrite(in_l3, LOW);
    digitalWrite(in_l4, HIGH);
  }
  else
  {
    digitalWrite(in_l3, HIGH);
    digitalWrite(in_l4, LOW);
  }
}
void movePID(float ControlSignal)
{
  directionL(ControlSignal);
  directionR(ControlSignal);
  check_BoundriesOUT(ControlSignal,ControlSignal);
  analogWrite(pwm_r, abs(ControlSignal));
  analogWrite(pwm_l, abs(ControlSignal));
}
void moveF()
{
  directionL(1);
  directionR(1);
  
  analogWrite(pwm_r, abs( speedT+speedControlR));
  analogWrite(pwm_l, abs( speedT+speedControlL));
}
void moveB()
{
  directionL(-1);
  directionR(-1);
  analogWrite(pwm_r,abs( speedR+speedControlR));
  analogWrite(pwm_l,abs( speedL+speedControlL));
}
void moveR()
{
  directionL(-1);
  directionR(1);
  analogWrite(pwm_r, speedT);
  analogWrite(pwm_l, speedT);
}
void moveL()
{
  directionL(-1);
  directionR(1);
  analogWrite(pwm_r, speedT);
  analogWrite(pwm_l, speedT);
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