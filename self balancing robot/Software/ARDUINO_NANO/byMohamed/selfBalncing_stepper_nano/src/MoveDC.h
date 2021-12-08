#include <Arduino.h>
#include "defines.h"
#ifndef _MoveDC_H_
#define _MoveDC_H_

static int maxSpeed = 160; // 140; //255
static int minSpeed = 60;  // to avoid vibrations in the motor without moving

extern int speedR = 180;
extern int speedL = 180;

extern int speedT = 200; // total all wheels
extern int speedC = 20;  // speed for curve movment

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
  if (outR <= minSpeed) outR = minSpeed; // 0;
  if (outL >= maxSpeed) outL = maxSpeed;
  if (outL <= minSpeed) outL = minSpeed; // 0;
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

/*void movePID_dir()
{
  //get control signal from PID controller
  //change wheel direction while turning in curve
  directionR(controlOUT);
  directionL(-controlOUT);

  if (controlOUT || -controlOUT)
  {
    outR = speedR + controlOUT;
    outL = speedL - controlOUT;
  }
  else
  {
    outR = speedR;
    outL = speedL;
  }
  check_Boundries();
  analogWrite(pwm_r, outR);
  analogWrite(pwm_l, outL);
}

void movePID()
{
  //get control signal from PID controller
  directionR(controlOUT);
  directionL(controlOUT);

  if (controlOUT || -controlOUT)
  {
    outR = speedR + controlOUT;
    outL = speedL - controlOUT;
  }
  else
  {
    outR = speedR;
    outL = speedL;
  }

  check_Boundries();
  analogWrite(pwm_r, outR);
  analogWrite(pwm_l, outL);
}*/
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