#include <Arduino.h>
#include "Control.h"
#ifndef _motorMove_H_
#define _motorMove_H_

static int maxSpeed = 150; //140; //255
static int minSpeed = 0 ;  //to avoid vibrations in motor without moving

extern int speedR = 120;
extern int speedL = 80;

extern int speedT = 100; //total all wheels
extern int speedC = 20;  //speed for curve

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
  if (outR >= maxSpeed)
  {
    outR = maxSpeed;
  }
  if (outR <= minSpeed)
  {
    outR = minSpeed;// 0;
  }
  if (outL >= maxSpeed)
  {
    outL = maxSpeed;
  }
  if (outL <= minSpeed)
  {
    outL = minSpeed;// 0;
  }
}

void directionR(int dir)
{
  if (dir>=0)
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
  if (dir>=0)
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


void movePID_dir()
{
  directionR(controlOUT);
  directionL(-controlOUT);
  outR = speedR + controlOUT;
  outL = speedL - controlOUT;

  check_Boundries();
  analogWrite(pwm_r, outR);
  analogWrite(pwm_l, outL);
}

void movePID()
{

  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);

  outR = speedR + controlOUT;
  outL = speedL - controlOUT;

  check_Boundries();
  analogWrite(pwm_r, outR);
  analogWrite(pwm_l, outL);
}
void moveF()
{
  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);

  analogWrite(pwm_r, speedT);
  analogWrite(pwm_l, speedT);
}
void moveB()
{
  digitalWrite(in_r1, LOW);
  digitalWrite(in_r2, HIGH);

  digitalWrite(in_l3, LOW);
  digitalWrite(in_l4, HIGH);

  analogWrite(pwm_r, speedT);
  analogWrite(pwm_l, speedT);
}
void moveR()
{
  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);
  analogWrite(pwm_r, speedT + speedC);
  analogWrite(pwm_l, speedT - speedC);
}
void moveL()
{
  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);
  analogWrite(pwm_r, speedT - speedC);
  analogWrite(pwm_l, speedT + speedC);
}
void Robot_stop()
{
  analogWrite(pwm_r, 0);
  analogWrite(pwm_l, 0);
}

void movementHandler()
{
  
  switch (state)//state
  {
  case Finished: //stop PID calculations for 10 seconds
    Robot_stop();
    delay(20000); //10sec then start anew loop
    break;
  case OutOfPath: //try to find a path or stop the robot
    moveF();
    //moveR();
    break;
  default:
    movePID(); //apply PID here
    //movePID_dir();
    break;
  }
  
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