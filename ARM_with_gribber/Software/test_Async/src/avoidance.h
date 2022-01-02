#pragma once
#include "defines.h"
#ifndef __AVOIDANCE_H_
#define __AVOIDANCE_H_

extern bool button = 0;
extern bool flag = 1;

extern float measured_distance = 0; // distance measured by ultrasonic
extern float Distance = 0;          // distance measured inside function by ultrasonic
extern float t = 0;                 // time traveled by sound wave
extern float distanceR = 0;         // distance measured by ultrasonic when servo looking right direction
extern float distanceL = 0;         // distance measured by ultrasonic when servo looking left direction

void forward()
{
  ledcWrite(PWM_IN1, spd1);
  ledcWrite(PWM_IN2, 0);
  ledcWrite(PWM_IN3, spd2);
  ledcWrite(PWM_IN4, 0);
}
void backward()
{
  ledcWrite(PWM_IN1, 0);
  ledcWrite(PWM_IN2, spd1);
  ledcWrite(PWM_IN3, 0);
  ledcWrite(PWM_IN4, spd2);
}
void move_stop()
{
  ledcWrite(PWM_IN1, 0);
  ledcWrite(PWM_IN2, 0);
  ledcWrite(PWM_IN3, 0);
  ledcWrite(PWM_IN4, 0);
}
void rot_r()
{
  ledcWrite(PWM_IN1, 0);
  ledcWrite(PWM_IN2, spd1);
  ledcWrite(PWM_IN3, spd2);
  ledcWrite(PWM_IN4, 0);
}
void rot_l()
{
  ledcWrite(PWM_IN1, spd1);
  ledcWrite(PWM_IN2, 0);
  ledcWrite(PWM_IN3, 0);
  ledcWrite(PWM_IN4, spd2);
}
float measure_d() // above door
{
  digitalWrite(trig, LOW);
  delay(10);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  t = pulseIn(echo, HIGH);
  t /= 1000000;                  // t in second
  Distance = soundspeed * t / 2; // distace in cm
  return Distance;
}
int lookRight()
{
  ledcWrite(servo_channel, 7100);
  delay(500);
  Distance = measure_d() * 100;
  delay(100);
  ledcWrite(servo_channel, 5100);
  return Distance;
}

int lookLeft()
{
  ledcWrite(servo_channel, 3500);
  delay(500);
  Distance = measure_d() * 100;
  delay(100);
  ledcWrite(servo_channel, 5100);
  return Distance;
}


//rtos task
void automance(void *parameter)
{
  Serial.print("automance is running on core ");
  Serial.println(xPortGetCoreID());
  for (;;)
  {
    Serial.println("Hi from autonomous mode");
    if (measured_distance <= max_distance) // condition for detecting an object at distance equal or less than 15cm
    {
      move_stop(); // robot is stopping
      delay(300);
      distanceR = lookRight(); // servo looking right and measuring distance by ultrasonic for detecting object in right direction
      Serial.println(distanceR);
      delay(200);
      distanceL = lookLeft(); // servo looking right and measuring distance by ultrasonic for detecting object in left direction
      Serial.println(distanceL);
      delay(200);
      if (distanceR >= distanceL) // condition if there is no object in right side of the robot
      {
        if (distanceR <= 15)
        {
          backward();
          delay(500);
          move_stop();
          delay(20);
          rot_r();
          delay(300);
          forward();
        }
        else
        {
          rot_r(); // robot is rotating right for 300ms
          delay(300);
          forward();
          move_stop();
        }
      }
      else if (distanceL >= distanceR) // condition if there is no object in left side of the robot
      {
        if (distanceL <= 15)
        {
          backward();
          delay(500);
          move_stop();
          delay(20);
          rot_l();
          delay(300);
          forward();
        }
        else
        {
          rot_l();
          delay(300);
          forward();
          move_stop(); // robot gonna rotating left for 300ms
        }
      }
    }
    else
    {
      forward(); // if there is no object in front robot is moving forward
    }
    measured_distance = measure_d() * 100; // measuring distance of object in front of the robot
    Serial.println(measured_distance);
    vTaskDelay(1);
  }
}

#endif