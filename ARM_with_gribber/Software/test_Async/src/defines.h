#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>
#include "esp32-hal-ledc.h"


#define motor11     26
#define motor12     25
#define motor21     33
#define motor22     32
    
#define PWM_IN1     2
#define PWM_IN2     3
#define PWM_IN3     4
#define PWM_IN4     5

#define PWM_RES     8

#define PWM_FREQ    1000

#define spd1 150                                           //left motor speed
#define spd2 150                                           //right motor speed

#define servo_pin       14
#define servo_channel   1
#define servo_freq      50

#define COUNT_LOW      1638
#define COUNT_HIGH     7864

#define TIMER_WIDTH    16

#define soundspeed 340                                      //sound speed for ultrasonic sensor
#define trig    13                                          //ultrasonic triger pin 
#define echo    12                                          //ultrasonic echo pin
#define max_distance  20        
