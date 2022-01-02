#include <Arduino.h>
//#include "WiFiManager.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>
#include "esp32-hal-ledc.h"
#ifndef _DEFINES_H_
#define _DEFINES_H_

#define station_mode 1
#define AP_mode 0
#define serial_debug 1

#define Ssid  "espAPP1"
#define Password  "123456789"

#define PARAM_MESSAGE1  "message1"
#define PARAM_MESSAGE2  "message2"
#define PARAM_MODE  "mode"
#define PARAM_VOICE  "voice"
#define PARAM_MOVE  "move"

#define PWM_RES     8
#define PWM_FREQ    1000


#define PWM_IN1     2
#define PWM_IN2     3
#define PWM_IN3     4
#define PWM_IN4     5


#define motor11     26
#define motor12     25
#define motor21     33
#define motor22     32


//pins
#define mode_pin 2 //change mode

#define servo_pin       15
#define servo_channel   1
#define servo_freq      50

#define COUNT_LOW      1638
#define COUNT_HIGH     7864

#define TIMER_WIDTH    16

#define soundspeed 340                                      //sound speed for ultrasonic sensor
#define trig    27                                         //ultrasonic triger pin 
#define echo    19                                          //ultrasonic echo pin
#define max_distance  20        


#define spd1 150                                           //left motor speed
#define spd2 150                                           //right motor speed


// the number of the LED pin
#define ARMservo1Pin  34
#define ARMservo2Pin  35
#endif