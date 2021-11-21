#include<Arduino.h>
#ifndef _defines_H_
#define _defines_H_

//right motor pins connected to arduino uno
#define pwm_r 3
#define in_r1 4 
#define in_r2 5
//leftt motor pins connected to arduino uno
#define pwm_l 10
#define in_l3 9
#define in_l4 8

//sensor pins just three sensors
#define no_sensors 3 //numbers of sensors
#define sensorR A2
#define sensorF A1
#define sensorL A0

int sensor_pins[no_sensors] = {sensorF, sensorR, sensorL};

#endif