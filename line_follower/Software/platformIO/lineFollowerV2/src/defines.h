
/*
used for changing pins in microcontroller and number of sensors used in the system. 
*/
#ifndef _defines_H_
#define _defines_H_

#define debug 0 //write 1 to enable but becarful this slow down the robot it contain (serial print and delay)

//motor driver l298D has 3 pins for each motor 
//2 for control direction , 1 for enable and used as pwm control

//right motor pins connected to arduino uno
#define pwm_r 3
#define in_r1 4 
#define in_r2 5
//leftt motor pins connected to arduino uno
#define pwm_l 10
#define in_l3 9
#define in_l4 8


#define no_sensors 3 //numbers of sensors

//sensor pins just three sensors
#define sensorR A2
#define sensorF A1
#define sensorL A0

int sensor_pins[no_sensors] = {sensorF, sensorR, sensorL};

#endif