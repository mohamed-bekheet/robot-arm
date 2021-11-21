#include <Arduino.h>
#include "states.h"
#ifndef _Control_H_
#define _Control_H_
//we will use PID controller to handle this sytem and minimize the driffting error
//in this system we use just 3 sensors thus the error will be simple and its region will be about
// 2~-2 best is 0

//in case us wanna update those values from esp server remove static and access them in hte main loop
static float _KP;
static float _KD;
static float _KI;

static float prop;
static float derv;
static float Integ;
static int E;//error
static int prevE;//previous error

extern float controlOUT = 0;//used to control the motor

void initPID(float kP,float kD,float kI){
    _KP=kP;
    _KD=kD;
    _KI=kI;
}

void calculate_error(){
    switch(state){
        case Finished://stop PID calculations for 10 seconds
            E=0;
            break;
        case Centered:
            E = 0;
            break;
        case Right_Shifted:
            E = 1;
            break;
        case Right_Drift:
            E = 2;
            break;
        case Left_Shifted:
            E = -1;
            break;
        case Left_Drift:
            E = -2;
            break;
        case OutOfPath://try to find a path or stop the robot
            E = 100;//out of band
            break;
        default:
            break;
    }
}

void updatePID(){ 
//we considered the time as constant in PID parameters
  prop = E;
  Integ = Integ + E;
  derv = E-prevE;
  controlOUT = (_KP*prop) + (_KD*derv) + (_KI*Integ) ;
  prevE = E;
}


void printPID_out(void){
    Serial.print("Error: ");
    Serial.println(E);
    Serial.print("OUTPUT: ");
    Serial.println(controlOUT);
    Serial.print("State: ");
    Serial.println(state);
    Serial.println("");

    delay(500);
}
#endif
