#include <Arduino.h>
#include "defines.h"
#ifndef _states_H_
#define _states_H_
typedef enum
{
    Finished,
    Centered,
    Right_Shifted,
    Right_Drift, //drifted
    Left_Shifted,
    Left_Drift,
    OutOfPath, //no line detected

} OnLineStates;

int sensor_values[no_sensors];

//best to be as an struct to handle all data related to sensor liske pin and value
//if you need to add a new sensor write its position name and edit in update sate function
typedef enum
{
    Front,
    Right1, //1>first sensor on the right of front sensor,2>second
    Left1,
} OnBoardSensor;

extern OnLineStates state = Finished;
OnBoardSensor sensor;

void init_State(void)
{
    for (int i = 0; i < no_sensors; i++)
        pinMode(sensor_pins[i], INPUT); //initialize all pins as INPUT
}

//in case of reading is 1 that means >> sensor receiver see (*black*) or long distance
//centered          R,L>>0 , F >> 1
//right_shifted
//left_shifted
void updateSensors(void)
{
    for (int i = Front; i < no_sensors; i++)
        sensor_values[i] = digitalRead(sensor_pins[i]);
}
void updateState()
{
    if (sensor_values[Front] && sensor_values[Right1] && sensor_values[Left1])//100
    {
        state = Finished;//stop for 10 seconds then start anew loop in the black path
    }
    else if (sensor_values[Front] && !sensor_values[Right1] && !sensor_values[Left1])//100
    {
        state = Centered;
    }
    else if (sensor_values[Front] && !sensor_values[Right1] && sensor_values[Left1])//101
    {
        state = Right_Shifted;
    }
    
    else if (!sensor_values[Front] && !sensor_values[Right1] && sensor_values[Left1])
    {
        state = Right_Drift;//huge
    }
    else if (sensor_values[Front] && sensor_values[Right1] && !sensor_values[Left1])
    {
        state = Left_Shifted;
    }
    else if (!sensor_values[Front] && !sensor_values[Right1] && sensor_values[Left1])
    {
        state = Left_Drift;
    }
    else
        state = OutOfPath;
}

void printReadings()
{
    Serial.print("S_R: ");
    Serial.println(sensor_values[Front]);
    Serial.print("S_F: ");
    Serial.println(sensor_values[Right1]);
    Serial.print("S_L: ");
    Serial.println(sensor_values[Left1]);
    Serial.println("");

    delay(500);
}

#endif