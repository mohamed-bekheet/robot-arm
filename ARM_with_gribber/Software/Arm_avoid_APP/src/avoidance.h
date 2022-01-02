#include "defines.h"
#ifndef __AVOIDANCE_H_
#define __AVOIDANCE_H_

extern float measured_distance = 0; // distance measured by ultrasonic
extern float Distance = 0;          // distance measured inside function by ultrasonic
extern float t = 0;                 // time traveled by sound wave
extern float distanceR = 0;         // distance measured by ultrasonic when servo looking right direction
extern float distanceL = 0;         // distance measured by ultrasonic when servo looking left direction



void avoidanceInit(void)
{
    ledcSetup(servo_channel, servo_freq, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
    ledcAttachPin(servo_pin, servo_channel);           // GPIO 14 assigned to channel

    pinMode(motor11, OUTPUT);
    pinMode(motor12, OUTPUT);
    pinMode(motor21, OUTPUT);
    pinMode(motor22, OUTPUT);

    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    ledcSetup(PWM_IN1, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_IN2, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_IN3, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_IN4, PWM_FREQ, PWM_RES);

    ledcAttachPin(motor11, PWM_IN1);
    ledcAttachPin(motor12, PWM_IN2);
    ledcAttachPin(motor21, PWM_IN3);
    ledcAttachPin(motor22, PWM_IN4);
}

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
    vTaskDelay(20 / portTICK_PERIOD_MS);//10ms
    digitalWrite(trig, HIGH);
    //vTaskDelay(1);//10 us
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
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Distance = measure_d() * 100;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ledcWrite(servo_channel, 5100);
    return Distance;
}

int lookLeft()
{
    ledcWrite(servo_channel, 3500);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Distance = measure_d() * 100;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ledcWrite(servo_channel, 5100);
    return Distance;
}

// rtos task
void avoidanceTask(void *parameter)
{

#if serial_debug
    Serial.print("avoidanceTask is running on core ");
    Serial.println(xPortGetCoreID());
#endif

    for (;;)
    {
#if serial_debug

        Serial.println("Hi from avoidance mode");
#endif

        if (measured_distance <= max_distance) // condition for detecting an object at distance equal or less than 15cm
        {
            move_stop(); // robot is stopping
            vTaskDelay(300 / portTICK_PERIOD_MS);
            distanceR = lookRight(); // servo looking right and measuring distance by ultrasonic for detecting object in right direction
            Serial.println(distanceR);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            distanceL = lookLeft(); // servo looking right and measuring distance by ultrasonic for detecting object in left direction
            Serial.println(distanceL);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            if (distanceR >= distanceL) // condition if there is no object in right side of the robot
            {
                if (distanceR <= 15)
                {
                    backward();
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    move_stop();
                    vTaskDelay(20 / portTICK_PERIOD_MS);
                    rot_r();
                    vTaskDelay(300 / portTICK_PERIOD_MS);
                    forward();
                }
                else
                {
                    rot_r(); // robot is rotating right for 300ms
                    vTaskDelay(300 / portTICK_PERIOD_MS);
                    forward();
                    move_stop();
                }
            }
            else if (distanceL >= distanceR) // condition if there is no object in left side of the robot
            {
                if (distanceL <= 15)
                {
                    backward();
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    move_stop();
                    vTaskDelay(20 / portTICK_PERIOD_MS);
                    rot_l();
                    vTaskDelay(300 / portTICK_PERIOD_MS);
                    forward();
                }
                else
                {
                    rot_l();
                    vTaskDelay(300 / portTICK_PERIOD_MS);
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

#if serial_debug 
        Serial.println(measured_distance);
#endif

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

#endif