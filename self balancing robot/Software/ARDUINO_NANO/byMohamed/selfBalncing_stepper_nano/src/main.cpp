#include <defines.h>

#if test_stepper_speed
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

int MOTOR_RPM = 300;
int MOTOR_R_RPM =300;
int MOTOR_L_RPM =300;

// X motor
#define DIR_L 2
#define STEP_L 4

// Y motor
#define DIR_R 5
#define STEP_R 6

// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperL(MOTOR_STEPS, DIR_L, STEP_L);
BasicStepperDriver stepperR(MOTOR_STEPS, DIR_R, STEP_R);

// Pick one of the two controllers below
// each motor moves independently, trajectory is a hockey stick
// MultiDriver controller(stepperX, stepperY);
// OR
// synchronized move, trajectory is a straight line
SyncDriver controller(stepperL, stepperR);

void setup() {
    /*
     * Set target motors RPM.
     */
    stepperR.begin(MOTOR_R_RPM, MICROSTEPS);
    stepperL.begin(MOTOR_L_RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
    // stepperX.setEnableActiveState(LOW);
    // stepperY.setEnableActiveState(LOW);
}

void loop() {

stepperL.setRPM(MOTOR_RPM);
controller.move(100,-100);
MOTOR_RPM++;
if(MOTOR_RPM>500)MOTOR_RPM = 0;
/*

    controller.rotate(90*5, 60*15);
    delay(1000);
    controller.rotate(-90*5, -30*15);
    delay(1000);
    controller.rotate(0, -30*15);
    delay(30000);
    */
}
#endif

#if test_stepper_PID

#include <PID_v1.h>
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

#define MOTOR_R_RPM 300
#define MOTOR_L_RPM 900

// X motor
#define DIR_L 2
#define STEP_L 4

// Y motor
#define DIR_R 5
#define STEP_R 6

// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperL(MOTOR_STEPS, DIR_L, STEP_L);
BasicStepperDriver stepperR(MOTOR_STEPS, DIR_R, STEP_R);

// Pick one of the two controllers below
// each motor moves independently, trajectory is a hockey stick
// MultiDriver controller(stepperX, stepperY);
// OR
// synchronized move, trajectory is a straight line
SyncDriver controller(stepperL, stepperR);

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID stepperPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    /*
     * Set target motors RPM.
     */
   
    stepperR.begin(MOTOR_R_RPM, MICROSTEPS);
    stepperL.begin(MOTOR_L_RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
    // stepperX.setEnableActiveState(LOW);
    // stepperY.setEnableActiveState(LOW);

    Setpoint = 100;//stepper position

    //turn the PID on
    //PID will work depending on varying stepper position
    stepperPID.SetMode(AUTOMATIC);
}

void loop() {
    stepperPID.Compute();
    //stepperR.setRPM(Output);
    //stepperR.setRPM(Output);
    controller.move(Output,-Output);//steps //PID change direction
    Input = controller.getCount();
    //delay(10000);
/*
    controller.rotate(90*5, 60*15);
    delay(1000);
    controller.rotate(-90*5, -30*15);
    delay(1000);
    controller.rotate(0, -30*15);
    delay(30000);*/
}


#endif

#if test_stepper
/*
 * Multi-motor control (experimental)
 *
 * Move two or three motors at the same time.
 * This module is still work in progress and may not work well or at all.
 *
 * Copyright (C)2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

#define MOTOR_R_RPM 300
#define MOTOR_L_RPM 300

// X motor
#define DIR_L 2
#define STEP_L 4

// Y motor
#define DIR_R 5
#define STEP_R 6

// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperL(MOTOR_STEPS, DIR_L, STEP_L);
BasicStepperDriver stepperR(MOTOR_STEPS, DIR_R, STEP_R);

// Pick one of the two controllers below
// each motor moves independently, trajectory is a hockey stick
// MultiDriver controller(stepperX, stepperY);
// OR
// synchronized move, trajectory is a straight line
SyncDriver controller(stepperL, stepperR);

void setup() {
    /*
     * Set target motors RPM.
     */
    stepperR.begin(MOTOR_R_RPM, MICROSTEPS);
    stepperL.begin(MOTOR_L_RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
    // stepperX.setEnableActiveState(LOW);
    // stepperY.setEnableActiveState(LOW);
}

void loop() {

//controller.move(10,100);
//delay(10000);

    controller.rotate(90*5, 60*15);
    delay(1000);
    controller.rotate(-90*5, -30*15);
    delay(1000);
    controller.rotate(0, -30*15);
    delay(30000);
}
#endif

#if test_IMU
#include <Arduino.h>
#include <AccelStepper.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 3  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
/*
XAccel			YAccel				ZAccel			XGyro			YGyro			ZGyro
  [-1043,-1042] --> [-2,17]
	[1231,1232] --> [-4,13]	
  [1303,1304] --> [16380,16406]

  [-42,-41] --> [-1,1]
  [39,40] --> [0,3]
  [-33,-32] --> [-1,2]
-------------- done --------------
*/
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-42);//220
    mpu.setYGyroOffset(39);//76
    mpu.setZGyroOffset(-33);//-85

    mpu.setZAccelOffset(1303); //1788 // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
#endif