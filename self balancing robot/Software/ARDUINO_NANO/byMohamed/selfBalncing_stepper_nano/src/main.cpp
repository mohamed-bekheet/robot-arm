
// choose what code you want to run from defines.h file write 1 for desied and 0 for others
#include "defines.h"

#if test_DC_PID_IMU_esp01
#include "Arduino.h"
#include "MoveDC.h"
#include <SoftwareSerial.h>

SoftwareSerial espSerial(10, 11); // RX, TX
#include "Arduino.h"
#include "MoveDC.h"

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
////////////////////IMU///////////////////
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

/////////PID////////////
float Kp = 2.8;
float Ki = 0;
float Kd = 1.3;

float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

float E;
float derE;
float integE;

double prevE = 0;
double deltaT;

double curT = 0;
double prevT;

float angle_gyro, realAngle;

void setup()
{
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    init_motor_pins();

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(9600);
    espSerial.begin(115200);
    while (!Serial)
        ; // wait for Leonardo enumeration, others continue immediately

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
    XAccel      YAccel        ZAccel      XGyro     YGyro     ZGyro
      [-1043,-1042] --> [-2,17]
        [1231,1232] --> [-4,13]
      [1303,1304] --> [16380,16406]

      [-42,-41] --> [-1,1]
      [39,40] --> [0,3]
      [-33,-32] --> [-1,2]
    -------------- done --------------
    */
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-42); // 220
    mpu.setYGyroOffset(39);  // 76
    mpu.setZGyroOffset(-33); //-85

    mpu.setZAccelOffset(1303); // 1788 // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
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
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    digitalWrite(LED_PIN, HIGH);
}

void loop()
{
    if (espSerial.available())
    {
        // meaasge example
        //" kp15."
        String esp_message, sub_mess;
        esp_message = espSerial.readStringUntil('*');
        sub_mess = esp_message.substring(2, esp_message.indexOf("*"));
        sub_mess.toFloat();

        if (esp_message.startsWith("kp"))
        {
            Kp = sub_mess.toFloat();
        }
        else if (esp_message.startsWith("kd"))
        {
            Kd = sub_mess.toFloat();
        }
        else if (esp_message.startsWith("ki"))
        {
            Ki = sub_mess.toFloat();
        }
    }

    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("r\t");
        Serial.println(ypr[1] * 180 / M_PI);
    }

    angle_gyro = ypr[1] * 180 / M_PI;
    if (angle_gyro <= 1.5 && angle_gyro >= -1.5)
        angle_gyro = 0;

    if (angle_gyro == 0)
    {
        realAngle = 0;
    }
    else
    {
        realAngle = angle_gyro; //+5.6; //5.6 is best so far    //To compensate IMU's offset
    }

    E = realAngle;
    derE = (E - prevE);
    integE = integE + E;
    pid_output = Kp * E + Kd * derE + Ki * integE;
    prevE = E;

    movePID(pid_output);
}

#endif

#if test_DC_PID_IMU
#include "Arduino.h"
#include "MoveDC.h"

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
////////////////////IMU///////////////////
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

/////////PID////////////
#define Kp 2.8
#define Ki 0
#define Kd 1.3

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;

float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

float E;
float derE;
float integE;

double prevE = 0;
double deltaT;

double curT = 0;
double prevT;

float angle_gyro;

void setup()
{
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    init_motor_pins();

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(9600);
    while (!Serial)
        ; // wait for Leonardo enumeration, others continue immediately

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
    mpu.setXGyroOffset(-42); // 220
    mpu.setYGyroOffset(39);  // 76
    mpu.setZGyroOffset(-33); //-85

    mpu.setZAccelOffset(1303); // 1788 // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
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
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    digitalWrite(LED_PIN, HIGH);
}

void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("r\t");
        Serial.println(ypr[2] * 180 / M_PI);
    }

    angle_gyro = ypr[2] * 180 / M_PI;
    if (angle_gyro <= 3 && angle_gyro >= -3)
        angle_gyro = 0;

    E = angle_gyro;
    derE = (E - prevE);
    integE = integE + E;
    pid_output = Kp * E + Kd * derE + Ki * integE;
    prevE = E;

    // if(pid_output <= 0.5*Kp && pid_output >= -0.5*Kp)pid_output= 0;

    pid_output_left = pid_output;  // Copy the controller output to the pid_output_left variable for the left motor
    pid_output_right = pid_output; // Copy the controller output to the pid_output_right variable for the right motor

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Motor pulse calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
    if (pid_output_left > 0)
        pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
    else if (pid_output_left < 0)
        pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

    if (pid_output_right > 0)
        pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
    else if (pid_output_right < 0)
        pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;

    // Calculate the needed pulse time for the left and right stepper motor controllers
    if (pid_output_left > 0)
        left_motor = 400 - pid_output_left;
    else if (pid_output_left < 0)
        left_motor = -400 - pid_output_left;
    else
        left_motor = 0;

    if (pid_output_right > 0)
        right_motor = 400 - pid_output_right;
    else if (pid_output_right < 0)
        right_motor = -400 - pid_output_right;
    else
        right_motor = 0;

    // Copy the pulse time to the throttle variables so the interrupt subroutine can use them
    outR = left_motor;
    outL = right_motor;

    // check_Boundries();
    analogWrite(pwm_r, outR);
    analogWrite(pwm_l, outL);
}

#endif

#if test_DC_speed_direction
#include "Arduino.h"
#include "MoveDC.h"
void setup()
{
    init_motor_pins();
}
void loop()
{
    init_motor_pins();
    moveF();
    delay(5000);
    moveB();
    delay(5000);
}

#endif

#if test_stepper_PID_timers

#include <Arduino.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
////////////////////IMU///////////////////
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 3 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

/////////PID////////////
#define Kp 2.8
#define Ki 0
#define Kd 1.3

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;

float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

float E;
float derE;
float integE;

double prevE = 0;
double deltaT;

double curT = 0;
double prevT;

float angle_gyro;

void setup()
{

    TWBR = 12; // Set the I2C clock speed to 400kHz

    // To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
    // This subroutine is called TIMER2_COMPA_vect
    TCCR2A = 0;              // Make sure that the TCCR2A register is set to zero
    TCCR2B = 0;              // Make sure that the TCCR2A register is set to zero
    TIMSK2 |= (1 << OCIE2A); // Set the interupt enable bit OCIE2A in the TIMSK2 register
    TCCR2B |= (1 << CS21);   // Set the CS21 bit in the TCCRB register to set the prescaler to 8
    OCR2A = 39;              // The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
    TCCR2A |= (1 << WGM21);  // Set counter 2 to CTC (clear timer on compare) mode

    pinMode(2, OUTPUT); // Configure digital poortD 2 as output
    pinMode(4, OUTPUT); // Configure digital poortD 3 as output
    pinMode(5, OUTPUT); // Configure digital poortD 4 as output
    pinMode(6, OUTPUT); // Configure digital poortD 5 as output

    pinMode(13, OUTPUT); // Configure digital poortD 6 as output

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(9600);
    while (!Serial)
        ; // wait for Leonardo enumeration, others continue immediately

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
    mpu.setXGyroOffset(-42); // 220
    mpu.setYGyroOffset(39);  // 76
    mpu.setZGyroOffset(-33); //-85

    mpu.setZAccelOffset(1303); // 1788 // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
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
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("r\t");
        Serial.println(ypr[2] * 180 / M_PI);
    }

    angle_gyro = ypr[2] * 180 / M_PI;
    if (angle_gyro <= 3 && angle_gyro >= -3)
        angle_gyro = 0;

    E = angle_gyro;
    derE = (E - prevE);
    integE = integE + E;
    pid_output = Kp * E + Kd * derE + Ki * integE;
    prevE = E;

    // if(pid_output <= 0.5*Kp && pid_output >= -0.5*Kp)pid_output= 0;

    pid_output_left = pid_output;  // Copy the controller output to the pid_output_left variable for the left motor
    pid_output_right = pid_output; // Copy the controller output to the pid_output_right variable for the right motor

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Motor pulse calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
    if (pid_output_left > 0)
        pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
    else if (pid_output_left < 0)
        pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

    if (pid_output_right > 0)
        pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
    else if (pid_output_right < 0)
        pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;

    // Calculate the needed pulse time for the left and right stepper motor controllers
    if (pid_output_left > 0)
        left_motor = 400 - pid_output_left;
    else if (pid_output_left < 0)
        left_motor = -400 - pid_output_left;
    else
        left_motor = 0;

    if (pid_output_right > 0)
        right_motor = 400 - pid_output_right;
    else if (pid_output_right < 0)
        right_motor = -400 - pid_output_right;
    else
        right_motor = 0;

    // Copy the pulse time to the throttle variables so the interrupt subroutine can use them
    throttle_left_motor = left_motor;
    throttle_right_motor = right_motor;
}

ISR(TIMER2_COMPA_vect)
{

    // DIR,Step pins.
    // 2,4 pins//left motor
    // 5,6 pins//right motor
    // Left motor pulse calculations

    throttle_counter_left_motor++; // Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
    if (throttle_counter_left_motor > throttle_left_motor_memory)
    {                                                     // If the number of loops is larger then the throttle_left_motor_memory variable
        throttle_counter_left_motor = 0;                  // Reset the throttle_counter_left_motor variable
        throttle_left_motor_memory = throttle_left_motor; // Load the next throttle_left_motor variable
        if (throttle_left_motor_memory < 0)
        { // If the throttle_left_motor_memory is negative
            // PORTD &= 0b11011111;
            PORTD &= ~(1 << 2);               // Set output 5 high for a forward direction of the stepper motor
            throttle_left_motor_memory *= -1; // Invert the throttle_left_motor_memory variable
        }
        else
            PORTD |= (1 << 2); // Set output 5 low to reverse the direction of the stepper controller
    }
    // change step case
    else if (throttle_counter_left_motor == 1)
        PORTD |= (1 << 4); // Set output 2 high to create a pulse for the stepper controller
    else if (throttle_counter_left_motor == 2)
        PORTD &= ~(1 << 4); // Set output 2 low because the pulse only has to last for 20us

    // right motor pulse calculations
    throttle_counter_right_motor++; // Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
    if (throttle_counter_right_motor > throttle_right_motor_memory)
    {                                                       // If the number of loops is larger then the throttle_right_motor_memory variable
        throttle_counter_right_motor = 0;                   // Reset the throttle_counter_right_motor variable
        throttle_right_motor_memory = throttle_right_motor; // Load the next throttle_right_motor variable
        if (throttle_right_motor_memory < 0)
        {                                      // If the throttle_right_motor_memory is negative
            PORTD |= (1 << 5);                 // Set output 5 high for a forward direction of the stepper motor
            throttle_right_motor_memory *= -1; // Invert the throttle_right_motor_memory variable
        }
        else
            PORTD &= ~(1 << 5); // Set output 5 low to reverse the direction of the stepper controller
    }
    else if (throttle_counter_right_motor == 1)
        PORTD |= (1 << 6); // Set output 4 high to create a pulse for the stepper controller
    else if (throttle_counter_right_motor == 2)
        PORTD &= ~(1 << 6); // Set output 4 low because the pulse only has to last for 20us
}

#endif

#if test_stepper_speed_timers

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

// PID
// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID stepperPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
    /*
     * Set target motors RPM.
     */

    stepperR.begin(MOTOR_R_RPM, MICROSTEPS);
    stepperL.begin(MOTOR_L_RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
    // stepperX.setEnableActiveState(LOW);
    // stepperY.setEnableActiveState(LOW);

    Setpoint = 100; // stepper position

    // turn the PID on
    // PID will work depending on varying stepper position
    stepperPID.SetMode(AUTOMATIC);
}

void loop()
{
    stepperPID.Compute();
    // stepperR.setRPM(Output);
    // stepperR.setRPM(Output);
    controller.move(Output, -Output); // steps //PID change direction
    Input = controller.getCount();
    // delay(10000);
    /*
        controller.rotate(90*5, 60*15);
        delay(1000);
        controller.rotate(-90*5, -30*15);
        delay(1000);
        controller.rotate(0, -30*15);
        delay(30000);*/
}

#endif

#if test_stepper_speed
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

int MOTOR_RPM = 300;
int MOTOR_R_RPM = 300;
int MOTOR_L_RPM = 300;

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

void setup()
{
    /*
     * Set target motors RPM.
     */
    stepperR.begin(MOTOR_R_RPM, MICROSTEPS);
    stepperL.begin(MOTOR_L_RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
    // stepperX.setEnableActiveState(LOW);
    // stepperY.setEnableActiveState(LOW);
}

void loop()
{

    stepperL.setRPM(MOTOR_RPM);
    controller.move(100, -100);
    MOTOR_RPM++;
    if (MOTOR_RPM > 500)
        MOTOR_RPM = 0;
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

void setup()
{
    /*
     * Set target motors RPM.
     */
    stepperR.begin(MOTOR_R_RPM, MICROSTEPS);
    stepperL.begin(MOTOR_L_RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
    // stepperX.setEnableActiveState(LOW);
    // stepperY.setEnableActiveState(LOW);
}

void loop()
{

    // controller.move(10,100);
    // delay(10000);

    controller.rotate(90 * 5, 60 * 15);
    delay(2000);
    controller.rotate(-90 * 5, -30 * 15);
    delay(2000);
    controller.rotate(0, -30 * 15);
    delay(3000);
}
#endif

#if test_IMU
#include <Arduino.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 3 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(9600);
    while (!Serial)
        ; // wait for Leonardo enumeration, others continue immediately

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
    mpu.setXGyroOffset(-42); // 220
    mpu.setYGyroOffset(39);  // 76
    mpu.setZGyroOffset(-33); //-85

    mpu.setZAccelOffset(1303); // 1788 // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
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
    }
    else
    {
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

void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
#endif

#if test_website_code
/*Arduino Self Balancing Robot
 * Code by: B.Aswinth Raj
 * Build on top of Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 * Website: circuitdigest.com
 */
#include "MoveDC.h"
#include "I2Cdev.h"
#include <PID_v1.h>                     //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/*********Tune these 4 values for your BOT*********/
double setpoint = 176; // set the value when the bot is perpendicular to ground using serial monitor.
// Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 1;  // Set this first
double Kd = 0.8; // Set this secound
double Ki = 140; // Finally set this
/******End of values setting*********/

double input, output;
PID pid(&input, &speedT, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    /*
    [-2197,-2197] --> [-25,14]
    [-791,-790] --> [-9,4]
    [855,856] --> [16360,16390]

    [82,83] --> [0,3]
    [-34,-33] --> [0,3]
    [30,31] --> [0,4]
    */

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(82);
    mpu.setYGyroOffset(-34);
    mpu.setZGyroOffset(30);
    mpu.setZAccelOffset(855);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        // setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    init_motor_pins();
    /*
   //Initialise the Motor outpu pins
       pinMode (6, OUTPUT);
       pinMode (9, OUTPUT);
       pinMode (10, OUTPUT);
       pinMode (11, OUTPUT);

   //By default turn off both the motors
       analogWrite(6,LOW);
       analogWrite(9,LOW);
       analogWrite(10,LOW);
       analogWrite(11,LOW);
   }
   */
    Robot_stop();
}
void loop()
{

    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        // no mpu data - performing PID calculations and output to motors
        pid.Compute();

        // Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input);
        Serial.print(" =>");
        Serial.println(speedT);

        if (input > 150 && input < 200)
        { // If the Bot is falling

            if (speedT > 0)      // Falling towards front
                moveF();         // Rotate the wheels forward
            else if (speedT < 0) // Falling towards back
                moveB();         // Rotate the wheels backward
        }
        else              // If Bot not falling
            Robot_stop(); // Hold the wheels still
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize){
            fifoCount = mpu.getFIFOCount();
Serial.println("hi");
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);      // get value for q
        mpu.dmpGetGravity(&gravity, &q);           // get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // get value for ypr

        input = ypr[1] * 180 / M_PI + 180;
    }}
}

#endif

#if test_esp_code

#include <Arduino.h>
#include <SoftwareSerial.h>
SoftwareSerial ESPserial(10, 11); // RX | TX

void setup()
{
    Serial.begin(9600); // communication with the host computer
    // while (!Serial)   { ; }

    // Start the software serial for communication with the ESP8266
    ESPserial.begin(115200);

    Serial.println("");
    Serial.println("Remember to to set Both NL & CR in the serial monitor.");
    Serial.println("Ready");
    Serial.println("");
}

void loop()
{
    // listen for communication from the ESP8266 and then write it to the serial monitor
    if (ESPserial.available())
    {
        Serial.print(ESPserial.readStringUntil('/r'));
    }
    delay(5000);
    // listen for user input and send it to the ESP8266
    // if ( Serial.available() )       {  ESPserial.write( Serial.read() );  }
}
#endif
