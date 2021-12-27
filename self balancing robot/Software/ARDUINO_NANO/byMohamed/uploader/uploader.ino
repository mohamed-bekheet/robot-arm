 
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
double setpoint = 180; // set the value when the bot is perpendicular to ground using serial monitor.
// Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 21;//20;//20 ;//21;  // Set this first
double Kd = 0.8;//0.6;//0.5 ;//0.8; // Set this secound
double Ki = 90;//70;//0; //140; // Finally set this
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
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(2, INPUT);

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
       // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
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
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180 / M_PI + 180;
        // blink LED to indicate activity
    }

        // no mpu data - performing PID calculations and output to motors
        pid.Compute();

        // Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input);
        Serial.print(" =>");
        Serial.println(speedT);
        Serial.println(mpuInterrupt);

        if (input > 140 && input < 230)
        { // If the Bot is falling
          
            if (speedT > 0)      // Falling towards front
                moveF();         // Rotate the wheels forward
            else if (speedT < 0) // Falling towards back
                moveB();         // Rotate the wheels backward
        }
        else              // If Bot not falling
            Robot_stop(); // Hold the wheels still
        


        
/*
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        // no mpu data - performing PID calculations and output to motors
        pid.Compute();

        // Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input);
        Serial.print(" =>");
        Serial.println(speedT);
        Serial.println(mpuInterrupt);

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
    Serial.println("hi1");
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
        while (fifoCount < packetSize) {fifoCount = mpu.getFIFOCount();Serial.println("hi");}

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);      // get value for q
        mpu.dmpGetGravity(&gravity, &q);           // get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // get value for ypr

        input = ypr[1] * 180 / M_PI + 180;
    }
    */
}
