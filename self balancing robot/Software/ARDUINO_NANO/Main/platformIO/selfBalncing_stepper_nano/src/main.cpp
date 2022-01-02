
// choose what code you want to run from defines.h file write 1 for desied and 0 for others
#include "defines.h"

#if finalCode

/*Arduino Self Balancing Robot
 * Code by: B.Aswinth Raj
 * Build on top of Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 * Website: circuitdigest.com
 */
#include "MoveDC.h"
#include "I2Cdev.h"
#include <PID_v1.h>                     //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#if enable_esp
#include <SoftwareSerial.h>
SoftwareSerial espSerial(12, 11); // RX, TX
String esp_message, sub_mess;
#endif

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
double Kp = 21;  // 20;//20 ;//21;  // Set this first
double Kd = 0.8; // 0.6;//0.5 ;//0.8; // Set this secound
double Ki = 90;  // 70;//0; //140; // Finally set this
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
    #if serial_debug
    Serial.begin(9600);
    #endif
    #if enable_esp
    espSerial.begin(115200);
    #endif

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
    Robot_stop();
}
void loop()
{
    #if enable_esp
    if (espSerial.available())
    {
        // meaasge example
        //" kp15."
        
        esp_message = espSerial.readStringUntil('*');
        sub_mess = esp_message.substring(2, esp_message.indexOf("*"));
        sub_mess.toFloat();

        if (esp_message.startsWith("S"))
        { // stop moving
            setpoint = 180;
            speedControlR = 0;
            speedControlL = 0;
        }
        else if (esp_message.startsWith("F"))
        { // move forward
            setpoint = 182;
            speedControlR = 0;
            speedControlL = 0;
        }
        else if (esp_message.startsWith("B"))
        { // move backward
            setpoint = 178;
            speedControlR = 0;
            speedControlL = 0;
        }
        else if (esp_message.startsWith("R"))
        { // rotate right
            setpoint = 180;
            speedControlR -= 10;
            speedControlL += 10;
        }
        else if (esp_message.startsWith("L"))
        {
            setpoint = 180;
            speedControlR += 10;
            speedControlL -= 10;
        }
    }
    #endif

    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        input = ypr[1] * 180 / M_PI + 180;
    }

    // no mpu data - performing PID calculations and output to motors
    pid.Compute();

    if (input > 130 && input < 230)
     // If the Bot is falling
        movePID(speedT, speedControlR, speedControlL);
    
    else              // If Bot not falling
        Robot_stop(); // Hold the wheels still
        #if serial_debug
    // Print the value of Input and Output on serial monitor to check how it is working.
    Serial.print("esp meassage");
    Serial.println(esp_message);
    Serial.print(input);
    Serial.print(" =>");
    Serial.print(setpoint);
    Serial.print(" =>");
    Serial.print(speedControlR);
    Serial.print(" =>");
    Serial.print(speedControlL);
    Serial.print(" =>");
    Serial.println(speedT);
#endif

}
#endif

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

#if test_DC_speed_direction
#include "Arduino.h"
#include "MoveDC.h"
void setup()
{
    init_motor_pins();
    speedT = 200;
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
