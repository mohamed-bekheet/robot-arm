//code with timer for stepper motors
//#define video_code
#define github_code


#ifdef video_code

///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro

int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
int acc_calibration_value = 1000;                            //Enter the accelerometer calibration value

//Various settings
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150;                                //Max target speed (100)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(9600);                                                       //Start the serial port at 9600 kbps
  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode
  
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 

  pinMode(2, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(3, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(4, OUTPUT);                                                       //Configure digital poort 4 as output
  pinMode(5, OUTPUT);                                                       //Configure digital poort 5 as output
  pinMode(13, OUTPUT);                                                      //Configure digital poort 6 as output

  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    if(receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  if(Serial.available()){                                                   //If there is serial data available
    received_byte = Serial.read();                                          //Load the received serial data in the received_byte variable
    receive_counter = 0;                                                    //Reset the receive_counter variable
  }
  if(receive_counter <= 25)receive_counter ++;                              //The received byte will be valid for 25 program loops (100 milliseconds)
  else received_byte = 0x00;                                                //After 100 milliseconds the received byte is deleted
  
  //Load the battery voltage to the battery_voltage variable.
  //85 is the voltage compensation for the diode.
  //Resistor voltage divider => (3.3k + 3.3k)/2.2k = 2.5
  //12.5V equals ~5V @ Analog 0.
  //12.5V equals 1023 analogRead(0).
  //1250 / 1023 = 1.222.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) * 1.222) + 85;
  
  if(battery_voltage < 1050 && battery_voltage > 800){                      //If batteryvoltage is below 10.5V and higher than 8.0V
    digitalWrite(13, HIGH);                                                 //Turn on the led if battery voltage is to low
    low_bat = 1;                                                            //Set the low_bat variable to 1
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer

  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }
  
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read()<<8|Wire.read();                           //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read()<<8|Wire.read();                         //Combine the two bytes to make one integer
  
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if(pid_output < 5 && pid_output > -5)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if(received_byte & B00000001){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if(received_byte & B00000010){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
  }

  if(received_byte & B00000100){                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if(received_byte & B00001000){                                            //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
  }   

  if(!(received_byte & B00001100)){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
  
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while(loop_timer > micros());
  loop_timer += 4000;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
      PORTD &= 0b11110111;                                                  //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTD |= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_left_motor == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us 
  
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_right_motor == 1)PORTD |= 0b00010000;            //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)PORTD &= 0b11101111;            //Set output 4 low because the pulse only has to last for 20us
}

#endif

#ifdef github_code
/*
   Self balancing robot
   By Reko Meriö
   github.com/rekomerio
*/

#include "Arduino.h"
#include <Wire.h>
#include "PID.h"
#include "Channel.h"
#include "defines.h"

#define built_in_led 13
template <class T>

struct axis
{
    T x, y, z;
};

void stopTimer();
void startTimer();
void computeAngle();
void setPulseDuration(float pulseTime);
void blinkLed(uint8_t blinks, uint8_t pin);
void setBraking();
void setYaw();
void gyroCalibrate(struct axis<int32_t> &calibration);
void setInitialAngle();
float pulseTime(float speed);

/* P,   I,   D,   min,  max */
PID anglePID(19.0, 0.5, 30.0, -MAX_SPEED, MAX_SPEED);                         // Controls the motors to achieve desired angle
PID speedPID(0.0165f, 0.0f, 0.00425f, -MAX_ANGLE + 10.0f, MAX_ANGLE - 10.0f); // Adjusts the angle, so that speed is minimal
PID positionPID(0.3f, 0.0f, 9.0f, -400, 400);

struct axis<int16_t> gyro;

struct axis<int16_t> acc;

struct axis<int32_t> gyroOffset =
{
    0, 0, 0
};

Channel pitch;
Channel yaw;

Channel roll;

float angle;
float targetAngle = 0.0f;

bool isFallen = true;
bool shouldHoldPosition = true;
bool hasTargetChanged = false;
bool hasVehicleStopped = false;

/* Used for controlling the yaw */
uint16_t numInterrupts = 0;
uint16_t rightPulseToSkip = 0;
uint16_t leftPulseToSkip = 0;

int32_t actualPosition = 0;
int32_t targetPosition = 0;
uint32_t stickLastUsedAt = 0;

void setup()
{
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0xFFFF;                                            // When timer is equal to this value, interrupt is triggered
    TIMSK1 |= (1 << OCIE1A);                                   // Enable interrupt for OCR1A match
    TCCR1B |= (1 << WGM12);                                    // CTC mode - reset timer on match
    DDRD |= (1 << RMP) | (1 << LMP) | (1 << LMD) | (1 << RMD); // Set pins to output
    /*
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(11, INPUT_PULLUP); // Button 1
    pinMode(12, INPUT_PULLUP); // Button 2
    */
    pinMode(7, INPUT);         // Rx channel yaw
    pinMode(10, INPUT);        // Rx channel pitch

    // Enable interrupts for pin state changes for all digital pins
    PCICR = (1 << PCIE0 | 1 << PCIE2);

    // Enable interrupts only for D10 state changes
    PCMSK0 = 0;
    PCMSK0 = (1 << PCINT2);
    // Enable interrupts only for D7 state changes
    PCMSK2 = 0;
    PCMSK2 = (1 << PCINT23);

#if DEBUG
    Serial.begin(9600);
#endif
Serial.println("hi from start!");
    Wire.begin();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0); // Wake up MPU
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); // Gyro
    Wire.write(0b00010000);//1000 deg/s//set sensivity
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);       // Accelerometer
    Wire.write(0b00001000); // 4G scale
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03); // Set digital low pass filter to ~43Hz
    Wire.endTransmission();

    blinkLed(built_in_led, RED_LED_PIN);
    gyroCalibrate(gyroOffset);
    setInitialAngle();
    blinkLed(7, GREEN_LED_PIN);//not used
}

uint8_t counter = 0; // Used for debugging

void loop()
{
    computeAngle();

    if (abs(angle) > MAX_ANGLE)
    {
        isFallen = true;
        targetAngle = 0.0f;
        anglePID.reset();
        speedPID.reset();
        actualPosition = 0;
        targetPosition = 0;
    }

    if (abs(angle) < START_ANGLE)
    {
        isFallen = false;
    }

    setYaw();

    //int16_t receivedSpeed = pitch.getStickPosition();
    int16_t receivedSpeed = roll.getStickPosition();
    
    if (abs(receivedSpeed) > 25)
    {
        stickLastUsedAt = millis();
        hasTargetChanged = true;
        hasVehicleStopped = false;
    }

    if (isFallen)
    {
        stopTimer();
    }
    else
    {
        float vehicleSpeed = anglePID.compute(angle - targetAngle);

        if (shouldHoldPosition && hasVehicleStopped)
        {
            if (hasTargetChanged)
            {
                stopTimer();
                targetPosition = actualPosition;
                startTimer();
                hasTargetChanged = false;
            }
            receivedSpeed += positionPID.compute(actualPosition - targetPosition);
        }

        targetAngle = speedPID.compute((float)receivedSpeed * 1.5f - vehicleSpeed);

        if (vehicleSpeed > 0)
        {
            PORTD &= ~(1 << LMD); // Normal direction
            PORTD |= (1 << RMD);
        }
        else
        {
            PORTD |= (1 << LMD); // Reversed direction
            PORTD &= ~(1 << RMD);
        }

        if (abs(vehicleSpeed) > MIN_SPEED)
        {
            setPulseDuration(pulseTime(vehicleSpeed)); // Set motor speed
            SET_GREEN_LED_OFF;
        }
        else
        {
            stopTimer(); // Stop the motors
            SET_GREEN_LED_ON;
            if ((uint32_t)(millis() - stickLastUsedAt) > 100)
                hasVehicleStopped = true;
        }
    }
#if DEBUG
   // adjustPid();
#else
    setBraking();
#endif
}











void computeAngle()
{
    static uint32_t previousTime = 0;
    /*
    We wait here a while if necessary, because gyro angle is measured in degrees traveled per second,
    so we need to know exactly how long one loop takes to be able to calculate the current angle.
  */
    constexpr uint16_t loopTime = 1000000 / REFRESH_RATE;
/*delay 1 sec*/
    while ((uint32_t)(micros() - previousTime) < loopTime)
    {
    }

    previousTime = micros();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Address of acc x
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    acc.x = Wire.read() << 8 | Wire.read();
    acc.y = Wire.read() << 8 | Wire.read();
    acc.z = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Address of gyro x
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    gyro.x = (Wire.read() << 8 | Wire.read()) - gyroOffset.x;
    gyro.y = (Wire.read() << 8 | Wire.read()) - gyroOffset.y;
    gyro.z = (Wire.read() << 8 | Wire.read()) - gyroOffset.z;

    constexpr float complementGyro = 1.0f - (1.0f / REFRESH_RATE);
    constexpr float complementAcc = 1.0f / REFRESH_RATE;

    float accelerationY = atan(-1 * (acc.x / ACC_RATE) / sqrt(pow((acc.y / ACC_RATE), 2) + pow((acc.z / ACC_RATE), 2))) * RAD_TO_DEG - CG; // Calculate accelerometer angle from y-axis
    angle += (float)(gyro.y) / GYRO_RATE / REFRESH_RATE;                                                                                   // Calculate the angular rotation gyro has measured from this loop
    angle = complementGyro * angle + complementAcc * accelerationY;
    
    //float accelerationX = atan(-1 * (acc.y / ACC_RATE) / sqrt(pow((acc.x / ACC_RATE), 2) + pow((acc.z / ACC_RATE), 2))) * RAD_TO_DEG - CG; // Calculate accelerometer angle from x-axis
    //angle += (float)(gyro.x) / GYRO_RATE / REFRESH_RATE;                                                                                   // Calculate the angular rotation gyro has measured from this loop
    //angle = complementGyro * angle + complementAcc * accelerationX;    
    Serial.print("AccX");Serial.println(accelerationY);
    Serial.print("Angle");Serial.println(angle);                                                                    // Compensate for gyro drift with complementary filter
}
/*
  This function is used only in the setup to set the initial angle of the robot
*/
void setInitialAngle()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Address of acc.x
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    acc.x = Wire.read() << 8 | Wire.read();
    acc.y = Wire.read() << 8 | Wire.read();
    acc.z = Wire.read() << 8 | Wire.read();
    angle = atan(-1 * (acc.x / ACC_RATE) / sqrt(pow((acc.y / ACC_RATE), 2) + pow((acc.z / ACC_RATE), 2))) * RAD_TO_DEG - CG;
    //angle = atan(-1 * (acc.y / ACC_RATE) / sqrt(pow((acc.x / ACC_RATE), 2) + pow((acc.z / ACC_RATE), 2))) * RAD_TO_DEG - CG;
}
/*
  Read samples from gyro and calculate average
*/
void gyroCalibrate(struct axis<int32_t> &calibration)
{
    for (uint8_t i = 0; i < CALIBRATION_ROUNDS; i++)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 6, true);
        calibration.x += Wire.read() << 8 | Wire.read();
        calibration.y += Wire.read() << 8 | Wire.read();
        calibration.z += Wire.read() << 8 | Wire.read();
        delay(1000 / REFRESH_RATE); // Simulate actual loop time
    }
    calibration.x /= CALIBRATION_ROUNDS;
    calibration.y /= CALIBRATION_ROUNDS;
    calibration.z /= CALIBRATION_ROUNDS;
}

void blinkLed(uint8_t blinks, uint8_t pin)
{
    for (uint8_t i = 0; i < blinks; i++)
    {
        digitalWrite(pin, HIGH);
        delay(50);
        digitalWrite(pin, LOW);
        delay(50);
    }
}

void setYaw()
{
    int16_t receivedYaw = yaw.getStickPosition();

    if (receivedYaw < -25)
    {
        leftPulseToSkip = (1000 / (-receivedYaw));
    }
    else
    {
        leftPulseToSkip = 0;
    }

    if (receivedYaw > 25)
    {
        rightPulseToSkip = (1000 / receivedYaw);
    }
    else
    {
        rightPulseToSkip = 0;
    }
}

void setBraking()
{
    if (LEFT_BTN_IS_PRESSED)
    { // Slow braking
        //anglePID.setPID(19.0, 0.5, 30.0);
        //speedPID.setPID(0.0165, 0.0, 0.00425);
        shouldHoldPosition = false;
        SET_RED_LED_ON;
    }
    else if (RIGHT_BTN_IS_PRESSED)
    { // Fast braking
        //anglePID.setPID(15.0, 0.5, 25.0);
        //speedPID.setPID(0.0265, 0.0, 0.0095);
        shouldHoldPosition = true;
        SET_RED_LED_ON;
    }
    else
    {
        SET_RED_LED_OFF;
    }
}
/*
  For debugging purposes
*/
void adjustPid()
{
#if SPEED_P_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            speedPID.setP(speedPID.getP() - 0.00025);
        if (RIGHT_BTN_IS_PRESSED)
            speedPID.setP(speedPID.getP() + 0.00025);
        Serial.println(speedPID.getP(), 10);
    }
#endif
#if SPEED_I_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            speedPID.setI(speedPID.getI() - 0.00001);
        if (RIGHT_BTN_IS_PRESSED)
            speedPID.setI(speedPID.getI() + 0.00001);
        Serial.println(speedPID.getI(), 10);
    }
#endif

#if SPEED_D_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            speedPID.setD(speedPID.getD() - 0.0005);
        if (RIGHT_BTN_IS_PRESSED)
            speedPID.setD(speedPID.getD() + 0.0005);
        Serial.println(speedPID.getD(), 10);
    }
#endif

#if ANGLE_P_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            anglePID.setP(anglePID.getP() - 1);
        if (RIGHT_BTN_IS_PRESSED)
            anglePID.setP(anglePID.getP() + 1);
        Serial.println(anglePID.getP());
    }
#endif

#if ANGLE_I_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            anglePID.setI(anglePID.getI() - 0.05);
        if (RIGHT_BTN_IS_PRESSED)
            anglePID.setI(anglePID.getI() + 0.05);
        Serial.println(anglePID.getI());
    }
#endif

#if ANGLE_D_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            anglePID.setD(anglePID.getD() - 1);
        if (RIGHT_BTN_IS_PRESSED)
            anglePID.setD(anglePID.getD() + 1);
        Serial.println(anglePID.getD());
    }
#endif
#if POSITION_P_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            positionPID.setP(positionPID.getP() - 0.05);
        if (RIGHT_BTN_IS_PRESSED)
            positionPID.setP(positionPID.getP() + 0.05);
        Serial.println(positionPID.getP(), 10);
    }
#endif
#if POSITION_I_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            positionPID.setI(positionPID.getI() - 0.00001);
        if (RIGHT_BTN_IS_PRESSED)
            positionPID.setI(positionPID.getI() + 0.00001);
        Serial.println(positionPID.getI(), 10);
    }
#endif

#if POSITION_D_ADJ
    if (counter++ % 50 == 0)
    {
        if (LEFT_BTN_IS_PRESSED)
            positionPID.setD(positionPID.getD() - 0.5);
        if (RIGHT_BTN_IS_PRESSED)
            positionPID.setD(positionPID.getD() + 0.5);
        Serial.println(positionPID.getD(), 10);
    }
#endif
}
/*
  Set duration for the pulse generated for stepper motors
*/
void setPulseDuration(float pulseTime)
{
    if (pulseTime < 1)
        return;
    stopTimer();
    OCR1A = MINIMUM_TICKS * pulseTime;
    TCNT1 = (TCNT1 < OCR1A) ? TCNT1 : 0; // Set the counter to 0, if OCR1A is smaller than TCNT1. - Risk of corrupting TCNT1, if timer is not disabled!
    startTimer();
}
/*
  Sets prescaler to 0 to stop the timer
*/
inline void stopTimer()
{
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}
/*
  Sets prescaler to 64 to start the timer
*/
inline void startTimer()
{
    TCCR1B |= (1 << CS11) | (1 << CS10);
}

float pulseTime(float speed) 
{
    // Do this to avoid overflowing 16-bit register when going slow speed
    // 1000 / 65535 = 0.01525...
    // Keep the limit a bit higher because of Atmega328p's inaccurate floating point calculations
    if (speed < 0.016f && speed > -0.016f) 
    {
        return MAX_SPEED / 0.016f;
    }
    return fabs(MAX_SPEED / speed);
}
/*
  Pulse generation for stepper motors.
  Yawing of the robot is done by skipping some pulses
*/
#define RMP_HIGH (PIND >> RMP) & 1
#define LMP_HIGH (PIND >> LMP) & 1
#define RMD_HIGH (PIND >> RMD) & 1

ISR(TIMER1_COMPA_vect)
{
    // Drive right motor
    if (rightPulseToSkip == 0 || numInterrupts % rightPulseToSkip != 0)
    {
        if (RMP_HIGH)
        {
            /* Set pins low */
            //PORTD &= ~(1 << RMP);
            digitalWrite(RMP,LOW);
        }
        else
        {
            /* Set pins high */
            //PORTD |= (1 << RMP);
            digitalWrite(LMP,HIGH);
        }
    }

    // Drive left motor
    if (leftPulseToSkip == 0 || numInterrupts % leftPulseToSkip != 0)
    {
        if (LMP_HIGH)
        {
            /* Set pins low */
            //PORTD &= ~(1 << LMP);
            digitalWrite(LMP,LOW);
        }
        else
        {
            /* Set pins high */
            //PORTD |= (1 << LMP);
            digitalWrite(LMP,HIGH);
        }
    }
    numInterrupts++;

    if (numInterrupts % 2 == 0)
        actualPosition += (RMD_HIGH) ? -1 : 1;
}

// Receiver channel 4, pin D7
#define RX1_IS_HIGH (PIND >> 7) & 1
// Receiver channel 2, pin D10
#define RX2_IS_HIGH (PINB >> 2) & 1

/*
  Capture the pulsewidth of rx channel
*/
ISR(PCINT0_vect)
{
    if (RX2_IS_HIGH)
    {
        pitch.setStartTime(micros());
    }
    else
    {
        pitch.setEndTime(micros());
    }
}

ISR(PCINT2_vect)
{
    if (RX1_IS_HIGH)
    {
        yaw.setStartTime(micros());
    }
    else
    {
        yaw.setEndTime(micros());
    }
}
#endif



#if 0
#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 700

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1
//dir,step
//2,4
//5,6
// All the wires needed for full functionality
#define DIR 2
#define STEP 4
//Uncomment line to use enable/disable functionality
//#define SLEEP 13

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);

void setup() {
    stepper.begin(RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
}

void loop() {
  
    // energize coils - the motor will hold position
    // stepper.enable();
  
    /*
     * Moving motor one full revolution using the degree notation
     */
    stepper.rotate(360);

    /*
     * Moving motor to original position using steps
     */
    //stepper.move(-MOTOR_STEPS*MICROSTEPS);

    // pause and allow the motor to be moved by hand
    // stepper.disable();

    delay(2000);
}

#endif


#if 0

#include <Arduino.h>// defines pins numbers

//dir,step
//2,4
//5,6

const int dirPin = 2;
const int stepPin = 4; 
 
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  //pinMode(2,OUTPUT);
}
void loop() {
/*
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }
  
  delay(1000); // One second delay
  
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(500);
  }
  delay(1000);
  */
}
#endif

#if 0
int led_pin=13;

void setup() {

pinMode(led_pin,OUTPUT);

}

void loop()

{

digitalWrite(led_pin,HIGH);
delay(500);

digitalWrite(led_pin,LOW);

delay(500);

}
#endif