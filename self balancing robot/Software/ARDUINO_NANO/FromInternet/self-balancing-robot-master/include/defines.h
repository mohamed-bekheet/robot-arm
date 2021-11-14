#define DEBUG 1

#define ANGLE_P_ADJ 0
#define ANGLE_I_ADJ 0
#define ANGLE_D_ADJ 0

#define SPEED_P_ADJ 0
#define SPEED_I_ADJ 0
#define SPEED_D_ADJ 0

#define POSITION_P_ADJ 0
#define POSITION_I_ADJ 0
#define POSITION_D_ADJ 0

/* GYRO */
#define MPU_ADDR 0x68
#define REFRESH_RATE 250.0f
#define ACC_RATE 8192.0f
#define GYRO_RATE 32.8f
#define CALIBRATION_ROUNDS 50
/*
   Used for pulse generation
   1/1 step: 62
   1/2 step: 31
   1/4 step: 16
*/
#define MINIMUM_TICKS 62//16

/* ROBOT */
#define CG 0
#define MAX_SPEED 700.0f
#define MIN_SPEED 1
#define MAX_ANGLE 110//40

#define START_ANGLE 1
/*
  Pins dedicated for the motors
  L: LEFT
  R: RIGHT
  M: MOTOR
  P: PULSE
  D: DIRECTION
*/
//dir,step
//5,6
//2,4
#define LMD 2 //direction
#define LMP 4 //step

#define RMD 5
#define RMP 6


#define GREEN_LED_PIN 8
#define RED_LED_PIN 13

#define SET_GREEN_LED_ON PORTB |= (1 << 0)
#define SET_GREEN_LED_OFF PORTB &= ~(1 << 0)
#define SET_RED_LED_ON PORTB |= (1 << 1)
#define SET_RED_LED_OFF PORTB &= ~(1 << 1)

#define LEFT_BTN_IS_PRESSED !((PINB >> 3) & 1)
#define RIGHT_BTN_IS_PRESSED !((PINB >> 4) & 1)
