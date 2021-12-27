//choose what code you want to apply
//Important >>
//Hint:If motors stucked try to release wheel coupler screw
#define test_esp_arduino_serial 0
#define test_DC_PID_IMU_esp01 0
#define test_DC_speed_direction 0
#define test_stepper_PID_timers 0
#define test_stepper_speed_timers 0//variable speed with timer
#define test_stepper_PID 0//PID
#define test_stepper_speed 0//variable speed with library
#define test_stepper 0 //constant speed
#define test_IMU 1 //suceeful

#define test_website_code 0//code from internet
#define test_esp_code 0

#define enable_esp 0
//right motor pins connected to arduino uno
#define pwm_r 5
#define in_r1 3
#define in_r2 4
//leftt motor pins connected to arduino uno
#define pwm_l 6
#define in_l3 7
#define in_l4 8

#define forward 1
#define backward -1