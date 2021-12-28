//choose what code you want to apply
//Important >>
//Hint:If motors stucked try to release wheel coupler screw
#define finalCode 1
#define test_DC_PID_IMU_esp01 0
#define test_DC_speed_direction 0
#define test_IMU 0 //suceeful
#define test_esp_code 0
//features
#define serial_debug 1
#define enable_esp 1



//right motor pins connected to arduino uno
#define pwm_r 10
#define in_r1 9
#define in_r2 8
//leftt motor pins connected to arduino uno
#define pwm_l 5
#define in_l3 7
#define in_l4 6

#define forward 1
#define backward -1