//choose what code you want to apply
//Important >>
//Hint:If motors stucked try to release wheel coupler screw

/*Testing Codes*/
#define finalCode 1
#define test_DC_PID_IMU_esp01 0
#define test_DC_speed_direction 0
#define test_IMU 0 //suceeful
#define test_esp_code 0

/*FEATURES*/
#define serial_debug 0
#define enable_esp 0

/*PINS*/
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