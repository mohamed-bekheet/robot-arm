//this code target is just two motors
//aims to make global timer for two Stepper motors
//Target : generate pulses for moving motor to control (speed,direction)
//
#include <includes.h>

//will use timer 1 16bit in AVR to generate pulses but for arduino nano which clockfreq is 16Mhz non changable
//try to make it with pwm in timer 2 and just adjust the frequency
typedef enum {
 COMPA,//compare match A mode
 COMPB,//compare match B mode
 OVF,//overflow mode
 CTC,
}timer1Modes;//16 bit timer

void set_timer1_mode(timer1Modes mode){
    switch(mode){
        case CTC :
        TIMSK1 |= (1 << OCIE1A);         // Enable interrupt for OCR1A match
        TCCR1B |= (1 << WGM12);          // CTC mode - reset timer on match
    
    }
};
int get_timer2_mode(timer1Modes mode);


void timer1_start(){
    //from Table 15-6.
    TCCR1B |= (1 << CS11) | (1 << CS10);//prescaler is 64
};

//no clock source
void timer1_stop(){
    //from Table 15-6
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}

//write this in your main code if doesnot worked
//timer ticks calculated from prescaler and cpu frequency
ISR(TIMER1_COMPB_vect){

};


#define  RMD 2//left motor step pin
#define  RMS 4//left motor step pin

#define  LMD 5//left motor step pin
#define  LMS 6//left motor step pin
void stepers_moveStep(int no_of_steps){

    timer2_start();

    if()
}


void stepers_move(int speed);
void steppers_forward(int distance,int w_dia);