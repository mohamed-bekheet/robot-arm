
#define pwm_r 3
#define in_r1 4 
#define in_r2 5

#define pwm_l 10
#define in_l3 9
#define in_l4 8
 

int speedR = 0;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
pinMode(pwm_r,OUTPUT);
pinMode(in_r1,OUTPUT);
pinMode(in_r2,OUTPUT);

pinMode(pwm_l,OUTPUT);
pinMode(in_l3,OUTPUT);
pinMode(in_l4,OUTPUT);


digitalWrite(in_r1,HIGH);
digitalWrite(in_r2,LOW);

digitalWrite(in_l3,HIGH);
digitalWrite(in_l4,LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
analogWrite(pwm_r,speedR);
analogWrite(pwm_l,speedR);
Serial.println(speedR);
delay(1000);
speedR +=10;
if(speedR>255)speedR = 0;

}
