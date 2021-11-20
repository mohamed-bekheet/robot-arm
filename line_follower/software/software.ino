#define pwm_r 3
#define in_r1 4
#define in_r2 5

#define pwm_l 10
#define in_l3 9
#define in_l4 8

#define sensorR A2
#define sensorF A1
#define sensorL A0


bool S1, S2, S3;
int speedR = 100;
int speedL = 100;
int speedT = 100;
int speedC = 100;//speed for curve

void moveF() {
  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);

  analogWrite(pwm_r, speedR);
  analogWrite(pwm_l, speedL);
}
void moveB() {
  digitalWrite(in_r1, LOW);
  digitalWrite(in_r2, HIGH);

  digitalWrite(in_l3, LOW);
  digitalWrite(in_l4, HIGH);

  analogWrite(pwm_r, speedR);
  analogWrite(pwm_l, speedL);
}
void moveR() {
  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);
  analogWrite(pwm_r, speedR+speedC);
  analogWrite(pwm_l, speedL-speedC);
}
void moveL() {
  digitalWrite(in_r1, HIGH);
  digitalWrite(in_r2, LOW);

  digitalWrite(in_l3, HIGH);
  digitalWrite(in_l4, LOW);
  analogWrite(pwm_r, speedR-speedC);
  analogWrite(pwm_l, speedL+speedC);
}
void Robot_stop() {
  analogWrite(pwm_r, 0);
  analogWrite(pwm_l, 0);

}

void setup() {

  pinMode(sensorR, INPUT);
  pinMode(sensorF, INPUT);
  pinMode(sensorL, INPUT);
  pinMode(pwm_r, OUTPUT);
  pinMode(in_r1, OUTPUT);
  pinMode(in_r2, OUTPUT);
  pinMode(pwm_l, OUTPUT);
  pinMode(in_l3, OUTPUT);
  pinMode(in_l4, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
  moveF();
  delay(2000);
  moveB();
  delay(2000);
  moveR();
  delay(2000);
  moveL();
  delay(2000);
  Robot_stop();
  delay(2000);
}
