
#define sensorR A2
#define sensorF A1
#define sensorL A0

int speedR = 0;
void setup() {
  Serial.begin(9600);

pinMode(sensorR,INPUT);
pinMode(sensorF,INPUT);
pinMode(sensorL,INPUT);

}

void loop() {


Serial.print("S_R: ");
Serial.println(digitalRead(sensorR));
Serial.print("S_F: ");
Serial.println(digitalRead(sensorF));
Serial.print("S_L: ");
Serial.println(digitalRead(sensorL));
Serial.println("");

delay(500);

}
