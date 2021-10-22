
void IRAM_ATTR test(){
  Serial.println("yes");
  }

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial.println("hi");
pinMode(18,INPUT_PULLDOWN);
attachInterrupt(18,test,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

}
