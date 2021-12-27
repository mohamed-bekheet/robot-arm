#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>
/**
 * @brief 
 * this code control
 * 
 */
/* Setting PWM properties */
const int PWMFreq = 50;
const int PWMChannel = 0;
const int PWMChannel1 = 1;
const int PWMResolution = 8;

int servo1_start_angle = 0;
int servo2_start_angle = 0;

/**
 * @brief func
 * 
 * 
 * @param pin1 
 * @param pin2 
 * @return ** void 
 */
void initServos(int pin1, int pin2)
{
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(pin1, PWMChannel);
  ledcWrite(PWMChannel, servo1_start_angle);

  ledcSetup(PWMChannel1, PWMFreq, PWMResolution);
  ledcAttachPin(pin2, PWMChannel1);
  ledcWrite(PWMChannel1, servo2_start_angle);
}

void moveServoAngle(int degree1, int degree2)
{
  ledcWrite(PWMChannel, degree1);
  ledcWrite(PWMChannel, degree2);
}

// TaskHandle_t self_balancing_T;
AsyncWebServer server(80);

const char *ssid = "esp";
const char *password = "123456789";

const char *PARAM_MESSAGE1 = "message1";
const char *PARAM_MESSAGE2 = "message2";
int angle1, angle2;

void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

// the number of the LED pin
const int servo1Pin = 4;
const int servo2Pin = 5;
Servo servo1;
Servo servo2;

/*void self_balancing( void * parameter ){
  Serial.print("Task2 is running on core ");
  Serial.println(xPortGetCoreID());
    pinMode(32,OUTPUT);
  for(;;){
    digitalWrite(32, HIGH);
    delay(1000);
    digitalWrite(32, LOW);
    delay(1000);
    Serial.println("hi from task balanceing");
  }
}*/

void setup()
{

  // xTaskCreatePinnedToCore(self_balancing,"self_balancing",10000,NULL,1,&self_balancing_T,1);
  Serial.begin(115200);
  /*
  initServos(servo1Pin,servo2Pin);
  moveServoAngle(20,20);
  delay(2000);
  moveServoAngle(50,120);
  delay(2000);
  moveServoAngle(70,20);
  delay(2000);
  moveServoAngle(90,120);
  delay(2000);
  moveServoAngle(110,20);
  delay(2000);
  moveServoAngle(130,120);
  delay(2000);
  moveServoAngle(150,20);
  delay(2000);
  moveServoAngle(255,120);
  delay(2000);
  */
   servo1.attach(servo1Pin);
   servo2.attach(servo2Pin);

  
  /* WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
     
   if (WiFi.waitForConnectResult() != WL_CONNECTED) {
       Serial.printf("WiFi Failed!\n");
       return;
   }

   Serial.print("IP Address: ");
   Serial.println(WiFi.localIP());
  
*/

  WiFi.mode(WIFI_MODE_AP);
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);


  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hello, world"); });

  // Send a GET request to <IP>/get?message=<message>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        String message;
        if (request->hasParam(PARAM_MESSAGE1)) {
            message = request->getParam(PARAM_MESSAGE1)->value();
            angle1=message.toInt();
            //servo1.write(angle1);
            Serial.print("angle1: ");
            Serial.println(angle1);}

        else if (request->hasParam(PARAM_MESSAGE2)) {
            message = request->getParam(PARAM_MESSAGE2)->value();
            angle2=message.toInt();
            //servo2.write(angle2);
            Serial.print("angle2: ");
            Serial.println(angle2);
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", ", GET: " + message);});

  server.onNotFound(notFound);
  server.begin();
}

void loop()
{
}
