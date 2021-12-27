#include "defines.h"

int servo1_start_angle = 0;
int servo2_start_angle = 0;

// TaskHandle_t self_balancing_T;
 TaskHandle_t control_T;
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


Servo servo1;
Servo servo2;

// the number of the LED pin
const int servo1Pin = 4;
const int servo2Pin = 5;

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

void control( void * parameter ){
  Serial.print("Task2 is running on core ");
  Serial.println(xPortGetCoreID());
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
 for(;;){
    vTaskDelay(10000);
  }
  }

void setup()
{

  // xTaskCreatePinnedToCore(self_balancing,"self_balancing",10000,NULL,1,&self_balancing_T,1);
  xTaskCreatePinnedToCore(control,"control",10000,NULL,1,&control_T,1);

  Serial.begin(115200);

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  
  /*
  WiFi.mode(WIFI_STA);
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
  
// Send a GET request to <IP>/mode?message=<message>
/*  server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        String mode;
        if (request->hasParam("changeMode")) {
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
        request->send(200, "text/plain", ", GET: " + message);});*/


  // Send a GET request to <IP>/get?message=<message>
  /*server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
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
        request->send(200, "text/plain", ", GET: " + message);});*/
}

void loop()
{
}
