#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>

//TaskHandle_t self_balancing_T;
AsyncWebServer server(80);

const char *ssid = "mohamed";
const char *password = "mohamed2023";

const char *PARAM_MESSAGE1 = "message1";
const char *PARAM_MESSAGE2 = "message2";
int angle1,angle2;
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
    servo1.attach(servo1Pin);
    servo2.attach(servo2Pin);
    /*
     WiFi.mode(WIFI_STA);
     WiFi.begin(ssid, password);
    */

    WiFi.mode(WIFI_MODE_AP);
    Serial.print("Setting AP (Access Point)…");
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    
   /* 
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
   */

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "Hello, world"); });

    // Send a GET request to <IP>/get?message=<message>
    server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String message;
        if (request->hasParam(PARAM_MESSAGE1)) {
            message = request->getParam(PARAM_MESSAGE1)->value();
            angle1=message.toInt();
            servo1.write(angle1);
            Serial.print("angle1: ");
            Serial.println(angle1);}

        else if (request->hasParam(PARAM_MESSAGE2)) {
            message = request->getParam(PARAM_MESSAGE2)->value();
            angle2=message.toInt();
            servo2.write(angle2);
            Serial.print("angle2: ");
            Serial.println(angle2);
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, GET: " + message); });


    server.onNotFound(notFound);

    server.begin(); 
   //pinMode(2,OUTPUT);
}

void loop()
{
   /* digitalWrite(2,HIGH);
    delay(9000);
    digitalWrite(2,LOW);
    delay(9000);*/
}

 