#include "defines.h"
#include "avoidance.h"

int mode_state;
int angle1, angle2;

// the number of the LED pin
const int servo1Pin = 4;
const int servo2Pin = 5;

TaskHandle_t control_T = NULL;
TaskHandle_t automance_T = NULL;



AsyncWebServer server(80);

const char *ssid = "mazen";
const char *password = "123456789";

const char *PARAM_MESSAGE1 = "message1";
const char *PARAM_MESSAGE2 = "message2";
const char *PARAM_MODE = "mode";
const char *PARAM_VOICE = "voice";
const char *PARAM_MOVE = "move";

Servo servo1;
Servo servo2;

void control(void *parameter)
{
    Serial.print("in control mode now");
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
      }
      else if (request->hasParam(PARAM_VOICE)) {
          message = request->getParam(PARAM_VOICE)->value();
          Serial.print("voice: ");
          Serial.println(message);
          /*if (message.startsWith("forward")){
              forward();
          }
          else if(message.startsWith("backward")){
              backward();
          }
          else if(message.startsWith("right")){
              rot_r();
              delay(300);
              forward();
          }
          else if(message.startsWith("left")){
              rot_l();
              delay(300);
              forward();
          }
          else if(message.startsWith("stop")){
              move_stop();
          }    
      */}

      else if (request->hasParam(PARAM_MOVE)){
          message = request->getParam(PARAM_MOVE)->value();
          Serial.print("move: ");
          Serial.println(message);
          /*if (message=="forward"){
              forward();
          }
          else if(message=="backward"){
              backward();
          }
          else if(message=="right"){
              rot_r();
              delay(300);
          }
          else if(message=="left"){
              rot_l();
              delay(300);
          }
          else if(message=="stop"){
              move_stop();  
          }  
          */}


      else if (request->hasParam(PARAM_MODE)) {
          message = request->getParam(PARAM_MODE)->value();
          mode_state=message.toInt();
          Serial.print("mode");
          Serial.println(mode_state);
      }
      else {
          message = "No message sent";
      }
      request->send(200, "text/plain", ", GET: " + message); });
    server.begin();
    for (;;)
    {
        vTaskDelay(10000);
    }
}

// long int delay_t;
void setup()
{


    // delay_t=millis();

    // xTaskCreatePinnedToCore(automance,"automance",10000,NULL,1,&automance_T,0);
    // xTaskCreatePinnedToCore(control, "control", 30000, NULL,1, &control_T, 0);

    Serial.begin(115200);
    Serial.println("start");
    // server.begin();
    /*
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  ledcSetup(servo_channel, servo_freq, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(servo_pin, servo_channel);   // GPIO 14 assigned to channel

    pinMode(motor11, OUTPUT);
    pinMode(motor12, OUTPUT);
    pinMode(motor21, OUTPUT);
    pinMode(motor22, OUTPUT);

    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

  ledcSetup(PWM_IN1,PWM_FREQ,PWM_RES);
  ledcSetup(PWM_IN2,PWM_FREQ,PWM_RES);
  ledcSetup(PWM_IN3,PWM_FREQ,PWM_RES);
  ledcSetup(PWM_IN4,PWM_FREQ,PWM_RES);

  ledcAttachPin(motor11,PWM_IN1);
  ledcAttachPin(motor12,PWM_IN2);
  ledcAttachPin(motor21,PWM_IN3);
  ledcAttachPin(motor22,PWM_IN4);

*/

    /*
  WiFi.mode(WIFI_MODE_AP);
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);*/
}

void loop()
{
    /*if(mode_state==1 and button==0){ //automance mode on
     vTaskSuspend(control_T);
     vTaskResume(automance_T);
     mode_state=0;
  }
  if(button==1 and flag){

    vTaskSuspend(automance_T);
    vTaskResume(control_T);
    mode_state=0;
    flag=0;
    }
  else if(button==0){
    flag=1;
  }*/
}