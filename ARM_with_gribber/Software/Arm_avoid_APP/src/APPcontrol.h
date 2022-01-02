/**
 * @file APPcontrol.h
 * @author khaled (you@domain.com)
 * @brief contains RTOS task for controlling ARM and rover with
 * mobile application in station mode or ACcess point mode
 *control dc motors and 2 servo for arm
 */
#include "defines.h"
#ifndef __APPCONTROL_H_
#define __APPCONTROL_H_

AsyncWebServer server(80);

Servo ARMservo1;
Servo ARMservo2;

TaskHandle_t avoidance_T = NULL;
TaskHandle_t ResetServer_T = NULL;

extern int angle1 = 0 ;
extern int angle2 = 0;

extern double currT = 0 ;
extern int prevT = 0;




extern bool mode_state = 0; // 0 app control ,1 autonomous just 2 modes
extern int mode_changed = 0; // 0 app control ,1 autonomous

void AppControl_Enable(void) {  server.begin();}
void AppControl_Disable(void) { server.end(); }

void AppControl_Init(void)
{
    pinMode(mode_pin,INPUT);
    server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      String message;
      if (request->hasParam(PARAM_MESSAGE1)) {
          message = request->getParam(PARAM_MESSAGE1)->value();
          int angle1=message.toInt();
             ARMservo1.write(angle1);

#if serial_debug
           Serial.print("angle1: ");
          Serial.println(angle1);
#endif
          }

      else if (request->hasParam(PARAM_MESSAGE2)) {
          message = request->getParam(PARAM_MESSAGE2)->value();
          int angle2=message.toInt();
             ARMservo2.write(angle2);

#if serial_debug
           Serial.print("angle2: ");
          Serial.println(angle2);
#endif
      }
      else if (request->hasParam(PARAM_VOICE)) {
          message = request->getParam(PARAM_VOICE)->value();

#if serial_debug
          Serial.print("voice: ");
          Serial.println(message);
#endif
        }

      else if (request->hasParam(PARAM_MOVE)){
          message = request->getParam(PARAM_MOVE)->value();
#if serial_debug
          Serial.print("move: ");
          Serial.println(message);
#endif
          
          }
      else if (request->hasParam(PARAM_MODE)) {
          message = request->getParam(PARAM_MODE)->value();
          mode_state=message.toInt();
          mode_changed =1;
#if serial_debug
          Serial.print("mode");
          Serial.println(mode_state);
#endif
          
          //for test
          //if (mode_state)AppControl_Disable();
      }
      else {
          message = "No message sent";
      }
      request->send(200, "text/plain", "Done" + message); });
}


void modeHandler(void)
{
  if ((!digitalRead(mode_pin)) || !mode_state) return;

  if( mode_state = 0) mode_state=1;

#if serial_debug
Serial.print("mode_state = ");
Serial.println(mode_state);
#endif

  //mode_state >> 0 app control ,1 avoidance
  if (mode_state)
  { // avoidance mode on
    vTaskResume(avoidance_T);
    mode_state = 0;
  }
  else{
    vTaskSuspend(avoidance_T);
  }

  //mode_state=!mode_state;

}


#endif