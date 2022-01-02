#include <Arduino.h>
#include "APPcontrol.h"
#include "avoidance.h"

// TaskHandle_t control_T = NULL;

void setup()
{

#if serial_debug
  Serial.begin(115200);
  Serial.println("start");
#endif

#if AP_mode
  WiFi.mode(WIFI_MODE_AP);
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(Ssid, Password);

  IPAddress IP = WiFi.softAPIP();

#if serial_debug
  Serial.print("AP IP address: ");
  Serial.println(IP);
#endif

#endif

#if station_mode
  WiFi.mode(WIFI_STA);
  WiFi.begin(Ssid, Password);

  while (WiFi.status() != WL_CONNECTED){Serial.printf("WiFi Failed!\n");delay(1000);}
    

#if serial_debug
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
#endif

#endif

  avoidanceInit();

  AppControl_Init();

  // xTaskCreatePinnedToCore(controlTask, "control", 10000, NULL,1, &control_T, 0);
  xTaskCreatePinnedToCore(avoidanceTask, "avoidance", 10000, NULL, 1, &avoidance_T, 1);
  vTaskSuspend(avoidance_T);

  mode_state = 0;
  AppControl_Enable();

}

void loop()
{
  // modeHandler();
  //|| !mode_state
  //if ((!digitalRead(mode_pin))) return;

if ((!mode_changed)) return;

 mode_changed=0;

#if serial_debug
Serial.print("mode_state = ");
Serial.println(mode_state);
#endif

  //mode_state >> 0 app control ,1 avoidance
  if (mode_state)
  { // avoidance mode on
    vTaskResume(avoidance_T);
        #if serial_debug
    Serial.println("avoidanceTask Resumeed ");
  
#endif
  }
  else{
    vTaskSuspend(avoidance_T);
    #if serial_debug
    Serial.println("avoidanceTask suspended ");
  
#endif
  }

  //mode_state=!mode_state;

}
