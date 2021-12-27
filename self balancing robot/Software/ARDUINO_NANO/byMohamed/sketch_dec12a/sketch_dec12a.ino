
 
#include <SoftwareSerial.h>
SoftwareSerial espSerial(10, 11); // RX | TX
 
void setup() 
{
    Serial.begin(9600);     // communication with the host computer
    //while (!Serial)   { ; }
 
    // Start the software serial for communication with the ESP8266
    espSerial.begin(115200);  
 
    Serial.println("");
    Serial.println("Remember to to set Both NL & CR in the serial monitor.");
    Serial.println("Ready");
    Serial.println("");    
}
 
void loop() 
{
 if (espSerial.available())
    {
        //meaasge example 
        //" kp15."
        String esp_message,sub_mess;
        esp_message = espSerial.readStringUntil('*');
        sub_mess = esp_message.substring(2,esp_message.indexOf("*"));
        sub_mess.toFloat();
        
        if(esp_message.startsWith("kp")){
            Serial.print("kp value:");
            Serial.println(sub_mess);
        }
        if(esp_message.startsWith("kd")){
            Serial.print("kd value:");
            Serial.println(sub_mess);
        }
        if(esp_message.startsWith("ki")){
            Serial.print("ki value:");
            Serial.println(sub_mess);
        }
        
        
        
    }}
