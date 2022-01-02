#include <Arduino.h>

#if 1
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

float _kp = 0;
float _ki = 0;
float _kd = 0;

WiFiClient client;
ESP8266WebServer server(80); // Start server on port 80 (default for a web-browser, change to your requirements, e.g. 8080 if your Router uses port 80
                             // To access server from the outsid of a WiFi network e.g. ESP8266WebServer server(8266); and then add a rule on your Router that forwards a
                             // connection request to http://your_network_ip_address:8266 to port 8266 and view your ESP server from anywhere.
                             // Example http://g6ejd.uk.to:8266 will be directed to http://192.168.0.40:8266 or whatever IP address your router gives to this server

String Argument_Name, Clients_Response1, Clients_Response2;

void edit_param()
{
    if (server.args() > 0)
    {
        //Serial.print(server.args());
        // Arguments were received
        for (uint8_t i = 0; i < server.args(); i++)
        {
            //send it for arduino nano and it will handel it
            Serial.print("");
            Serial.print(server.argName(i));// Display the argument
            Serial.print(server.arg(i));
            Serial.print("*");
            

        }
    }
    server.send(200,"text/html","ok");
}
void edit_direction() {

  if (server.args() > 0) {
  //Serial.print(server.args());
  // Arguments were received
  for (uint8_t i=0; i < server.args(); i++) {
    //send it for arduino nano and it will handel it
    Serial.print("");
    Serial.print(server.argName(i)); // send to arduino the argument
    Serial.print(server.arg(i));
    Serial.print("*");
  }
  }

server.send(200, "text/html", "ok");


}

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 8);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 255, 0);

IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

void setup()
{
    Serial.begin(115200);
    // WiFiManager intialisation. Once completed there is no need to repeat the process on the current board
    WiFiManager wifiManager;
    //wifiManager.setSTAStaticIPConfig((192,168,1,8),(192,168,1,1),(255,255,0,0));
  //wifiManager.resetSettings();
    // New OOB ESP8266 has no Wi-Fi credentials so will connect and not need the next command to be uncommented and compiled in, a used one with incorrect credentials will
    // so restart the ESP8266 and connect your PC to the wireless access point called 'ESP8266_AP' or whatever you call it below in ""
    // wifiManager.resetSettings(); // Command to be included if needed, then connect to http://192.168.4.1/ and follow instructions to make the WiFi connection
    // Set a timeout until configuration is turned off, useful to retry or go to sleep in n-seconds
    wifiManager.setTimeout(180);
  /*
    // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }*/
    // fetches ssid and password and tries to connect, if connections succeeds it starts an access point with the name called "ESP8266_AP" and waits in a blocking loop for configuration
    if (!wifiManager.autoConnect("ESP8266_AP"))
    {
        Serial.println("failed to connect and timeout occurred");
        delay(3000);
        ESP.reset(); // reset and try again
        delay(5000);
    }
    
    //WiFi.softAPConfig(local_IP ,gateway,subnet);
    //WiFi.softAP("selfBalance", "123456789");
  
    // At this stage the WiFi manager will have successfully connected to a network, or if not will try again in 180-seconds
    //----------------------------------------------------------------------------------------------------------------------
    Serial.println("WiFi connected..");
    server.begin();
    Serial.println("Webserver started...");           // Start the webserver
    Serial.print("Use this URL to connect: http://"); // Print the IP address
    Serial.print(WiFi.localIP());
    Serial.println("/");
 
    server.on("/editParam",HTTP_GET, edit_param);
    server.on("/editDirection", HTTP_GET, edit_direction);

}

void loop()
{
    server.handleClient();
}
#endif
#if 0
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

/* Put your SSID & Password */
const char* ssid = "selfbalance";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

ESP8266WebServer server(80);

uint8_t LED1pin = 0;
bool LED1status = LOW;

uint8_t LED2pin = 1;
bool LED2status = LOW;



float _kp = 0;
float _ki = 0;
float _kd = 0;


String SendHTML(uint8_t led1stat,uint8_t led2stat){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>LED Control</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #1abc9c;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #1abc9c;}\n";
  ptr +=".button-on:active {background-color: #16a085;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP8266 Web Server</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
  
   if(led1stat)
  {ptr +="<p>LED1 Status: ON</p><a class=\"button button-off\" href=\"/led1off\">OFF</a>\n";}
  else
  {ptr +="<p>LED1 Status: OFF</p><a class=\"button button-on\" href=\"/led1on\">ON</a>\n";}

  if(led2stat)
  {ptr +="<p>LED2 Status: ON</p><a class=\"button button-off\" href=\"/led2off\">OFF</a>\n";}
  else
  {ptr +="<p>LED2 Status: OFF</p><a class=\"button button-on\" href=\"/led2on\">ON</a>\n";}

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}


void handle_OnConnect() {
  LED1status = LOW;
  LED2status = LOW;
  Serial.println("GPIO7 Status: OFF | GPIO6 Status: OFF");
  server.send(200, "text/html", SendHTML(LED1status,LED2status)); 
}

void handle_led1on() {
  LED1status = HIGH;
  Serial.println("GPIO7 Status: ON");
  server.send(200, "text/html", SendHTML(true,LED2status)); 
}

void handle_led1off() {
  LED1status = LOW;
  Serial.println("GPIO7 Status: OFF");
  server.send(200, "text/html", SendHTML(false,LED2status)); 
}

void handle_led2on() {
  LED2status = HIGH;
  Serial.println("GPIO6 Status: ON");
  server.send(200, "text/html", SendHTML(LED1status,true)); 
}

void handle_led2off() {
  LED2status = LOW;
  Serial.println("GPIO6 Status: OFF");
  server.send(200, "text/html", SendHTML(LED1status,false)); 
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}



void setup() {
  Serial.begin(115200);
  pinMode(LED1pin, OUTPUT);
  pinMode(LED2pin, OUTPUT);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  
  server.on("/", handle_OnConnect);
  server.on("/led1on", handle_led1on);
  server.on("/led1off", handle_led1off);
  server.on("/led2on", handle_led2on);
  server.on("/led2off", handle_led2off);
  server.onNotFound(handle_NotFound);
  
  server.begin();
  Serial.println("HTTP server started");
}
void loop() {
  server.handleClient();
  if(LED1status)
  {digitalWrite(LED1pin, HIGH);}
  else
  {digitalWrite(LED1pin, LOW);}
  
  if(LED2status)
  {digitalWrite(LED2pin, HIGH);}
  else
  {digitalWrite(LED2pin, LOW);}
}
#endif

// for local and global server
#if 0 // get and post
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

const char* ssid = "mohamed";
const char* password = "mohamed2024";

//Your Domain name with URL path or IP address with path
String serverName = "http://192.168.1.8/";

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;

void setup() {
  Serial.begin(115200); 

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
}

void loop() {
  // Send an HTTP POST request depending on timerDelay
  if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      WiFiClient client;
      HTTPClient http;

      String serverPath = serverName + "?temperature=24.37";
      
      // Your Domain name with URL path or IP address with path
      http.begin(client, serverPath.c_str());
      
      // Send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}
#endif
#if 0
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
float _kp = 0;
float _ki = 0;
float _kd = 0;

// Replace with your network credentials
const char *ssid = "mohamed";
const char *password = "mohamed2024";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup()
{
    Serial.begin(115200);

    // Connect to Wi-Fi network with SSID and password
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    // Print local IP address and start web server
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
}

void loop()
{
    WiFiClient client = server.available(); // Listen for incoming clients
//Serial.println("wainting");
    if (client)
    {                                  // If a new client connects,
        Serial.println("New Client."); // print a message out in the serial port
        String currentLine = "";       // make a String to hold incoming data from the client
        currentTime = millis();
        previousTime = currentTime;
        while (client.connected() && currentTime - previousTime <= timeoutTime)
        { // loop while the client's connected
            currentTime = millis();
            if (client.available())
            {                           // if there's bytes to read from the client,
                char c = client.read(); // read a byte, then
               // Serial.write(c);        // print it out the serial monitor
                header += c;
                if (c == '\n')
                { // if the byte is a newline character
                    // if the current line is blank, you got two newline characters in a row.
                    // that's the end of the client HTTP request, so send a response:
                    if (currentLine.length() == 0)
                    {
                        // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                        // and a content-type so the client knows what's coming, then a blank line:
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-type:text/html");
                        client.println("Connection: close");
                        client.println();
                        header.
                        // turns the GPIOs on and off
                        if (header.indexOf("GET /p++") >= 0)
                        {
                            Serial.println("GPIO 5 on");
                            _kp = 
                         
                        }
                        else if (header.indexOf("GET /p--") >= 0)
                        {
                            Serial.println("GPIO 5 off");
                            output5State = "off";
                            
                        }
                        else if (header.indexOf("GET /d++") >= 0)
                        {
                            Serial.println("GPIO 4 on");
                            output4State = "on";
                            
                        }
                        else if (header.indexOf("GET /d--") >= 0)
                        {
                            Serial.println("GPIO 4 off");
                            output4State = "off";
                            
                        }
                        Serial.println(header);
                    }
                }
            }
        }
    }
}
#endif

#if 0

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

const char* ssid = "Self-Blance";
const char* password = "";

const char* PARAM_MESSAGEP = "editP";
const char* PARAM_MESSAGEI = "editI";
const char* PARAM_MESSAGED = "editD";

float _kp = 0;
float _ki = 0;
float _kd = 0;

bool updateValues = 0;


void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}
void handleRequest(){
    String message;
        if (request->hasParam(PARAM_MESSAGEP)) {
            message = request->getParam(PARAM_MESSAGEP)->value();
            _kp=message.toFloat();
        }
        else if (request->hasParam(PARAM_MESSAGEP)) {
            message = request->getParam(PARAM_MESSAGEI)->value();
            _ki=message.toFloat();
        }
        else if (request->hasParam(PARAM_MESSAGEP)) {
            message = request->getParam(PARAM_MESSAGED)->value();
            _kd=message.toFloat();
        }
        else if (request->hasParam("++p")) {
            message = request->getParam("++p")->value();
            _kp+=0.1;
        }
         else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, GET: " + message);
}

void setup() {

    Serial.begin(115200);
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);

    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

  /*
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
 */

    // Send a GET request to <IP>/get?message=<message>
    server.on("/get", HTTP_GET, handleRequest);

    server.onNotFound(notFound);

    server.begin();
}

void loop() {
}
#endif

#if 0
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#else
#include <ESP31BWiFi.h>
#endif
#include "ESPAsyncTCP.h"
#include "SyncClient.h"

const char* ssid = "mohamed";
const char* password = "mohamed2024";

void setup(){
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  Serial.printf("WiFi Connected!\n");
  Serial.println(WiFi.localIP());
#ifdef ESP8266
  ArduinoOTA.begin();
#endif
  
  SyncClient client;
  if(!client.connect("www.google.com", 80)){
    Serial.println("Connect Failed");
    return;
  }
  client.setTimeout(2);
  if(client.printf("GET / HTTP/1.1\r\nHost: www.google.com\r\nConnection: close\r\n\r\n") > 0){
    while(client.connected() && client.available() == 0){
      delay(1);
    }
    while(client.available()){
      Serial.write(client.read());
    }
    if(client.connected()){
      client.stop();
    }
  } else {
    client.stop();
    Serial.println("Send Failed");
    while(client.connected()) delay(0);
  }
}

void loop(){
#ifdef ESP8266
  ArduinoOTA.handle();
#endif
}

#endif