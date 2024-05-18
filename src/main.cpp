#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <Arduino.h>
//#include "nvs_flash.h"

const char *ssid = "Zebra";
const char *password = "ihatehtml";

WebServer server(80);

const int pwm0=32,pwm1=26,an1=36,an2=39,an3=34,an4=35,fdcpin=21,jackpin=4,ballpin=17;   //motor and sensor pins
const int dir1=33,dir2=27,bk=25,red=14,yellow=12,green=13;                              //motor settings pins + leds
const int freq=20000,chn0=0,chn1=1,res=10,basespeed = 341;                              //Pwm setup

int motdir1=0,motdir2=0,brake,farleft,left,right,farright,leftMotSp,rightMotSp;         //variables for reading/writing
float error_prior = 0, KP=0.5, KD=4.5,pid_output=0;                                     //variables for processing
int state=0;                                                                            //machine states

bool fdc=0,jack=1,ball=0,rstat=0,ystat=0,gstat=0;                                       //states

void leftmot(int speed,int dirrection);
void rightmot(int speed,int dirrection);
float pidControl(int difference);
int current_value(void);
void handleRoot();
void handleNotFound();

void setup() {
  Serial.begin(115200);

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");
//---------------------------------------------------FLASH ACCESS-------------------------
/*
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
}
ESP_ERROR_CHECK(ret);
nvs_handle_t nvs_handle;
esp_err_t ret = nvs_open("bay", NVS_READWRITE, &nvs_handle);
if (ret != ESP_OK) {
}
*/
//---------------------------------------------------FLASH ACCESS-------------------------
    ledcSetup(chn0,freq,res);
    ledcSetup(chn1,freq,res);

    ledcAttachPin(pwm0,chn0);
    ledcAttachPin(pwm1,chn1);

    pinMode(an1,INPUT_PULLDOWN);
    pinMode(an2,INPUT_PULLDOWN);
    pinMode(an3,INPUT_PULLDOWN);
    pinMode(an4,INPUT_PULLDOWN);

    pinMode(fdcpin,INPUT_PULLDOWN);
    pinMode(jackpin,INPUT_PULLDOWN);
    pinMode(ballpin,INPUT_PULLDOWN);

    pinMode(dir1,OUTPUT);
    pinMode(dir2,OUTPUT);
    pinMode(bk,OUTPUT);
    pinMode(red,OUTPUT);
    pinMode(yellow,OUTPUT);
    pinMode(green,OUTPUT);

    leftmot(0,motdir1);
    rightmot(0,motdir2);
    digitalWrite(bk,LOW);
}

void loop() {
    fdc = digitalRead(fdcpin);                  //true if fdc is pressed
    jack = !digitalRead(jackpin);               //true if jack is missing
    pid_output = pidControl(current_value());   //this reads the sensors and computes the PID signal
    ball = !digitalRead(ballpin);               //true if ball is present
    server.handleClient();
//---------------------------------------------------MAIN MACHINE STATE-------------------------
    switch (state) { 
            case 0: //Jack present - Stop
                if (jack) {
                    state = 1;
                } else {
                    leftMotSp=0;
                    rightMotSp=0;
                    ystat=1;
                    gstat=0;
                }
                break;
            case 1:                                         //Jack not present, fdc not reached - Continue
                if (!jack) {                                //Detected jack - Stop
                    state = 0;
                }
                if (fdc) {                                  //Fdc detected - Stop
                    state = 3;
                } else {                                    //No road-blocks - Run
                    leftMotSp = basespeed + pid_output;
                    rightMotSp = basespeed - pid_output;
                    gstat=1;
                    ystat=0;
                }
                break;
            case 2:                                         //Take a shortcut or else
                
                break;
            case 3:                                         //Fdc reached
                leftMotSp=0;
                rightMotSp=0;
                ystat=1;
                if (!jack) {                                //Jack detected, return to state 0
                    state = 0;
                }
                break;
            default:
                leftMotSp=0;
                rightMotSp=0;
                break;
        }
//---------------------------------------------------MAIN MACHINE STATE-------------------------
    digitalWrite(red,rstat);
    digitalWrite(yellow,ystat);
    digitalWrite(green,gstat);
    leftmot(leftMotSp,motdir1);
    rightmot(rightMotSp,motdir2);
}

void leftmot(int speed,int dirrection){
    digitalWrite(dir1,dirrection);
    ledcWrite(chn0,speed);
}
void rightmot(int speed,int dirrection){
    digitalWrite(dir2,dirrection);
    ledcWrite(chn1,speed);
}

int current_value(void){//This updates the sensor variables
    left = analogRead(an1);
    right = analogRead(an2);
    farleft = analogRead(an3);
    farright = analogRead(an4);
    //anRead1 - anRead2 maybe ?
    int out = left - right;
    return out;
}
float pidControl(int difference) {// PID controller function
    float error = 0 - difference;
    float derivative = error - error_prior;
    //integral = integral + error;
    float output = (KP * error) + (KD * derivative); //(KI * integral) +
    error_prior = error;
    return output;
}
void handleRoot() {
  Serial.println("Handling root request");
  File file = SPIFFS.open("/index.html", "r");
  if (!file) {
    Serial.println("Failed to open /index.html");
    server.send(500, "text/plain", "File not found");
    return;
  }

  Serial.println("Serving /index.html");
  server.streamFile(file, "text/html");
  file.close();
}
void handleNotFound() {//For debugging purposses
  Serial.println("Not Found: " + server.uri());
  server.send(404, "text/plain", "Not found");
}
/*
//FOR WRITING:
ret = nvs_set_i32(nvs_handle, "key", REVALUE);
if (ret != ESP_OK) {
    printf("Error writing value\n");
}

ret = nvs_commit(nvs_handle);
if (ret != ESP_OK) {
    printf("Error committing changes\n");
}

FOR READING:
int32_t value;
ret = nvs_get_i32(nvs_handle, "key", &value);
if (ret == ESP_OK) {
    printf("Value: %d\n", value);
} else if (ret == ESP_ERR_NVS_NOT_FOUND) {
    printf("Value not found\n");
} else {
    printf("Error reading value\n");
}
*/