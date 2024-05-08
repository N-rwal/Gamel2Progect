#include <Arduino.h>
//#include "nvs_flash.h"
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

const int pwm0=32,pwm1=26,an1=36,an2=39,an3=34,an4=35,fdcpin=21,jackpin=4;   //motor and sensor pins
const int dir1=33,dir2=27,bk=25,red=14,yellow=12,green=13;                   //motor settings pins + leds
const int freq=500,chn0=0,chn1=1,res=10,basespeed = 341;                     //Pwm setup

int motdir1=0,motdir2=0,brake,farleft,farright,leftMotSp,rightMotSp;         //variables for reading/writing
float error_prior = 0, KP=0.5, KD=4.5,pid_output=0;                          //variables for processing
int state=0, menu=0;                                                         //machine states

bool fdc=0,jack=1,rstat=0,ystat=0,gstat=0;       //states

void leftmot(int speed,int dirrection);
void rightmot(int speed,int dirrection);
float pidControl(int difference);
int current_value(void);

void setup() {
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
    SerialBT.begin("Zala");
    ledcSetup(chn0,freq,res);
    ledcSetup(chn1,freq,res);

    ledcAttachPin(pwm0,chn0);
    ledcAttachPin(pwm1,chn1);

    pinMode(an1,INPUT);
    pinMode(an2,INPUT);
    pinMode(an3,INPUT);
    pinMode(an4,INPUT);
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
    jack = !digitalRead(jackpin);               // true if jack is missing
    pid_output = pidControl(current_value());   //this reads the sensors and computes the PID signal

    if (SerialBT.available()) {
    char input = SerialBT.read();
    
    switch (input) {
        case '0':
            break;
        case '1':
            break;
        case '2':
            break;
    }
}
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
    int left = analogRead(an1);
    int right = analogRead(an2);
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