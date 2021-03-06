#include <Arduino.h>
#include <GyverMotor.h>
#include <MPU6050.h>
#include <Wire.h>

#define Mot1 7
#define Mot2 8
#define Mot3 9

#define Mot4 4
#define Mot5 5
#define Mot6 6

#define MinSpeed 90
#define MaxSpeed 255

#define k_p 0.3

#define RWS 2
#define LWS 3

#define Min_R 67
#define Min_L 82

#define MaxRPS_L 5.7
#define MaxRPS_R 6.5

MPU6050 mpu;

float volatile yaw = 0;

unsigned long timer = 0;
unsigned long r_timer = 0;
unsigned long p_timer = 0;
float timeStep = 0.1;

float path = 0.0;
float angle = 0.0;

float coef[2] = {1.0, 0.8};

float r = 6.7;
int L = 12;


GMotor motor1(DRIVER3WIRE, Mot1, Mot2, Mot3, LOW);
GMotor motor2(DRIVER3WIRE, Mot4, Mot5, Mot6, LOW);

int lWheelCounter = 0;
int rWheelCounter = 0;

float rps_r = 0;
float rps_l = 0;

float w_r = 0;
float w_l = 0;


void LWheelCounter(){
  lWheelCounter++;
}

void RWheelCounter(){
  rWheelCounter++;
}

void updatePath(){
  w_l = PI*rps_l;
  w_r = PI*rps_r;

  path = path + (r/2)*(w_l+w_r)*(millis()-p_timer)/1000.0;
  angle = angle + (r/L)*(w_r-w_l)*(millis()-p_timer)/1000.0;
  p_timer = millis();
}

void update(){
  rps_r = (rWheelCounter/20.0)/((millis()-r_timer)/1000.0);
  rps_l = (lWheelCounter/20.0)/((millis()-r_timer)/1000.0);
  r_timer = millis();
  lWheelCounter = 0;
  rWheelCounter = 0;
  updatePath();
}

void setup() {
  Serial.begin(115200);
  motor1.setMode(AUTO);
  motor2.setMode(AUTO);

  attachInterrupt(digitalPinToInterrupt(RWS), RWheelCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(LWS), LWheelCounter, FALLING);

  path = 0.0;
  angle = 0.0;
}

double l_max = 0;
double r_max = 0;

void loop() {
  // timer = millis();
  // Serial.println("Start");
  motor1.setSpeed(100);
  motor2.setSpeed(100);

  Serial.print(path);
  Serial.print(',');
  Serial.print(angle);
  Serial.print(',');
  
  update();
  delay(50);
}
