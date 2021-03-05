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

#define TRIG 10
#define ECHO 11

#define k_p 2

#define RWS 2
#define LWS 3

#define Min_R 67
#define Min_L 82

MPU6050 mpu;

float volatile yaw = 0;

unsigned long timer = 0;
float timeStep = 0.1;

float coef[2] = {1.0, 0.8};

int lWheelCounter = 0;
int rWheelCounter = 0;

int rpm_r = 0;
int rpm_l = 0;


GMotor motor1(DRIVER3WIRE, Mot1, Mot2, Mot3, LOW);
GMotor motor2(DRIVER3WIRE, Mot4, Mot5, Mot6, LOW);

void LWheelCounter(){
  lWheelCounter++;
}

void RWheelCounter(){
  rWheelCounter++;
}

void setup() {
  Serial.begin(115200);
  motor1.setMode(AUTO);
  motor2.setMode(AUTO);

  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  mpu.calibrateGyro();
  mpu.setThreshold(3);

  pinMode(TRIG, OUTPUT); 
  pinMode(ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(RWS), RWheelCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(LWS), LWheelCounter, FALLING);
}

void updateRPM(){
  rpm_r = 60*4*rWheelCounter/20;
  rWheelCounter = 0;
  rpm_l = 60*4*lWheelCounter/20;
  lWheelCounter = 0;
  Serial.print("Right = ");
  Serial.print(rpm_r);
  Serial.print(" Left = ");
  Serial.print(rpm_l);
  Serial.println();
}

int x = 1;
void loop() {
  motor1.setSpeed(Min_R+50*(255-Min_R));
  motor2.setSpeed(Min_L+50*(255-Min_L));
  updateRPM();
  delay(250);
 
  
}
