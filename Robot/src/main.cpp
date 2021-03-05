#include <Arduino.h>
#include <GyverMotor.h>
#include <MPU6050.h>
#include <Wire.h>
#include "GyverTimers.h"

#define Mot1 7
#define Mot2 8
#define Mot3 9

#define Mot4 4
#define Mot5 5
#define Mot6 6

MPU6050 mpu;

float volatile yaw = 0;

unsigned long timer = 0;
float timeStep = 0.1;

float coef[2] = {1.0, 0.9};


//=== MILLISTIMER MACRO v2 ===
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flag = millis() - tmr >= (x);\
  if (flag) tmr += (x);\
  if (flag)
//===========================

GMotor motor1(DRIVER3WIRE, Mot1, Mot2, Mot3, LOW);
GMotor motor2(DRIVER3WIRE, Mot4, Mot5, Mot6, LOW);

void turn(int degrees){
  int point = yaw + degrees;
  int err = point - yaw;

  motor1.setSpeed(150*(err/abs(err))*coef[1]);
  motor2.setSpeed(-150*(err/abs(err))*coef[0]);

  while (abs(err) > 35.0)
  {
    err = point - yaw;
    Serial.print(" Yaw = ");
    Serial.print(yaw);
    Serial.print(" Error = ");
    Serial.print(err);
    Serial.print(" Point = ");
    Serial.print(point);
    
    Serial.println();
    Vector norm = mpu.readNormalizeGyro();
    yaw = yaw + norm.ZAxis * (millis()-timer)/1000;
    // delay((timeStep*1000) - (millis() - timer));
    timer = millis();
  }
  
  motor1.setSpeed(0);
  motor2.setSpeed(0);

}

void setup() {
  Serial.begin(115200);
  motor1.setMode(AUTO);
  motor2.setMode(AUTO);

  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  mpu.calibrateGyro();
  mpu.setThreshold(3);
}

int x = 1;
void loop() {
  turn(-90);
  delay(1000);
  turn(180);
  delay(2000);
  turn(-90);
  delay(100000000);
}
