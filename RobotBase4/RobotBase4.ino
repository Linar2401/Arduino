#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <GyverMotor.h>

#define Mot1 7
#define Mot2 8
#define Mot3 9

#define Mot4 4
#define Mot5 5
#define Mot6 6


#define RWS 2
#define LWS 3

#define MinSpeed 90
#define MaxSpeed 255

#define k_p 0.001
#define k_d 0.002

#define WHEEL_D 67

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.05;

float yaw = 0;
int path = 0;

int lWheelCounter = 0;
int rWheelCounter = 0;

float coef[2] = {1.0, 0.8};
int minSpeed[2] = {70, 110};

GMotor motor1(DRIVER3WIRE, Mot1, Mot2, Mot3, LOW);
GMotor motor2(DRIVER3WIRE, Mot4, Mot5, Mot6, LOW);

void LWheelCounter(){
  lWheelCounter++;
}

void RWheelCounter(){
  rWheelCounter++;
}

void updatePath(){
  path += ((lWheelCounter/20.0) + (rWheelCounter/20.0))*3.14*WHEEL_D/2.0;
  lWheelCounter = 0;
  rWheelCounter = 0;
}

void turn(int angle){
  int point = yaw + angle;
  int err = point - yaw;
  while(abs(err) > 1.5){

    motor1.setSpeed(minSpeed[0]*(err/abs(err)));
    motor2.setSpeed(-minSpeed[1]*(err/abs(err)));

    Vector norm = mpu.readNormalizeGyro();
    yaw = yaw + norm.ZAxis * (millis()-timer)/1000;
    timer = millis();
  }
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

void move(int v, int l){
    int point = yaw;
    int err = 0;
    int prev_err = 0;
    int speed = 0;
    int path_point = path + l;
    while (path_point - path > 0)
    {   
        updatePath();
        Serial.println(path_point - path);
        err = point - yaw;
        speed = constrain(k_p*abs(err) + (abs(err) - abs(prev_err))*k_d, 0, MaxSpeed-MinSpeed - v);
               
        motor1.setSpeed(minSpeed[0] + v + (speed)*(err/abs(err)));
        motor2.setSpeed(minSpeed[1] + v -(speed)*(err/abs(err)));
        Vector norm = mpu.readNormalizeGyro();
        yaw = yaw + norm.ZAxis * (millis()-timer)/1000;
        timer = millis();
        delay(50);
    }
    motor1.setSpeed(0);
    motor2.setSpeed(0);
}



void setup() 
{
  Serial.begin(115200);
  motor1.setMode(AUTO);
  motor2.setMode(AUTO);
  attachInterrupt(digitalPinToInterrupt(RWS), RWheelCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(LWS), LWheelCounter, RISING);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(100);
  }

  mpu.calibrateGyro();
  mpu.setThreshold(3);
}

void loop()
{
  Serial.println("Start");
  move(50, 500);
  // motor2.setSpeed(110);
  delay(1000000000);
}