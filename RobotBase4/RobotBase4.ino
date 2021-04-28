#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <GyverMotor.h>
#include <Servo.h>

#define ServoPin1 7
#define ServoPin2 8

#define Mot1 A2
#define Mot2 A1
#define Mot3 A0

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

Servo servo1;
Servo servo2;

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
    err = point - yaw;
    Serial.println(err);
    motor1.setSpeed((minSpeed[0] + abs(err) + 5)*(err/abs(err)));
    motor2.setSpeed(-(minSpeed[1] + abs(err) + 5)*(err/abs(err)));
    Vector norm = mpu.readNormalizeGyro();
    yaw = yaw + norm.ZAxis * (millis()-timer)/1000;
    timer = millis();
    delay(35);
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    delay(35);
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
               
        if(v > 0){
          motor1.setSpeed(minSpeed[0] + v + (speed)*(err/abs(err)));
          motor2.setSpeed(minSpeed[1] + v -(speed)*(err/abs(err)));
        }
        else{
          motor1.setSpeed(-(minSpeed[0] + v + (speed)*(err/abs(err))));
          motor2.setSpeed(-(minSpeed[1] + v -(speed)*(err/abs(err))));
        }
        Vector norm = mpu.readNormalizeGyro();
        yaw = yaw + norm.ZAxis * (millis()-timer)/1000;
        timer = millis();
        delay(50);
    }
    motor1.setSpeed(0);
    motor2.setSpeed(0);
}


void grab(bool s){
  if(s){
    servo1.write(90);
    servo2.write(0);
  }
  else{
    servo1.write(0);
    servo2.write(90);
  }
  
}

int getDist(){
  return 20;
}


void commandGet(){
  int dist[5];
  turn(-20);
  for(int i = 0; i < 5; i++){
    dist[i] = getDist();
    if(i < 4){
      turn(10);
    }
  }
  turn(-20);
  int min_ang = 0;
  for(int i = 1; i < 5; i++){
    if(dist[min_ang] > dist[i]){
      min_ang = i;
    }
  }
  turn(min_ang*10-20);
  
  move(20, dist[min_ang]*10 - 50);
  grab(true);
  move(-20, dist[min_ang]*10 - 50);
  turn(-(min_ang*10-20));
}

void commandRelease(){
  move(20, 100);
  grab(false);
  move(-20,100);
}

void commandGo2Position(int positionNumber){
  move(-40, 200*(positionNumber/2));
  turn(90*(positionNumber%2*2-1));
}

void commandGoFromPosition(int positionNumber){
  turn(-90*(positionNumber%2*2-1));
  move(40, 200*(positionNumber/2));
}

void commandGetPackage(int positionNumber){
  commandGo2Position(positionNumber);
  commandGet();
  commandGoFromPosition(positionNumber);
  commandRelease();
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

  servo1.attach(ServoPin1);
  servo2.attach(ServoPin2);
}

int value = -1;
int state = -1;
int data = -1;

void loop()
{
//  Serial.println("Start");
   if (Serial.available() > 0)
  {
    data = Serial.parseInt();
    if (data != 0)
    {
      state = data/1000;
      value = data % 1000;
      Serial.println(data);
    }
  }
  switch (state)
  {
  case 1:
    commandGetPackage(value);       
    state = 0;
    break;
  default:
    state = 0;
    break;
  }
//  delay(1000000000);
}
