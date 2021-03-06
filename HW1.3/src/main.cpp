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

MPU6050 mpu;

float volatile yaw = 0;

unsigned long timer = 0;
float timeStep = 0.1;

float coef[2] = {1.0, 0.8};


GMotor motor1(DRIVER3WIRE, Mot1, Mot2, Mot3, LOW);
GMotor motor2(DRIVER3WIRE, Mot4, Mot5, Mot6, LOW);

void turn(int degrees){
  int point = yaw + degrees;
  int err = point - yaw;
  int speed = 0;
  

  // motor1.setSpeed((speed + pid)*(err/abs(err))*coef[1]);
  // motor2.setSpeed(-(speed + pid)*(err/abs(err))*coef[0]);

  while (abs(err) > 5.0)
  {
    err = point - yaw;
    speed = constrain(k_p*abs(err), 0, MaxSpeed-MinSpeed);
    motor1.setSpeed((MinSpeed + speed)*(err/abs(err))*coef[1]);
    motor2.setSpeed(-(MinSpeed + speed)*(err/abs(err))*coef[0]);
    // Serial.print(" Yaw = ");
    // Serial.print(yaw);
    // Serial.print(" Error = ");
    // Serial.print(err);
    // Serial.print(" Point = ");
    // Serial.print(point);

    Serial.print(" Speed = ");
    Serial.print(speed);
  
    
    Serial.println();
    Vector norm = mpu.readNormalizeGyro();
    yaw = yaw + norm.ZAxis * (millis()-timer)/1000;
    // delay((timeStep*1000) - (millis() - timer));
    timer = millis();
  }
  
  motor1.setSpeed(0);
  motor2.setSpeed(0);

}

void dash(){
    motor1.setSpeed((MinSpeed + 50)*coef[1]);
    motor2.setSpeed(-(MinSpeed + 50)*coef[0]);
    delay(20);
    motor1.setSpeed(-(MinSpeed + 50)*coef[1]);
    motor2.setSpeed((MinSpeed + 50)*coef[0]);
    delay(20);
    motor1.setSpeed(0);
    motor2.setSpeed(0);
}

void keep(){
    int point = yaw;
    int err = 0;
    int speed = 0;
    while (true)
    {
        err = point - yaw;
        speed = constrain(k_p*abs(err), 0, MaxSpeed-MinSpeed);
        if (abs(err) > 5.0)
        {   
            // dash();
            motor1.setSpeed((MinSpeed + speed)*(err/abs(err))*coef[1]);
            motor2.setSpeed(-(MinSpeed + speed)*(err/abs(err))*coef[0]);
        }
        else{
            motor1.setSpeed(0);
            motor2.setSpeed(0);
        }
        
        Vector norm = mpu.readNormalizeGyro();
        yaw = yaw + norm.ZAxis * (millis()-timer)/1000;
        timer = millis();
    }
    
}

int get_dist(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  int duration = pulseIn(ECHO, HIGH);
  //???????????????? ?????????? 0.034 
  return duration * 0.034 / 2;
}

void keep_dist(int dist){
    int point = dist;
    int err = 0;
    int speed = 0;
    while (true)
    {
        err = get_dist() - point;
        speed = constrain(k_p*abs(err), 0, MaxSpeed-MinSpeed);
        if (abs(err) > 2)
        {   
            // dash();
            motor1.setSpeed((MinSpeed + speed)*(err/abs(err))*coef[1]);
            motor2.setSpeed((MinSpeed + speed)*(err/abs(err))*coef[0]);
        }
        else{
            motor1.setSpeed(0);
            motor2.setSpeed(0);
        }
        delay(50);
    }
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
}

int x = 1;
void loop() {
  keep_dist(20);
}
