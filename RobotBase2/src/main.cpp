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

#define MinSpeed 100
#define MaxSpeed 255

#define k_p 0.3

MPU6050 mpu;

float volatile yaw = 0;

unsigned long timer = 0;
float timeStep = 0.1;

float coef[2] = {1.0, 0.8};


GMotor motor1(DRIVER3WIRE, Mot1, Mot2, Mot3, LOW);
GMotor motor2(DRIVER3WIRE, Mot4, Mot5, Mot6, LOW);

void turn(int degrees){
    int point = degrees;
    int err = point - yaw;
    int speed = 0;
    int counter = 0;
    while (abs(err) > 3)
    {   
        counter++;
        err = point - yaw;
        speed = constrain(k_p*abs(err), 0, MaxSpeed-MinSpeed);
        motor1.setSpeed((MinSpeed + speed)*(err/abs(err))*coef[1]);
        motor2.setSpeed(-(MinSpeed + speed)*(err/abs(err))*coef[0]);
        
        Vector norm = mpu.readNormalizeGyro();
        yaw = yaw + norm.ZAxis * (millis()-timer)/1000;
        timer = millis();

        if(counter == 100){
          Serial.println(yaw);
          counter = 0; 
        }
    }
    
}


void keep(){
    int point = yaw;
    int err = 0;
    int speed = 0;
    while (true)
    {
        Serial.println("Keeping");
        err = point - yaw;
        speed = constrain(k_p*abs(err), 0, MaxSpeed-MinSpeed);
        if (abs(err) > 5.0)
        {  
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
  Serial.println("Start");
  // keep();
  turn(90);
  delay(1000);
  turn(-90);
  delay(100000000);
}
