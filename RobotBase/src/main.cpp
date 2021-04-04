#include <Arduino.h>
#include <GyverMotor.h>
#include <MPU6050.h>
#include <Wire.h>
#include <math.h>

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

#define M_V 800.0
#define M_W 502.0


MPU6050 mpu;

float volatile yaw = 0;

unsigned long timer = 0;
unsigned long r_timer = 0;
unsigned long p_timer = 0;
unsigned long k_timer = 0;

float path = 0.0;
float angle = 0.0;

float v = 0;
float w = 0;

float coef[2] = {1.0, 0.8};

float r = 67;
int L = 120;


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

int PID(float input, float setpoint, float kp, float ki, float kd, float dt) {
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = integral + (float)err * dt * ki;
  float D = (err - prevErr) / dt;
  prevErr = err;
  return err * kp + integral + D * kd;
}

int P(float input, float setpoint, float kp){
  float err = setpoint - input;
  return err * kp;
}

int m1;
int m2;

void keepSpeed(int linear, int angular, int forward){
  m1 = constrain(80 + P((v/M_V)*100, linear, 0.7)  - PID((w/(M_W))*100, angular, 1,0.5,0,(millis() - k_timer)/1000.0), 80, 255);
  m2 = constrain(100 + P((v/M_V)*100, linear, 0.7) + PID((w/(M_W))*100, angular, 1,0.5,0,(millis() - k_timer)/1000.0),80, 255);
  k_timer = millis();
  
  // m1 = constrain(80 + P((v/M_V)*100, linear, 0.7), 80, 255);
  // m2 = constrain(100 + P((v/M_V)*100, linear, 0.7),80, 255);
  motor1.setSpeed(m1*forward);
  motor2.setSpeed(m2*forward);

  // Serial.print((-(w/(M_W))*100+angular));
  // Serial.print(',');
  // Serial.print(80 + P((v/M_V)*100, linear, 0.7));
  // Serial.print(',');
  // Serial.print(P((w/(M_W))*100, angular, 10));
  // Serial.print(',');
  Serial.print(w);
  Serial.println();
  
}

void updatePath(){
  w_l = PI*rps_l;
  w_r = PI*rps_r;

  v = (r/2)*(w_l+w_r);
  w = (180.0/PI)*(r/L)*(w_l-w_r);

  if(isnan(v)){
    v = 0.0;
    w = 0.0;
  }

  path = path + v*(millis()-p_timer)/1000.0;
  angle = angle + w*(millis()-p_timer)/1000.0;
  p_timer = millis();

  // Serial.print(path);
  // Serial.print(',');
  // Serial.print(angle);
  // Serial.println();
  
}

void update(){
  float k = 0.7;
  if (isnan(rps_r))
  {
    rps_r = (rWheelCounter/20.0)/((millis()-r_timer)/1000.0);
    rWheelCounter = 0;
    rps_l = (lWheelCounter/20.0)/((millis()-r_timer)/1000.0);
    lWheelCounter = 0;
  }
  
  rps_r = (rWheelCounter/20.0)/((millis()-r_timer)/1000.0)*(k) + rps_r*(1-k);
  rWheelCounter = 0;
  rps_l = (lWheelCounter/20.0)/((millis()-r_timer)/1000.0)*(k) + rps_l*(1-k);
  lWheelCounter = 0;
  r_timer = millis();
  updatePath();
}


void setup() {
  Serial.begin(115200);
  motor1.setMode(AUTO);
  motor2.setMode(AUTO);

  attachInterrupt(digitalPinToInterrupt(RWS), RWheelCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(LWS), LWheelCounter, RISING);

  path = 0.0;
  angle = 0.0;
}

void move(int speed){
  motor1.setSpeed(speed * 0.8);
  motor2.setSpeed(speed);
}

void turn(int a){
  motor1.setSpeed(a*0.8);
  motor2.setSpeed(-a);
  update();
}

void stop(){
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

int data;
int state = 0;
int value;
void loop() {
  update();
  // delay(50);
  // keepSpeed(150,50);
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
    stop();       
    state = 0;
    break;
  case 2:
    turn(value - 200);
    break;
  case 3:
    keepSpeed(100,value - 4, 1);
    break;
  case 4:
    keepSpeed(value - 200, 0, ((value > 200)? 1:-1));
    break;
  case 5:
    move(value - 200);
    break;
  default:
    stop();
    state = 0;
    break;
  }

}
