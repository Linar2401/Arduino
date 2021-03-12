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

#define M_V 43.0
#define M_W 7.0


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

float r = 6.7/2;
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

void keepSpeed(int linear, int angular){
  m1 = constrain(80 + P((v/M_V)*100, linear, 0.75) + angular + P((w/(M_W*PI))*100, angular, 0.9), 0, 255);
  m2 = constrain(100 + P((v/M_V)*100, linear, 0.75) - angular - P((w/(M_W*PI))*100, angular, 0.9), 0, 255);
  motor1.setSpeed(m1);
  motor2.setSpeed(m2);

  Serial.print(m1);
  Serial.print(',');
  Serial.print(m2);
  Serial.print(',');
  Serial.print(v);
  Serial.print(',');
  Serial.print(w);
  Serial.println();
  k_timer = millis();
}

void updatePath(){
  w_l = PI*rps_l;
  w_r = PI*rps_r;

  v = (r/2)*(w_l+w_r);
  w = (r/L)*(w_l-w_r);

  if(isnan(v)){
    v = 0.0;
    w = 0.0;
  }

  path = path + v*(millis()-p_timer)/1000.0;
  angle = angle + (180.0/PI)*w*(millis()-p_timer)/1000.0;
  p_timer = millis();

  // Serial.print(path);
  // Serial.print(',');
  // Serial.print(angle);
  // Serial.println();
  
}

void update(){
  rps_r = (rWheelCounter/20.0)/((millis()-r_timer)/1000.0);
  rWheelCounter = 0;
  rps_l = (lWheelCounter/20.0)/((millis()-r_timer)/1000.0);
  lWheelCounter = 0;
  r_timer = millis();
  updatePath();
}

void move(int dist){
  int p_point = path + dist;
  while (abs(p_point - path) > 3)
  { 
    motor1.setSpeed(200);
    motor2.setSpeed(250);
    update();
    Serial.print(path);
    Serial.print(',');
    Serial.print(p_point);
    Serial.print(',');
    Serial.print(abs(p_point - path));
    Serial.println();
   
  }
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

void turn(int deg){
  int a_point = angle + deg;
  while (abs(a_point - angle) > 3)
  { 
    if((deg/abs(deg)) > 0){
      motor2.setSpeed(120 * (deg/abs(deg)));
    }
    else{
      motor1.setSpeed(-80 * (deg/abs(deg)));
    }
    
    update();

    Serial.print(angle);
    Serial.print(',');
    Serial.print(a_point);
    Serial.print(',');
    Serial.print(abs(a_point - angle));
    Serial.println();
  }
  motor1.setSpeed(0);
  motor2.setSpeed(0);
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

void loop() {
  update();
  delay(47);
  // keepSpeed(200,0);
  // motor1.setSpeed(80);
  // motor2.setSpeed(-100);

  int a = 50;
  int d = 15; 

  
  move(d);
  delay(500);
  turn(a);
  delay(500);
  move(d);
  delay(500);
  turn(a);
  delay(500);
  move(d);
  delay(500);
  turn(a);
  delay(500);
  move(d);
  delay(500);
  turn(a);
  delay(500);
  
  delay(500000);
}
