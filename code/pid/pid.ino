#include "motor.h"

const int encoderPinA = 2;  // A相
const int encoderPinB = 3;  // B相
volatile long encoderCount = 0;  // エンコーダーのカウンター
int degrees = 30;  // エンコーダーの角度

double Kp = 20;
double Ki = 0.01; 
double Kd = 0;

Motor mtr[1];

int speed1 =0;
int target =0;

void setup() {
  Serial.begin(115200);
  mtr[0].init(9, 10, 11);
  
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  // エンコーダーピン割り込み
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
}

void loop() {  
  mtr[0].ugoki(speed1);
  pid();
  // エンコーダーの角度を計算
  degrees = encoderCount*360/(2048*2);
  Serial.print("Angle: ");
  Serial.print(degrees);
  Serial.print("speed1:");
  Serial.print(speed1);
  Serial.println('\t');
}

// エンコーダーカウンターを更新する割り込みサービスルーチン
void updateEncoder() {
  if (digitalRead(encoderPinB) == digitalRead(encoderPinA)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void pid() {
  static int previousError = 0; // 前回の誤差
  static int integral = 0; // 積分項
  int error = target - degrees; // 現在の誤差

  // P制御
  int P = Kp * error;

  // I制御（積分項）
  integral += error;
  int I = Ki * integral;

  // D制御（微分項）
  int derivative = error - previousError;
  int D = Kd * derivative;

  // PID制御の計算
  speed1 = P + I + D;

  // 誤差と積分項を更新
  previousError = error;
}
