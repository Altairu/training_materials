#include "motor.h"

Motor::Motor() {
}
void Motor::init(int apin, int bpin, int pwm) {
  pin[0] = apin;
  pin[1] = bpin;
  pin[2] = pwm;

  pinMode(pin[0], OUTPUT);
  pinMode(pin[1], OUTPUT);
  pinMode(pin[2], OUTPUT);
  analogWrite(pin[2], 0); // Arduino UnoではanalogWriteを使います
}

void Motor::ugoki(int syutu) {
  if (syutu > 0) {
    digitalWrite(pin[0], HIGH);
    digitalWrite(pin[1], LOW);
    analogWrite(pin[2], syutu); // Arduino UnoではanalogWriteを使います
  } else if (syutu < 0) {
    digitalWrite(pin[0], LOW);
    digitalWrite(pin[1], HIGH);
    analogWrite(pin[2], -1 * syutu); // Arduino UnoではanalogWriteを使います
  } else {
    digitalWrite(pin[0], LOW);
    digitalWrite(pin[1], LOW);
    analogWrite(pin[2], 0); // 停止時
  }
}

void Motor::tomaru() {
  digitalWrite(pin[0], HIGH);
  digitalWrite(pin[1], HIGH);
  analogWrite(pin[2], 0); // 停止
}