#ifndef MOTOR_H_
#define MOTOR_H_
#include <Arduino.h>

class Motor {
  public:
    Motor();
    void init(int apin, int bpin, int pwm);
    void ugoki(int syutu);
    void tomaru();
  private:
    int pin[3];
};
#endif


