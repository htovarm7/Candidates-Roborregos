#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(int encA, int encB, int in1, int in2, int ena);
    void avanzar(int velocidad);
    void detener();  

private:
    int ENC_A, ENC_B, IN1, IN2, ENA;  // Pin variables
};

#endif
