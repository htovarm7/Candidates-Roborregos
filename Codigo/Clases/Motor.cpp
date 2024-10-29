#include "Motor.h"

Motor::Motor(int encA, int encB, int in1, int in2, int ena)
    : ENC_A(encA), ENC_B(encB), IN1(in1), IN2(in2), ENA(ena) {
    // Set motor pins as output
    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
}

void Motor::avanzar(int velocidad) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, velocidad);  // Set speed (0 to 255)
}

void Motor::detener() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);  // Stop the motor
}
