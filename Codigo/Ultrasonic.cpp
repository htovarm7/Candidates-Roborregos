#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int trigPin, int echoPin) {
    trigPin_ = trigPin;
    echoPin_ = echoPin;
}

void Ultrasonic::init() {
    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
}

float Ultrasonic::getDistance() {
    digitalWrite(trigPin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);

    float duration = pulseIn(echoPin_, HIGH);
    float distance = duration * 0.034 / 2;
    return distance;
}