#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int trigPin, int echoPin) {
    trigPin_ = trigPin;
    echoPin_ = echoPin;
}

void Ultrasonic::init() {
    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
}

long Ultrasonic::getDistance() {
    digitalWrite(trigPin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);

    long duration = pulseIn(echoPin_, HIGH);
    long distance = duration * 0.034 / 2;
    Serial.print(duration);
    Serial.print(" ");
    Serial.println(distance);
    return distance;
}