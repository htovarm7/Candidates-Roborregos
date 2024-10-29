// Aun no se como implementarlo

#ifndef ULTRASONICO_H
#define ULTRASONICO_H

#include <Arduino.h>

class Ultrasonico {
public:
    Ultrasonico(int trigPin, int echoPin);
    long medirDistancia();

private:
    int trigPin;
    int echoPin;
};

Ultrasonico::Ultrasonico(int trigPin, int echoPin) {
    this->trigPin = trigPin;
    this->echoPin = echoPin;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

long Ultrasonico::medirDistancia() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duracion = pulseIn(echoPin, HIGH);
    long distancia = duracion * 0.034 / 2;
    return distancia;
}

#endif // ULTRASONICO_H