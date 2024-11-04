#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class Ultrasonic {
public:
    Ultrasonic(int, int);
    long getDistance();
    void init();

private:
    int trigPin_;
    int echoPin_;
};

#endif // ULTRASONIC_H