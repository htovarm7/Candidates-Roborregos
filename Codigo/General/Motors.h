#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

class Motors {
public:
    Motors(int INA1L, int INA2L, int ENAL,    // Top-left motor
           int INB1L, int INB2L, int ENBL,    // Top-right motor
           int INA1R, int INA2R, int ENAR,    // Bottom-left motor
           int INB1R, int INB2R, int ENBR);   // Bottom-right motor

    void forward(int speed);
    void backward();
    void stop();
    void init();

private:
    // Pin variables.
    int INA1L_, INA2L_, ENAL_; // Top-left corner.
    int INB1L_, INB2L_, ENBL_; // Top-right corner.
    int INA1R_, INA2R_, ENAR_; // Bottom-left corner.
    int INB1R_, INB2R_, ENBR_; // Bottom-right corner.
};

#endif
