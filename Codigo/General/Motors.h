#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "PID.h"

class Motors {
public:
    Motors(int IN1_SI, int IN2_SI, int ENB_SI,    // Top-left motor
           int IN1_SD, int IN2_SD, int ENA_SD,    // Top-right motor
           int IN1_II, int IN2_II, int ENB_II,    // Bottom-left motor
           int IN1_ID, int IN2_ID, int ENA_ID);   // Bottom-right motor

    void forward();
    void backward();
    void turnLeft();
    void turnRight();
    void stop();
    void init();

private:
    // Pin variables for each motor.
    int IN1_SI_, IN2_SI_, ENB_SI_; // Top-left motor.
    int IN1_SD_, IN2_SD_, ENA_SD_; // Top-right motor.
    int IN1_II_, IN2_II_, ENB_II_; // Bottom-left motor.
    int IN1_ID_, IN2_ID_, ENA_ID_; // Bottom-right motor.
};

#endif
