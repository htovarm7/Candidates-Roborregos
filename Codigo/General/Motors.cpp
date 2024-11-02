#include "Motors.h"

Motors::Motors(int INA1L, int INA2L, int ENAL,    // Top-left motor
               int INB1L, int INB2L, int ENBL,    // Top-right motor
               int INA1R, int INA2R, int ENAR,    // Bottom-left motor
               int INB1R, int INB2R, int ENBR) {   // Bottom-right motor 
    // Variable assignment.
    INA1L_ = INA1L;
    INA2L_ = INA2L;
    ENAL_ = ENAL;

    INB1L_ = INB1L;
    INB2L_ = INB2L;
    ENBL_ = ENBL;

    INA1R_ = INA1R;
    INA2R_ = INA2R;
    ENAR_ = ENAR;

    INB1R_ = INB1R;
    INB2R_ = INB2R;
    ENBR_ = ENBR;
}

void Motors::init() {
    // Initialization of pins.
    pinMode(INA1L_, OUTPUT);
    pinMode(INA2L_, OUTPUT);
    pinMode(ENAL_, OUTPUT);

    pinMode(INB1L_, OUTPUT);
    pinMode(INB2L_, OUTPUT);
    pinMode(ENBL_, OUTPUT);

    pinMode(INA1R_, OUTPUT);
    pinMode(INA2R_, OUTPUT);
    pinMode(ENAR_, OUTPUT);

    pinMode(INB1R_, OUTPUT);
    pinMode(INB2R_, OUTPUT);
    pinMode(ENBR_, OUTPUT);
}


void Motors::forward() {
    digitalWrite(INA1L_, HIGH);
    digitalWrite(INA2L_, LOW);
    analogWrite(ENAL_, 240);
    
    
    digitalWrite(INB1L_, HIGH);
    digitalWrite(INB2L_, LOW);
    analogWrite(ENBL_, 170);


    digitalWrite(INA1R_, HIGH);
    digitalWrite(INA2R_, LOW);
    analogWrite(ENAR_, 130);


    digitalWrite(INB1R_, HIGH);
    digitalWrite(INB2R_, LOW);
    analogWrite(ENBR_, 220);
}

void Motors::backward(){
    
    digitalWrite(INA1L_, LOW);
    digitalWrite(INA2L_, LOW);
    analogWrite(ENAL_, 0);


    digitalWrite(INA1R_, LOW);
    digitalWrite(INA2R_, LOW);
    analogWrite(ENAR_, 0);


    digitalWrite(INB1L_, LOW);
    digitalWrite(INB2L_, LOW);
    analogWrite(ENBL_, 0);


    digitalWrite(INB1R_, LOW);
    digitalWrite(INB2R_, LOW);
    analogWrite(ENBR_, 0);
}

void Motors::stop(){
    digitalWrite(INA1L_, LOW);
    digitalWrite(INA2L_, LOW);
    analogWrite(ENAL_, 0);


    digitalWrite(INA1R_, LOW);
    digitalWrite(INA2R_, LOW);
    analogWrite(ENAR_, 0);


    digitalWrite(INB1L_, LOW);
    digitalWrite(INB2L_, LOW);
    analogWrite(ENBL_, 0);


    digitalWrite(INB1R_, LOW);
    digitalWrite(INB2R_, LOW);
    analogWrite(ENBR_, 0);
}

void Motors::turnLeft() {
    digitalWrite(INA1L_, HIGH);
    digitalWrite(INA2L_, LOW);
    analogWrite(ENAL_, 200);


    digitalWrite(INA1R_, HIGH);
    digitalWrite(INA2R_, LOW);
    analogWrite(ENAR_, 200);


    digitalWrite(INB1L_, LOW);
    digitalWrite(INB2L_, HIGH);
    analogWrite(ENBL_, 230);


    digitalWrite(INB1R_, LOW);
    digitalWrite(INB2R_, HIGH);
    analogWrite(ENBR_, 230);

    delay(1280);
}

void Motors::turnRight() {
    digitalWrite(INA1L_, HIGH);
    digitalWrite(INA2L_, LOW);
    analogWrite(ENAL_, 0);


    digitalWrite(INA1R_, LOW);
    digitalWrite(INA2R_, HIGH);
    analogWrite(ENAR_, 200);


    digitalWrite(INB1L_, HIGH);
    digitalWrite(INB2L_, LOW);
    analogWrite(ENBL_, 200);


    digitalWrite(INB1R_, LOW);
    digitalWrite(INB2R_, HIGH);
    analogWrite(ENBR_, 200);

    delay(1280);
}