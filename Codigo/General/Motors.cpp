#include "Motors.h"

Motors::Motors(int IN1_SI, int IN2_SI, int ENB_SI,    // Top-left motor
               int IN1_SD, int IN2_SD, int ENA_SD,    // Top-right motor
               int IN1_II, int IN2_II, int ENB_II,    // Bottom-left motor
               int IN1_ID, int IN2_ID, int ENA_ID) {   // Bottom-right motor 
    // Variable assignment.
    IN1_SI_ = IN1_SI;
    IN2_SI_ = IN2_SI;
    ENB_SI_ = ENB_SI;

    IN1_SD_ = IN1_SD;
    IN2_SD_ = IN2_SD;
    ENA_SD_ = ENA_SD;

    IN1_II_ = IN1_II;
    IN2_II_ = IN2_II;
    ENB_II_ = ENB_II;

    IN1_ID_ = IN1_ID;
    IN2_ID_ = IN2_ID;
    ENA_ID_ = ENA_ID;
}

void Motors::init() {
    // Initialization of pins.
    pinMode(IN1_SI_, OUTPUT);
    pinMode(IN2_SI_, OUTPUT);
    pinMode(ENB_SI_, OUTPUT);

    pinMode(IN1_SD_, OUTPUT);
    pinMode(IN2_SD_, OUTPUT);
    pinMode(ENA_SD_, OUTPUT);

    pinMode(IN1_II_, OUTPUT);
    pinMode(IN2_II_, OUTPUT);
    pinMode(ENB_II_, OUTPUT);

    pinMode(IN1_ID_, OUTPUT);
    pinMode(IN2_ID_, OUTPUT);
    pinMode(ENA_ID_, OUTPUT);
}


void Motors::forward() {
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,190);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,190);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENA_II,200);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,200);

    delay(710); // Este delay jalara por cuadrante de 30 cm centrado en medio
}

void Motors::backward(){
    // Motor superior derecho
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,HIGH);
    analogWrite(ENA_SD,160);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,200);
    
    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENB_ID,160);

    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENB_SI,200);

    //delay(1000);
    delay(300);

void Motors::stop(){
    // Motor superior izquierdo
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,0);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,0);
    
    // Motor superior derecho
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,0);


    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENB_ID,0);
    delay(200);
}

void Motors::turnLeft() {
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,220);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,255);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENB_SI,220);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,255);

    delay(400);
}

void Motors::turnRight() {
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,70);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,70);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,200);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,190);

    //delay();
    delay(500);
}