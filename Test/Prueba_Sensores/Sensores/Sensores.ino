// Los include
//#include "I2Cdev.h"
//#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h>   

// Pines ultrasonico Izquierdo
const int leftEcho = 42;
const int leftTrig = 43;

// Pines ultrasonico Frontal
const int frontEcho = 40;
const int frontTrig = 41;

// Pines ultrasonico Derecha
const int rightEcho = 38;
const int rightTrig = 39;

// Motores
//  Pines motor superior izquierdo
const int IN1_SI = 47;
const int IN2_SI = 46;
const int ENA_SI = 7;
const int ENC_A_SI = 11;
const int ENC_B_SI = 10;

// Pines motor superior derecho
const int IN1_SD = 48;
const int IN2_SD = 49;
const int ENB_SD = 6;
const int ENC_A_SD = 9;
const int ENC_B_SD = 8;

// Pines motor inferior izquierdo
const int IN1_II = 52;
const int IN2_II = 53;
const int ENA_II = 5;
const int ENC_A_II = 13;
const int ENC_B_II = 12;

// Pines motor inferior derecho
const int IN1_ID = 50;
const int IN2_ID = 49;
const int ENB_ID = 4;
const int ENC_A_ID = 7;
const int ENC_B_ID = 6;

const int pwmIzq = 255;
const int pmwDer = 255;

// Actuadores

// Servomotores
const int Servo_SI = 22;
const int Servo_SD = 23;
Servo Servo1;
Servo Servo2;

// Sensor de color
const char SCL_COLOR = A5;
const char SDA_COLOR = A4; 

// Giroscopio
// const char SCL_GIRO = A2;
// const char SDA_GIRO = A3;

// LED RGB Falta poner los pines al arduino
const int R = 10;
const int G = 11;
const int B = 12;

// Giroscopio
const int giroSCL = 1; 
const int giroSDA = 2; 

// Sensores de línea
const int sensorLineaD8 = 46;
const int sensorLineaD7 = 47;
const int sensorLineaD6 = 48;
const int sensorLineaD5 = 49;
const int sensorLineaD4 = 50;
const int sensorLineaD3 = 51;
const int sensorLineaD2 = 52;
const int sensorLineaD1 = 53;

// Para los encoders
volatile int encoderCountSI = 0;
volatile int encoderCountSD = 0;
volatile int encoderCountII = 0;
volatile int encoderCountID = 0;

// Velocidad de motores
const int pwmAdelante = 130;

// FUnciones para probar cada sensor
long distanciaUltrasonico(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duracion = pulseIn(echoPin, HIGH);
    long distancia = duracion * 0.034 / 2;
    return distancia;
}

// Motores
// Funciones para probar cada motor
void motorSuperiorDerecho(){
  delay(1500);
  Serial.println("Motor Superior Derecho");
  delay(1500);

  digitalWrite(IN1_SD,HIGH);
  digitalWrite(IN2_SD,LOW);
  analogWrite(ENB_SD,100);
  delay(1500);

}

void motorInferiorDerecho(){
  delay(1500);
  Serial.println("Motor Inferior Derecho");
  delay(1500);

  digitalWrite(IN1_ID,HIGH);
  digitalWrite(IN2_ID,LOW);
  analogWrite(ENB_ID,100);
  delay(1500);
}

void motorSuperiorIzquierdo(){
  delay(1500);
  Serial.println("Motor Superior Izquierdo");
  delay(1500);

  digitalWrite(IN1_SI,HIGH);
  digitalWrite(IN2_SI,LOW);
  analogWrite(ENA_SI,100);
  delay(1500);
}

void motorInferiorIzquierdo(){
  delay(1500);
  Serial.println("Motor Inferior Izquierdo");
  delay(1500);

  digitalWrite(IN1_II,HIGH);
  digitalWrite(IN2_II,LOW);
  analogWrite(ENA_II,100);
  delay(1500);

}

// Funciones para probar todos los motores con sus diferentes movimientos
void adelante(){
    // Motor superior derecho JALA 
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENA_SI,240);

    // Motor inferior izquierdo JALA CM 
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENA_II,170);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENB_SD,130);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,220);
    delay(1200);
}

// ESTO YA QUEDA ASI FUNCIONA
void reversa(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENA_SI,pwmAdelante);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,pwmAdelante);
    
    // Motor superior derecho,
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENB_SD,255);

    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENB_ID,pwmAdelante);
    delay(1500);
}

void detener(){
  // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENA_SI,0);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,0);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENB_SD,0);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENB_ID,0);
    delay(3000);
}

void detenerConReversa(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENA_SI,pwmAdelante);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,pwmAdelante);
    
    // Motor superior derecho,
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENB_SD,255);

    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENB_ID,pwmAdelante);
    delay(300);
}

void giroDerecha(){

    // Motor superior derecho JALA 
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENA_SI,0);

    // Motor inferior izquierdo JALA CM
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENA_II,200);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENB_SD,200);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,200);
    delay(1280);
    
}

void giroIzquierda(){
    // Motor superior DERECHO
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENA_SI,200);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,200);
    
    // // Motor superior IZQUIERDO
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,HIGH);
    analogWrite(ENB_SD,230);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,230);
    delay(1280);
}

// void movLateral(){
//     // Motor superior izquierdo
//     digitalWrite(IN1_SI,LOW);
//     digitalWrite(IN2_SI,HIGH);
//     analogWrite(ENB_SI,pwmIzq);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,HIGH);
//     digitalWrite(IN2_II,LOW);
//     analogWrite(ENB_II,pwmIzq);
    
//     // Motor superior derecho
//     digitalWrite(IN1_SD,LOW);
//     digitalWrite(IN2_SD,HIGH);
//     analogWrite(ENA_SD,pwmDer);

//     // Motor inferior derecho
//     digitalWrite(IN1_ID,HIGH);
//     digitalWrite(IN2_ID,LOW);
//     analogWrite(ENA_ID,pwmDer);
    
//     delay(2000);
// }

// Para los encoders de cada motor
void encoderISR_SI() {
    encoderCountSI++;
}

void encoderISR_SD() {
    encoderCountSD++;
}

void encoderISR_II() {
    encoderCountII++;
}

void encoderISR_ID() {
    encoderCountID++;
}

void setupEncoders() {
    attachInterrupt(digitalPinToInterrupt(ENC_A_SI), encoderISR_SI, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A_SD), encoderISR_SD, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A_II), encoderISR_II, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A_ID), encoderISR_ID, RISING);
}

void calculateRPM() {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    unsigned long timeDiff = currentTime - lastTime;

    if (timeDiff >= 1000) { // Calculate RPM every second
        float rpmSI = (encoderCountSI / 20.0) * 60.0; // Assuming 20 pulses per revolution
        float rpmSD = (encoderCountSD / 20.0) * 60.0;
        float rpmII = (encoderCountII / 20.0) * 60.0;
        float rpmID = (encoderCountID / 20.0) * 60.0;

        Serial.print("RPM SI: ");
        Serial.println(rpmSI);
        Serial.print("RPM SD: ");
        Serial.println(rpmSD);
        Serial.print("RPM II: ");
        Serial.println(rpmII);
        Serial.print("RPM ID: ");
        Serial.println(rpmID);

        encoderCountSI = 0;
        encoderCountSD = 0;
        encoderCountII = 0;
        encoderCountID = 0;
        lastTime = currentTime;
        delay(1000);
    }
}

void RGB() {
  // Yellow
  Serial.println("Yellow");
  analogWrite(R, 255);
  analogWrite(G, 255);
  analogWrite(B, 0);
  delay(2000);

  // Black
  Serial.println("Black");
  analogWrite(R, 0);
  analogWrite(G, 0);
  analogWrite(B, 0);
  delay(2000);

  // Purple
  Serial.println("Purple");
  analogWrite(R, 31);
  analogWrite(G, 0);
  analogWrite(B, 56);
  delay(2000);
  
  // Blue
  Serial.println("Blue");
  analogWrite(R, 0);
  analogWrite(G, 0);
  analogWrite(B, 255);
  delay(2000);

  // Pink
  Serial.println("Pink");
  analogWrite(R, 255);
  analogWrite(G, 0);
  analogWrite(B, 200);
  delay(2000);

  // Red
  Serial.println("Red");
  analogWrite(R, 255);
  analogWrite(G, 0);
  analogWrite(B, 0);
  delay(2000);

  // Green
  Serial.println("Green");
  analogWrite(R, 0);
  analogWrite(G, 255);
  analogWrite(B, 0);
  delay(2000);
}

void abrirCupula(){
  Servo1.write(135); 
  Servo2.write(45);
  delay(1000); 
}

void cerrarCupula(){
  Servo1.write(135); 
  Servo2.write(45);
  delay(1000); 
}

void setup(){
  Serial.begin(9600);

    // Ultrasonico Izquierdo
    pinMode(leftEcho, INPUT);
    pinMode(leftTrig, OUTPUT);

    // Ultrasonico Frontal
    pinMode(frontEcho, INPUT);
    pinMode(frontTrig, OUTPUT);

    // Ultrasonico Derecha
    pinMode(rightEcho, INPUT);
    pinMode(rightTrig, OUTPUT);

    //Motores 
    // Motor superior izquierdo
    pinMode(IN1_SI, OUTPUT);
    pinMode(IN2_SI, OUTPUT);
    pinMode(ENA_SI, OUTPUT);

    // Motor superior derecho
    pinMode(IN1_SD, OUTPUT);
    pinMode(IN2_SD, OUTPUT);
    pinMode(ENB_SD, OUTPUT);

    // Motor inferior izquierdo
    pinMode(IN1_II, OUTPUT);
    pinMode(IN2_II, OUTPUT);
    pinMode(ENA_II, OUTPUT);

    // Motor inferior derecho
    pinMode(IN1_ID, OUTPUT);
    pinMode(IN2_ID, OUTPUT);
    pinMode(ENB_ID, OUTPUT);

    // Actuadores
    pinMode(R, OUTPUT);
    pinMode(B, OUTPUT);
    pinMode(G, OUTPUT);
    
    // Encoders
    // Encoder Superior Izquierdo
    pinMode(ENC_A_SI, INPUT);
    pinMode(ENC_B_SI, INPUT);
    
    // Encoder Superior Derecho
    pinMode(ENC_A_SD, INPUT);
    pinMode(ENC_B_SD, INPUT);
    
    // Encoder Inferior Izquierdo
    pinMode(ENC_A_II, INPUT);
    pinMode(ENC_B_II, INPUT);

    // Encoder Inferior Derecho
    pinMode(ENC_A_ID, INPUT);
    pinMode(ENC_B_ID, INPUT);
    
    // Sensor de Linea
    pinMode(sensorLineaD8, INPUT);
    pinMode(sensorLineaD7, INPUT);
    pinMode(sensorLineaD6, INPUT);
    pinMode(sensorLineaD5, INPUT);
    pinMode(sensorLineaD4, INPUT);
    pinMode(sensorLineaD3, INPUT);
    pinMode(sensorLineaD2, INPUT);
    pinMode(sensorLineaD1, INPUT);

    // Servomotor
    Servo1.attach(22); //Derecho
    Servo2.attach(23); // Izquierdo considerando que se ve desde enfrente
}

void loop(){
    // long distanciaIzquierdo = distanciaUltrasonico(leftTrig, leftEcho);

    // long distanciaFrontal = distanciaUltrasonico(frontTrig, frontEcho);
    
    // long distanciaDerecha = distanciaUltrasonico(rightTrig, leftEcho);

    // // Distancia de los ultrasonicos
    // //Izquierdo
    // Serial.print("Izquierdo: ");
    // Serial.print(distanciaIzquierdo);
    // Serial.println(" cm");
    // //Frontal
    // Serial.print("Frontal: ");
    // Serial.print(distanciaFrontal);
    // Serial.println(" cm");
    // //Derecha
    // Serial.print("Derecha: ");
    // Serial.print(distanciaDerecha);
    // Serial.println(" cm");
    // delay(1000);

    // Para que no choque
    // if(distanciaFrontal > 30){
    //     adelante();
    // }
    // else { 
    //     detenerConReversa();
    //     //detener();
    // }
    adelante();
    calculateRPM();
    // abrirCupula();
    // cerrarCupula();

    //RGB();
}