#include <Servo.h>

Servo Servo1;
Servo Servo2;

void abrirCupula(){
  Servo1.write(60);
  Servo2.write(120);
}

void cerrarCupula(){
  Servo1.write(150); 
  Servo2.write(30);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Servo1.attach(22); //Derecho
  Servo2.attach(23); // Izquierdo considerando que se ve desde enfrent
}

void loop() {
  abrirCupula();
  cerrarCupula();
  
}