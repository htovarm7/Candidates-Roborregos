#include <QTRSensors.h>

// Declaración de motores
const int ENC_A = 6;
const int ENC_B = 5;

const int IN1 = 2;
const int IN2 = 4;
const int ENA = 3;


// Definir el pin del sensor de línea
QTRSensorsAnalog qtrrc((unsigned char[]) {A0}, 1); // Un solo sensor
unsigned int sensorValue; // Variable para almacenar el valor del sensor
// Definir el umbral para detectar la línea
const int umbral = 500;  // Ajustar cuando se calibre


void setup() {

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(ENC_A), countPulse, RISING);

  calibrarSensor(); // Calibrar el sensor de línea
}

void loop() {
   qtrrc.read(&sensorValue); // Leer el valor del sensor
  Serial.println(sensorValue);

  if (sensorValue < umbral) {
    // Si detecta la línea, avanza (funcion generales)
    avanzar();
  } else {
    // Si no detecta la línea, gira (funcion generales)
    girar();
  }

  delay(100);
}


void calibrarSensor() {
  Serial.println("Calibrando el sensor de línea...");
  
  for (int i = 0; i < 400; i++) { // Calibrar durante 4 segundos aprox.
    qtrrc.calibrate();
    delay(10);
  }

  // Mostrar los valores máximo y mínimo calibrados
  Serial.print("Valor mínimo calibrado: ");
  Serial.println(qtrrc.calibratedMinimumOn[0]);
  Serial.print("Valor máximo calibrado: ");
  Serial.println(qtrrc.calibratedMaximumOn[0]);

  // Se ajusta el valor del umbral
  umbral = (qtrrc.calibratedMinimumOn[0] + qtrrc.calibratedMaximumOn[0]) / 2; // Se calcula el promedio para tener mejor rangos
  Serial.print("Umbral calculado: ");
  Serial.println(umbral);
}