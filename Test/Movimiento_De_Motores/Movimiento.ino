#include <Wire.h>
#include <MPU6050_tockn.h>

// CONSTANTES DE MOTORES

// Pines motor superior izquierdo
const int IN1_SI = 47;
const int IN2_SI = 46;
const int ENA_SI = 7;

// Pines motor superior derecho
const int IN1_SD = 48;
const int IN2_SD = 49;
const int ENB_SD = 6;

// Pines motor inferior izquierdo
const int IN1_II = 52;
const int IN2_II = 53;
const int ENA_II = 5;

// Pines motor inferior derecho
const int IN1_ID = 50;
const int IN2_ID = 49;
const int ENB_ID = 4;

const int pwmAdelante = 255;
const int pwmReversa = 230;
const int zero = 0;

// OBJETO GIROSCOPIO
MPU6050 mpu(Wire);

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Inicialización de pines de motores
    pinMode(IN1_SI, OUTPUT); pinMode(IN2_SI, OUTPUT); pinMode(ENA_SI, OUTPUT);
    pinMode(IN1_SD, OUTPUT); pinMode(IN2_SD, OUTPUT); pinMode(ENB_SD, OUTPUT);
    pinMode(IN1_II, OUTPUT); pinMode(IN2_II, OUTPUT); pinMode(ENA_II, OUTPUT);
    pinMode(IN1_ID, OUTPUT); pinMode(IN2_ID, OUTPUT); pinMode(ENB_ID, OUTPUT);

    // Inicializar el MPU-6050
    Serial.println("Inicializando el MPU-6050...");
    mpu.begin();
    Serial.println("avance...");
    mpu.calcGyroOffsets(true); // Calibra el giroscopio con el sensor estable
    Serial.println("MPU-6050 inicializado correctamente.");
}

// Funciones de movimiento
void adelante() { /* Mantén tu código de adelante */ }
void reversa() { /* Mantén tu código de reversa */ }
void detener() { /* Mantén tu código de detener */ }
void giroDerecha() { /* Mantén tu código de giroDerecha */ }
void giroIzquierda() { /* Mantén tu código de giroIzquierda */ }
void movLateral() { /* Mantén tu código de movLateral */ }

// Función para leer giroscopio y mostrar datos
void leerGiroscopio() {
    mpu.update();

    Serial.print("Gyro X: "); Serial.print(mpu.getGyroX());
    Serial.print(" | Gyro Y: "); Serial.print(mpu.getGyroY());
    Serial.print(" | Gyro Z: "); Serial.println(mpu.getGyroZ());

    Serial.print("Accel X: "); Serial.print(mpu.getAccX());
    Serial.print(" | Accel Y: "); Serial.print(mpu.getAccY());
    Serial.print(" | Accel Z: "); Serial.println(mpu.getAccZ());

    delay(500); // Pausa de 500ms entre lecturas
}

void loop() {
    // Prueba de funciones de movimiento y giroscopio
    adelante();
    detener();
    reversa();
    detener();
    giroDerecha();
    detener();
    leerGiroscopio();
}
