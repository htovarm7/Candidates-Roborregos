//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5
 
// #include "I2Cdev.h"
// #include "MPU6050.h"
// #include "Wire.h"
 
//  //INT 26
// const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
// MPU6050 mpu(mpuAddress);
 
// int ax, ay, az;
// int gx, gy, gz;
 
 
// // Factores de conversion
// const float accScale = 2.0 * 9.81 / 32768.0;
// const float gyroScale = 250.0 / 32768.0;
 
// void printTab()
// {
//    Serial.print(F("\t"));
// }
 
// // Mostrar medidas en Sistema Internacional
// void printRAW()
// {
//    Serial.print(F("a[x y z] (m/s2) g[x y z] (deg/s):t"));
//    Serial.print(ax * accScale); printTab();
//    Serial.print(ay * accScale); printTab();
//    Serial.print(az * accScale); printTab();
//    Serial.print(gx * gyroScale);  printTab();
//    Serial.print(gy * gyroScale);  printTab();
//    Serial.println(gz * gyroScale);
// } 
 
// void setup()
// {
//    Serial.begin(9600);
//    Wire.begin();
//    Serial.println("asjndknakd");
//    mpu.initialize();
//    Serial.println("Pase");
//    Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
// }
 
// void loop()
// {
//    // Leer las aceleraciones y velocidades angulares
//    mpu.getAcceleration(&ax, &ay, &az);
//    mpu.getRotation(&gx, &gy, &gz);
 
//    printRAW();
 
//    delay(100);
// }

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Motores
//  Pines motor superior izquierdo
const int IN1_SI = 47;
const int IN2_SI = 46;
const int ENA_SI = 7;
const int ENC_A_SI = 16;
const int ENC_B_SI = 17;

// Pines motor superior derecho
const int IN1_SD = 48;
const int IN2_SD = 49;
const int ENB_SD = 6;
const int ENC_A_SD = 18;
const int ENC_B_SD = 19;

// Pines motor inferior izquierdo
const int IN1_II = 52;
const int IN2_II = 53;
const int ENA_II = 5;
const int ENC_A_II = 9;
const int ENC_B_II = 8; 

// Pines motor inferior derecho
const int IN1_ID = 50;
const int IN2_ID = 49;
const int ENB_ID = 4;
const int ENC_A_ID = 26;
const int ENC_B_ID = 27;

const int mpuAddress = 0x68;  // Dirección I2C del MPU6050
MPU6050 mpu(mpuAddress);

int ax, ay, az;
int gx, gy, gz;
const float gyroScale = 250.0 / 32768.0;

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

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
    
    pinMode(IN1_SI,OUTPUT);
    pinMode(IN2_SI,OUTPUT);
    pinMode(ENA_SI,OUTPUT);

    pinMode(IN1_SD,OUTPUT);
    pinMode(IN2_SD,OUTPUT);
    pinMode(ENB_SD,OUTPUT);

    pinMode(IN1_II,OUTPUT);
    pinMode(IN2_II,OUTPUT);
    pinMode(ENA_II,OUTPUT);

    pinMode(IN1_ID,OUTPUT);
    pinMode(IN2_ID,OUTPUT);
    pinMode(ENB_ID,OUTPUT);

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
}

void loop() {
    // Leer datos de giroscopio
    mpu.getRotation(&gx, &gy, &gz);

    // Convierte la rotación alrededor del eje Z a grados/segundo
    float giroZ = gz * gyroScale;

    // Control de movimiento hacia adelante con corrección
    if (abs(giroZ) > 1) { // Si se detecta una rotación significativa
        if (giroZ > 1) {
            // Robot gira a la derecha, reduce velocidad de motores de la derecha
            analogWrite(ENB_SD, 95);  // Motor superior derecho
            analogWrite(ENB_ID, 95);  // Motor inferior derecho
        } else if (giroZ < -1) {
            // Robot gira a la izquierda, reduce velocidad de motores de la izquierda
            analogWrite(ENA_SI, 120);  // Motor superior izquierdo
            analogWrite(ENA_II, 120);  // Motor inferior izquierdo
        }
    } else {
        // Movimiento normal hacia adelante
        adelante();  // Función definida en tu código para avanzar
    }

    delay(100);  // Controla la frecuencia de ajuste
}
