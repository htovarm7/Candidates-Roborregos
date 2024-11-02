// //GND - GND
// //VCC - VCC
// //SDA - Pin A4
// //SCL - Pin A5
 
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
const int IN1_SI = 47;
const int IN2_SI = 46;
const int ENA_SI = 7;
const int IN1_SD = 48;
const int IN2_SD = 49;
const int ENB_SD = 6;
const int IN1_II = 52;
const int IN2_II = 53;
const int ENA_II = 5;
const int IN1_ID = 50;
const int IN2_ID = 49;
const int ENB_ID = 4;

const int mpuAddress = 0x68;  // Dirección I2C del MPU6050
MPU6050 mpu(mpuAddress);

int gx, gy, gz;
const float gyroScale = 250.0 / 32768.0;

// PID constants
float Kp = 1;   
float Ki = 3;  
float Kd = 10;

// PID variables
float error = 4;
float previous_error = 0;
float integral = 0;
float derivative = 0;
float output = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
    
    pinMode(IN1_SI, OUTPUT);
    pinMode(IN2_SI, OUTPUT);
    pinMode(ENA_SI, OUTPUT);

    pinMode(IN1_SD, OUTPUT);
    pinMode(IN2_SD, OUTPUT);
    pinMode(ENB_SD, OUTPUT);

    pinMode(IN1_II, OUTPUT);
    pinMode(IN2_II, OUTPUT);
    pinMode(ENA_II, OUTPUT);

    pinMode(IN1_ID, OUTPUT);
    pinMode(IN2_ID, OUTPUT);
    pinMode(ENB_ID, OUTPUT);
}

void loop() {
    // Leer datos de giroscopio
    mpu.getRotation(&gx, &gy, &gz);
  
    // Convierte la rotación alrededor del eje Z a grados/segundo
    float giroZ = gz * gyroScale;

    // PID control
    error = -giroZ;  // Mantener giroZ en cero
    integral += error;
    integral = constrain(integral, -50, 50);  // limite para el termino de integral

    if (abs(error) < 0.1) integral = 0;  // Banda muerta

    derivative = error - previous_error;
    output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, -255, 255);  // Limit output to valid range
    previous_error = error;

    // Ajuste de velocidades base por desbalanceo
    int leftBaseSpeed = 150;   
    int rightBaseSpeed = 130;  

    int leftSpeed = leftBaseSpeed + output;
    int rightSpeed = rightBaseSpeed - output;

    analogWrite(ENA_SI, constrain(leftSpeed, 0, 255));
    analogWrite(ENA_II, constrain(leftSpeed, 0, 255));
    analogWrite(ENB_SD, constrain(rightSpeed, 0, 255));
    analogWrite(ENB_ID, constrain(rightSpeed, 0, 255));

    // Movimiento hacia adelante
    digitalWrite(IN1_SI, HIGH);
    digitalWrite(IN2_SI, LOW);

    digitalWrite(IN1_SD, HIGH);
    digitalWrite(IN2_SD, LOW);

    digitalWrite(IN1_II, LOW);
    digitalWrite(IN2_II, HIGH);
    
    digitalWrite(IN1_ID, LOW);
    digitalWrite(IN2_ID, HIGH);

    delay(100);
}