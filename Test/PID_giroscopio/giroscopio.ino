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
const int IN1_SI = 47;
const int IN2_SI = 46;
const int ENA_SI = 7;
const int ENC_A_SI = 16;
const int ENC_B_SI = 17;

const int IN1_SD = 48;
const int IN2_SD = 49;
const int ENB_SD = 6;
const int ENC_A_SD = 18;
const int ENC_B_SD = 19;

const int IN1_II = 52;
const int IN2_II = 53;
const int ENA_II = 5;
const int ENC_A_II = 9;
const int ENC_B_II = 8; 

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

// PID constants
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

// PID variables
float error = 0;
float previous_error = 0;
float integral = 0;
float derivative = 0;
float output = 0;

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
    pinMode(ENC_A_SI, INPUT);
    pinMode(ENC_B_SI, INPUT);
    pinMode(ENC_A_SD, INPUT);
    pinMode(ENC_B_SD, INPUT);
    pinMode(ENC_A_II, INPUT);
    pinMode(ENC_B_II, INPUT);
    pinMode(ENC_A_ID, INPUT);
    pinMode(ENC_B_ID, INPUT);
}

void loop() {
    // Leer datos de giroscopio
    mpu.getRotation(&gx, &gy, &gz);

    // Convierte la rotación alrededor del eje Z a grados/segundo
    float giroZ = gz * gyroScale;

    // PID control
    error = -giroZ;  // Assuming we want to maintain zero rotation
    integral += error;
    derivative = error - previous_error;
    output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    // Adjust motor speeds based on PID output
    int baseSpeed = 200;  // Base speed for the motors
    int leftSpeed = baseSpeed + output;
    int rightSpeed = baseSpeed - output;

    analogWrite(ENA_SI, constrain(leftSpeed, 0, 255));
    analogWrite(ENA_II, constrain(leftSpeed, 0, 255));
    analogWrite(ENB_SD, constrain(rightSpeed, 0, 255));
    analogWrite(ENB_ID, constrain(rightSpeed, 0, 255));

    delay(100);  // Controla la frecuencia de ajuste
}