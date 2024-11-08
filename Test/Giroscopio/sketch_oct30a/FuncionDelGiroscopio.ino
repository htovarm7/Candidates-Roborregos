// /* CONSTANTS */

// // Motores
// //  Pines motor superior izquierdo
// const int IN1_SI = 47;
// const int IN2_SI = 46;
// const int ENA_SI = 7;
// const int ENC_A_SI = 9;
// const int ENC_B_SI = 8;

// // Pines motor superior derecho
// const int IN1_SD = 48;
// const int IN2_SD = 49;
// const int ENB_SD = 6;
// const int ENC_A_SD = 3;
// const int ENC_B_SD = 2;

// // Pines motor inferior izquierdo
// const int IN1_II = 52;
// const int IN2_II = 53;
// const int ENA_II = 5;
// const int ENC_A_II = 13;
// const int ENC_B_II = 12;

// // Pines motor inferior derecho
// const int IN1_ID = 50;
// const int IN2_ID = 49;
// const int ENB_ID = 4;
// const int ENC_A_ID = 11;
// const int ENC_B_ID = 10;

// const int pwmIzq = 250;
// const int pmwDer = 250;

// const int pwmAdelante = 255;
// const int pwmReversa = 230;

// // Valores de constante
// const int zero = 0;
  
// /* OBJECTS (SENSORS) */

// // Motor object instantiation.


// // Control de motores
// void adelante(){
//     // Motor superior izquierdo
//     digitalWrite(IN1_SI,HIGH);
//     digitalWrite(IN2_SI,LOW);
//     analogWrite(ENA_SI,190);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,LOW);
//     digitalWrite(IN2_II,HIGH);
//     analogWrite(ENA_II,190);
    
//     // Motor superior derecho
//     digitalWrite(IN1_SD,LOW);
//     digitalWrite(IN2_SD,HIGH);
//     analogWrite(ENB_SD,190);

//     // Motor inferior derecho
//     digitalWrite(IN1_ID,LOW);
//     digitalWrite(IN2_ID,HIGH);
//     analogWrite(ENB_ID,190);
//     delay(3000);
// }

// // ESTO YA QUEDA ASI FUNCIONA
// void reversa(){
//     // Motor superior izquierdo
//     digitalWrite(IN1_SI,LOW);
//     digitalWrite(IN2_SI,HIGH);
//     analogWrite(ENA_SI,250);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,HIGH);
//     digitalWrite(IN2_II,LOW);
//     analogWrite(ENA_II,250);
    
//     // Motor superior derecho,
//     digitalWrite(IN1_SD,HIGH);
//     digitalWrite(IN2_SD,LOW);
//     analogWrite(ENB_SD,250);

//     // Motor inferior derecho
//     digitalWrite(IN1_ID,HIGH);
//     digitalWrite(IN2_ID,LOW);
//     analogWrite(ENB_ID,250);

//     delay(3000);
// }

// void detener(){
//   // Motor superior izquierdo
//     digitalWrite(IN1_SI,LOW);
//     digitalWrite(IN2_SI,LOW);
//     analogWrite(ENA_SI,0);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,LOW);
//     digitalWrite(IN2_II,LOW);
//     analogWrite(ENA_II,0);
    
//     // Motor superior derecho
//     digitalWrite(IN1_SD,LOW);
//     digitalWrite(IN2_SD,LOW);
//     analogWrite(ENB_SD,0);

//     // Motor inferior derecho
//     digitalWrite(IN1_ID,LOW);
//     digitalWrite(IN2_ID,LOW);
//     analogWrite(ENB_ID,0);
//     delay(3000);
// }

// void giroDerecha(){
//     // Motor superior izquierdo
//     digitalWrite(IN1_SI,HIGH);
//     digitalWrite(IN2_SI,LOW);
//     analogWrite(ENA_SI,255);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,HIGH);
//     digitalWrite(IN2_II,LOW);
//     analogWrite(ENA_II,250);
    
//     // Motor superior derecho
//     digitalWrite(IN1_SD,HIGH);
//     digitalWrite(IN2_SD,LOW);
//     analogWrite(ENB_SD,243);


//     // Motor inferior derecho
//     digitalWrite(IN1_ID,HIGH);
//     digitalWrite(IN2_ID,LOW);
//     analogWrite(ENB_ID,248);
    
//     delay(1400);
// }

// void giroIzquierda(){
//     // Motor superior izquierdo
//     digitalWrite(IN1_SI,HIGH);
//     digitalWrite(IN2_SI,LOW);
//     analogWrite(ENA_SI,zero);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,HIGH);
//     digitalWrite(IN2_II,HIGH);
//     analogWrite(ENA_II,zero);
    
//     // Motor superior derecho
//     digitalWrite(IN1_SD,HIGH);
//     digitalWrite(IN2_SD,LOW);
//     analogWrite(ENB_SD,pmwDer);


//     // Motor inferior derecho
//     digitalWrite(IN1_ID,HIGH);
//     digitalWrite(IN2_ID,LOW);
//     analogWrite(ENB_ID,pmwDer);

//     delay(2000);
// }

// void movLateral(){
//     // Motor superior izquierdo
//     digitalWrite(IN1_SI,LOW);
//     digitalWrite(IN2_SI,HIGH);
//     analogWrite(ENA_SI,pwmIzq);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,HIGH);
//     digitalWrite(IN2_II,LOW);
//     analogWrite(ENA_II,pwmIzq);
    
//     // Motor superior derecho
//     digitalWrite(IN1_SD,LOW);
//     digitalWrite(IN2_SD,HIGH);
//     analogWrite(ENB_SD,pmwDer);

//     // Motor inferior derecho
//     digitalWrite(IN1_ID,HIGH);
//     digitalWrite(IN2_ID,LOW);
//     analogWrite(ENB_ID,pmwDer);
    
//     delay(2000);
// }

// /* ARDUINO SETUP */

// void setup() {
//     Serial.begin(9600);

//     pinMode(IN1_SI,OUTPUT);
//     pinMode(IN2_SI,OUTPUT);
//     pinMode(ENA_SI,OUTPUT);

//     pinMode(IN1_SD,OUTPUT);
//     pinMode(IN2_SD,OUTPUT);
//     pinMode(ENB_SD,OUTPUT);

//     pinMode(IN1_II,OUTPUT);
//     pinMode(IN2_II,OUTPUT);
//     pinMode(ENA_II,OUTPUT);

//     pinMode(IN1_ID,OUTPUT);
//     pinMode(IN2_ID,OUTPUT);
//     pinMode(ENB_ID,OUTPUT);

//     // Encoders
//     // Encoder Superior Izquierdo
//     pinMode(ENC_A_SI, INPUT);
//     pinMode(ENC_B_SI, INPUT);
    
//     // Encoder Superior Derecho
//     pinMode(ENC_A_SD, INPUT);
//     pinMode(ENC_B_SD, INPUT);
    
//     // Encoder Inferior Izquierdo
//     pinMode(ENC_A_II, INPUT);
//     pinMode(ENC_B_II, INPUT);

//     // Encoder Inferior Derecho
//     pinMode(ENC_A_ID, INPUT);
//     pinMode(ENC_B_ID, INPUT);
// }

// /* ARDUINO LOOP */

// void loop() {
//     adelante();
//     detener();
//     reversa();
//     detener();
    
//     // giroDerecha();
//     // detener();

//     //giroIzquierda();
//     //movLateral();
    
// }

/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Simple Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;

void setup() 
{
  Serial.begin(115200);

  // Initialize MPU6050
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // If you want, you can set gyroscope offsets
  // mpu.setGyroOffsetX(155);
  // mpu.setGyroOffsetY(15);
  // mpu.setGyroOffsetZ(15);
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  
  // Check settings
  checkSettings();
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:      ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Gyroscope:         ");
  switch(mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  } 
  
  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());
  
  Serial.println();
}

void loop()
{
  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();

  Serial.print(" Xraw = ");
  Serial.print(rawGyro.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawGyro.YAxis);
  Serial.print(" Zraw = ");
  Serial.println(rawGyro.ZAxis);

  Serial.print(" Xnorm = ");
  Serial.print(normGyro.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normGyro.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normGyro.ZAxis);
  
  delay(10);
}