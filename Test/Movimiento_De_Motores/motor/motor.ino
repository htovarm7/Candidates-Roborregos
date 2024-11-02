/* CONSTANTS */

// Motores
//  Pines motor superior izquierdo
const int IN1_SI = 43;
const int IN2_SI = 42;
const int ENB_SI = 4;
const int ENC_A_SI = 29;
const int ENC_B_SI = 28;

// Pines motor inferior izquierdo
const int IN1_II = 41;
const int IN2_II = 40;
const int ENB_II = 3;
const int ENC_A_II = 27;
const int ENC_B_II = 26;

// Pines motor superior derecho
const int IN1_SD = 45;
const int IN2_SD = 44;
const int ENA_SD = 5;
const int ENC_A_SD = 25;
const int ENC_B_SD = 24;

// Pines motor inferior derecho
const int IN1_ID = 39;
const int IN2_ID = 38;
const int ENA_ID = 2;
const int ENC_A_ID = 31;
const int ENC_B_ID = 30;


//Ultrasonicos
// Pines ultrasonico Izquierdo
const int ultrasonicoIzquierdoEcho = 33;
const int ultrasonicoIzquierdoTrig = 32;
// Pines ultrasonico Frontal
const int ultrasonicoFrontalEcho = 35;
const int ultrasonicoFrontalTrig = 34;
// Pines ultrasonico Derecha
const int ultrasonicoDerechaEcho = 37;
const int ultrasonicoDerechaTrig = 36;

// Para las velocidades que ocupe el motor
const int pwmIzq = 250;
const int pmwDer = 250;

const int pwmAdelante = 255;
const int pwmReversa = 230;

const int prueba = 164;

const int pwmMinimo = 150; // Minimo en el sentido de que tiene una velocidad apta para la pista

// Para los encoders
volatile int encoderCountSI = 0;
volatile int encoderCountSD = 0;
volatile int encoderCountII = 0;
volatile int encoderCountID = 0;

// Para el RGB
const int R = 10;
const int G = 11;
const int B = 12;

/* OBJECTS (SENSORS) */

// Motor object instantiation.

int siPWM = 120;
int iiPWM = 110;
int sdPWM = 200;
int idPWM = 200;

// Control de motores
void avanzar(){
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,200);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENA_ID,200);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENB_II,200);
    
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,200);

    delay(1000);
}


void reversa(){
      // Motor superior derecho
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,HIGH);
    analogWrite(ENA_SD,160);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENB_II,200);
    
    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENA_ID,160);

    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENB_SI,200);

    //delay(1000);
    delay(300);
}

void detener(){
      // Motor superior izquierdo
      digitalWrite(IN1_SD,LOW);
      digitalWrite(IN2_SD,LOW);
      analogWrite(ENA_SD,0);

      // Motor inferior izquierdo
      digitalWrite(IN1_II,LOW);
      digitalWrite(IN2_II,LOW);
      analogWrite(ENB_II,0);
      
      // Motor superior derecho
      digitalWrite(IN1_SI,LOW);
      digitalWrite(IN2_SI,LOW);
      analogWrite(ENB_SI,0);


      // Motor inferior derecho
      digitalWrite(IN1_ID,LOW);
      digitalWrite(IN2_ID,LOW);
      analogWrite(ENA_ID,0);
      delay(2000);
}

void giroDerecha(){

    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,70);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENA_ID,70);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENB_II,255);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,255);

    //delay();
    delay(500);
    
}

void giroIzquierda(){

    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,220);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENB_II,255);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENB_SI,220);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENA_ID,255);

    delay(400);
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

// Funciones para probar cada motor

/*
void motorSuperiorDerecho(){
  delay(1500);
  Serial.println("Motor Superior Derecho");
  delay(1500);

  digitalWrite(IN1_SD,HIGH);
  digitalWrite(IN2_SD,LOW);
  analogWrite(ENA_SD,100);
  delay(1500);

}

void motorInferiorDerecho(){
  delay(1500);
  Serial.println("Motor Inferior Derecho");
  delay(1500);

  digitalWrite(IN1_ID,HIGH);
  digitalWrite(IN2_ID,LOW);
  analogWrite(ENA_ID,100);
  delay(1500);
}

void motorSuperiorIzquierdo(){
  delay(1500);
  Serial.println("Motor Superior Izquierdo");
  delay(1500);

  digitalWrite(IN1_SI,HIGH);
  digitalWrite(IN2_SI,LOW);
  analogWrite(ENB_SI,100);
  delay(1500);
}

void motorInferiorIzquierdo(){
  delay(1500);
  Serial.println("Motor Inferior Izquierdo");
  delay(1500);

  digitalWrite(IN1_II,HIGH);
  digitalWrite(IN2_II,LOW);
  analogWrite(ENB_II,100);
  delay(1500);

}
*/

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

void medirRPM() {
    encoderCountSI = 0;
    encoderCountSD = 0;
    encoderCountII = 0;
    encoderCountID = 0;

    attachInterrupt(digitalPinToInterrupt(ENC_A_SI), encoderISR_SI, RISING); 
    attachInterrupt(digitalPinToInterrupt(ENC_A_SD), encoderISR_SD, RISING);  // Este si esta jalando
    attachInterrupt(digitalPinToInterrupt(ENC_A_II), encoderISR_II, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_A_ID), encoderISR_ID, RISING);

    delay(1500); // Measure for 1 second

    detachInterrupt(digitalPinToInterrupt(ENC_A_SI));
    detachInterrupt(digitalPinToInterrupt(ENC_A_SD));
    detachInterrupt(digitalPinToInterrupt(ENC_A_II));
    detachInterrupt(digitalPinToInterrupt(ENC_A_ID));

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
}

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

/* ARDUINO SETUP */

void setup() {
    Serial.begin(9600);

    pinMode(IN1_SD,OUTPUT);
    pinMode(IN2_SD,OUTPUT);
    pinMode(ENA_SD,OUTPUT);

    pinMode(IN1_SI,OUTPUT);
    pinMode(IN2_SI,OUTPUT);
    pinMode(ENB_SI,OUTPUT);

    pinMode(IN1_II,OUTPUT);
    pinMode(IN2_II,OUTPUT);
    pinMode(ENB_II,OUTPUT);

    pinMode(IN1_ID,OUTPUT);
    pinMode(IN2_ID,OUTPUT);
    pinMode(ENA_ID,OUTPUT);

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
    // Ultrasonico Frontal
    pinMode(ultrasonicoFrontalEcho, INPUT);
    pinMode(ultrasonicoFrontalTrig, OUTPUT);
    Serial.begin(9600);
}

/* ARDUINO LOOP */

void loop() {

    //medirRPM();
    // long distanciaFrontal = distanciaUltrasonico(ultrasonicoFrontalTrig, ultrasonicoFrontalEcho);
    // long distanciaIzquierda = distanciaUltrasonico(ultrasonicoIzquierdoTrig, ultrasonicoIzquierdoEcho);

    // Serial.print("Frontal: ");
    // Serial.print(distanciaFrontal);
    // Serial.println(" cm");
    // delay(50);

    // //Para que no choque
    // if(distanciaFrontal > 23){
    //     adelante();
    // }
    // else { 
    //   detener();
    // }

    // Serial.print("Izquierda: ");
    // Serial.print(distanciaIzquierda);
    // Serial.println(" cm");
    
    /*
    // Funciones para probar cada motor
    motorSuperiorDerecho();
    motorInferiorDerecho();
    motorSuperiorIzquierdo();
    motorInferiorIzquierdo();
    */

    // // Ya funcionan
    // adelante();
    // if (distanciaFrontal < 10) {
    //     giroDerecha();
    // }
    avanzar();
    medirRPM();
    

    //movLateral();
}