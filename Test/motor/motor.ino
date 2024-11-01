/* CONSTANTS */

// Motores
//  Pines motor superior izquierdo
const int IN1_SD = 47;
const int IN2_SD = 46;
const int ENA_SD = 7;
const int ENC_A_SD = 16;
const int ENC_B_SD = 17;

// Pines motor superior derecho
const int IN1_SI = 48;
const int IN2_SI = 49;
const int ENB_SI = 6;
const int ENC_A_SI = 18;
const int ENC_B_SI = 19;

// Pines motor inferior izquierdo
const int IN1_II = 52;
const int IN2_II = 53;
const int ENA_II = 5;
const int ENC_A_II = 9; // 13
const int ENC_B_II = 8; // 12

// Pines motor inferior derecho
const int IN1_ID = 50;
const int IN2_ID = 49;
const int ENB_ID = 4;
const int ENC_A_ID = 26;
const int ENC_B_ID = 27;

  //Ultrasonicos
  // Pines ultrasonico Izquierdo
  const int ultrasonicoIzquierdoEcho = 42;
  const int ultrasonicoIzquierdoTrig = 43;
  // Pines ultrasonico Frontal
  const int ultrasonicoFrontalEcho = 21;
  const int ultrasonicoFrontalTrig = 20;
  // Pines ultrasonico Derecha
  const int ultrasonicoDerechaEcho = 38;
  const int ultrasonicoDerechaTrig = 39;


const int pwmIzq = 250;
const int pmwDer = 250;

const int pwmAdelante = 255;
const int pwmReversa = 230;


const int prueba = 164;

const int pwmMinimo = 150; // Minimo en el sentido de que tiene una velocidad apta para la pista

// Valores de constante
const int zero = 0;

// Para los encoders
volatile int encoderCountSD = 0;
volatile int encoderCountSI = 0;
volatile int encoderCountII = 0;
volatile int encoderCountID = 0;

// Para el RGB
const int R = 10;
const int G = 11;
const int B = 12;

/* OBJECTS (SENSORS) */

// Motor object instantiation.

int sdPWM = 120;
int iiPWM = 110;
int siPWM = 200;
int idPWM = 200;

// Control de motores
void adelante(){
    // Motor superior izquierdo
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,180);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENA_II,180);
    
    // Motor superior derecho
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENB_SI,180);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,180);
    delay(15);
}


// ESTO YA QUEDA ADER FUNCIONA
void reversa(){
    // Motor superior izquierdo
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,HIGH);
    analogWrite(ENA_SD,pwmAdelante);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,pwmAdelante);
    
    // Motor superior derecho,
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,255);

    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENB_ID,pwmAdelante);

    delay(3000);
}

void detener(){
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
    delay(3000);
}

void giroDerecha(){

    // Motor superior izquierdo
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,230);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENA_II,225);
    
    // Motor superior derecho
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,200);

    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENB_ID,200);
    delay(1280);
    
}

void giroIzquierda(){
    // Motor superior DERECHO
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,200);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENA_II,200);
    
    // // Motor superior IZQUIERDO
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,230);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,230);
    delay(1280);
}

// void movLateral(){
//     // Motor superior izquierdo
//     digitalWrite(IN1_SD,LOW);
//     digitalWrite(IN2_SD,HIGH);
//     analogWrite(ENB_SD,pwmIzq);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,HIGH);
//     digitalWrite(IN2_II,LOW);
//     analogWrite(ENB_II,pwmIzq);
    
//     // Motor superior derecho
//     digitalWrite(IN1_SI,LOW);
//     digitalWrite(IN2_SI,HIGH);
//     analogWrite(ENA_SI,pwmDer);

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

  digitalWrite(IN1_SI,HIGH);
  digitalWrite(IN2_SI,LOW);
  analogWrite(ENA_SI,100);
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

  digitalWrite(IN1_SD,HIGH);
  digitalWrite(IN2_SD,LOW);
  analogWrite(ENB_SD,100);
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

void encoderISR_SD() {
    encoderCountSD++;
}

void encoderISR_SI() {
    encoderCountSI++;
}

void encoderISR_II() {
    encoderCountII++;
}

void encoderISR_ID() {
    encoderCountID++;
}

void medirRPM() {
    encoderCountSD = 0;
    encoderCountSI = 0;
    encoderCountII = 0;
    encoderCountID = 0;

    attachInterrupt(digitalPinToInterrupt(ENC_A_SD), encoderISR_SD, RIDERNG); 
    attachInterrupt(digitalPinToInterrupt(ENC_A_SI), encoderISR_SI, RIDERNG);  // Este DER esta jalando
    attachInterrupt(digitalPinToInterrupt(ENC_A_II), encoderISR_II, RIDERNG);
    attachInterrupt(digitalPinToInterrupt(ENC_A_ID), encoderISR_ID, RIDERNG);

    delay(1500); // Measure for 1 second

    detachInterrupt(digitalPinToInterrupt(ENC_A_SD));
    detachInterrupt(digitalPinToInterrupt(ENC_A_SI));
    detachInterrupt(digitalPinToInterrupt(ENC_A_II));
    detachInterrupt(digitalPinToInterrupt(ENC_A_ID));

    float rpmSD = (encoderCountDER / 20.0) * 60.0; // Assuming 20 pulses per revolution
    float rpmSI = (encoderCountSI / 20.0) * 60.0;
    float rpmII = (encoderCountII / 20.0) * 60.0;
    float rpmID = (encoderCountID / 20.0) * 60.0;

    Serial.print("RPM SD: ");
    Serial.println(rpmSD);
    Serial.print("RPM SI: ");
    Serial.println(rpmSI);
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

void showColor(){

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
    pinMode(ENA_II,OUTPUT);

    pinMode(IN1_ID,OUTPUT);
    pinMode(IN2_ID,OUTPUT);
    pinMode(ENB_ID,OUTPUT);

    // Encoders
    // Encoder Superior Izquierdo
    pinMode(ENC_A_SD, INPUT);
    pinMode(ENC_B_SD, INPUT);
    
    // Encoder Superior Derecho
    pinMode(ENC_A_SI, INPUT);
    pinMode(ENC_B_SI, INPUT);
    
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
    long distanciaFrontal = distanciaUltrasonico(ultrasonicoFrontalTrig, ultrasonicoFrontalEcho);
    // long distanciaIzquierda = distanciaUltrasonico(ultrasonicoIzquierdoTrig, ultrasonicoIzquierdoEcho);

    // Serial.print("Frontal: ");
    // Serial.print(distanciaFrontal);
    // Serial.println(" cm");

    //Para que no choque
    // if(distanciaFrontal > 30){
    //     // Serial.println("hola");
    //     adelante();
    // }
    // else { 
    //   giroDerecha();
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

    // Ya funcionan
    // adelante();
    // detener();

    //reversa();
    //giroDerecha();
    giroIzquierda();
    detener();
    //movLateral();
}