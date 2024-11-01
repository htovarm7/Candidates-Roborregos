/* CONSTANTS */

// Motores
//  Pines motor superior izquierdo
const int IN1_SI = 48; // 48
const int IN2_SI = 23; //23
const int ENB_SI = 6; // 6
const int ENC_A_SI = 18; 
const int ENC_B_SI = 19;

// Pines motor superior derecho
const int IN1_SD = 47; // 47
const int IN2_SD = 46; // 46
const int ENA_SD = 7; // 7
const int ENC_A_SD = 17; 
const int ENC_B_SD = 18;

// Pines motor inferior izquierdo
const int IN1_II = 52;
const int IN2_II = 53;
const int ENB_II = 5;
const int ENC_A_II = 9;
const int ENC_B_II = 8;

// Pines motor inferior derecho
const int IN1_ID = 50;
const int IN2_ID = 49;
const int ENA_ID = 4;
const int ENC_A_ID = 24;
const int ENC_B_ID = 23;

// Pines ultrasonico Frontal
const int ultrasonicoFrontalEcho = 21;
const int ultrasonicoFrontalTrig = 20;

// Pines ultrasonico Izquierdo
const int ultrasonicoIzquierdoEcho = 42;
const int ultrasonicoIzquierdoTrig = 43;


const int pwmIzq = 250;
const int pwmDer = 250;

const int prueba = 164;

const int pwmMinimo = 150; // Minimo en el sentido de que tiene una velocidad apta para la pista

// Valores de constante
const int zero = 0;

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
void adelante(){

    // 150 bits como valor predeterminado

    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,prueba); // Es el ultimo en pararse, 120
    
    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENB_II,prueba); // Se para a la misma vez, 110
    
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,prueba); // Se para a la misma vez, 200

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENA_ID,prueba); // Se para a la misma vez, 200
}

// ESTO YA QUEDA ASI FUNCIONA
void reversa(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENB_SI,zero);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENB_II,zero);
    
    // Motor superior derecho,
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,zero);

    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENA_ID,zero);

    delay(3000);
}

void detener(){
  int siStart = siPWM;
  int sdStart = sdPWM;
  int iiStart = iiPWM;
  int idStart = idPWM;
  float stopTime = 2.6;

  float siDiff = siStart / stopTime;
  float sdDiff = sdStart / stopTime;
  float iiDiff = iiStart / stopTime;
  float idDiff = idStart / stopTime;

  //for (float t = 0; t <= stopTime; t += 0.1) {
      // Motor superior izquierdo
      digitalWrite(IN1_SI,LOW);
      digitalWrite(IN2_SI,LOW);
      analogWrite(ENB_SI, 0);
      // analogWrite(ENB_SI, siPWM - t * siDiff);

      // Motor inferior izquierdo
      digitalWrite(IN1_II,LOW);
      digitalWrite(IN2_II,LOW);
      analogWrite(ENB_II, 0);
      // analogWrite(ENB_II, sdPWM - t * sdDiff);
      
      // Motor superior derecho
      digitalWrite(IN1_SD,LOW);
      digitalWrite(IN2_SD,LOW);
      analogWrite(ENA_SD, 0);
      // analogWrite(ENA_SD, iiPWM - t * iiDiff);

      // Motor inferior derecho
      digitalWrite(IN1_ID,LOW);
      digitalWrite(IN2_ID,LOW);
      analogWrite(ENA_ID, 0);
      // analogWrite(ENA_ID, idPWM - t * idDiff);

      // Serial.print("Time: ");
      // Serial.println(t);
      // Serial.println(siPWM - t * siDiff);
      // Serial.println(sdPWM - t * sdDiff);
      // Serial.println(iiPWM - t * iiDiff);
      // Serial.println(idPWM - t * idDiff);
  //}
  delay(50);
}

void giroDerecha(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,70);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENB_II,0);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,100);


    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENA_ID,75);
    
    delay(1400);
}

void giroIzquierda(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,zero);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENB_II,zero);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,pwmDer);


    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENA_ID,pwmDer);

    delay(2000);
}

void movLateral(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENB_SI,pwmIzq);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENB_II,pwmIzq);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,HIGH);
    analogWrite(ENA_SD,pwmDer);

    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENA_ID,pwmDer);
    
    delay(2000);
}

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

void showColor(){

}

/* ARDUINO SETUP */

void setup() {
    Serial.begin(9600);

    pinMode(IN1_SI,OUTPUT);
    pinMode(IN2_SI,OUTPUT);
    pinMode(ENB_SI,OUTPUT);

    pinMode(IN1_SD,OUTPUT);
    pinMode(IN2_SD,OUTPUT);
    pinMode(ENA_SD,OUTPUT);

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

    // Para que no choque
    // if(distanciaFrontal > 30){
    //     // Serial.println("hola");
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

    // Ya funcionan
    // adelante();
    // detener();

    //reversa();
    giroDerecha();
    //giroIzquierda();
    //movLateral();
}