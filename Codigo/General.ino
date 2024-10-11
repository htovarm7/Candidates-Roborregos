// Ultrasonicos
const int ultrasonicoFrontalEcho = 37;
const int ultrasonicoFrontalTrig = 38;

// Ultrasonico Derecha
const int ultrasonicoDerechaEcho = 52;
const int ultrasonicoDerechaTrig = 53;

// Ultrasonico Izquierda
const int ultrasonicoIzquierdaEcho = 22;
const int ultrasonicoIzquierdaTrig = 24;

// Puente H (motores a la izquierda)
// Motor superior izquierdo
const int ENA_Izquierdo = 5;
const int INA1_Izquierdo = 28;
const int INA2_Izquierdo = 27;

// Motor inferior izquierdo
const int ENB_Izquierdo = 6;
const int INB1_Izquierdo = 26;
const int INB2_Izquierdo = 25;

// Puente H (motores a la derecha)
// Motor superior derecho
const int ENA_Derecho = 7;
const int INA1_Derecho = 32;
const int INA2_Derecho = 31;

// Motor inferior derecho
const int ENB_Derecho = 8;
const int INB1_Derecho = 30;
const int INB2_Derecho = 29;

// LED RGB
const int R = 9;
const int B = 10;
const int G = 11;

// Buzzer
const int buzzer = 51;

// Motores con encoders
const int encoderSuperiorIzquierdo_A = 2;
const int encoderSuperiorIzquierdo_B = 3;
const int encoderSuperiorDerecho_A = 40;
const int encoderSuperiorDerecho_B = 41;
const int encoderInferiorIzquierdo_A = 19;
const int encoderInferiorIzquierdo_B = 18;
const int encoderInferiorDerecho_A = 43;
const int encoderInferiorDerecho_B = 42;

// Giroscopio
const int giroSCL = A5; 
const int giroSDA = A4; 

// Sensor de color
const int colorSDA = A4; 
const int colorSCL = A5; 


// Sensores de línea
const int sensorLineaD8 = 50;
const int sensorLineaD7 = 49;
const int sensorLineaD6 = 48;
const int sensorLineaD5 = 47;
const int sensorLineaD4 = 46;
const int sensorLineaD3 = 45;
const int sensorLineaD2 = 44;
const int sensorLineaD1 = 43;

void setup() {
  
    // Ultrasonico Frontal
    pinMode(ultrasonicoFrontalEcho, INPUT);
    pinMode(ultrasonicoFrontalTrig, OUTPUT);

    // Ultrasonico Derecha
    pinMode(ultrasonicoDerechaEcho, INPUT);
    pinMode(ultrasonicoDerechaTrig, OUTPUT);
    
    // Ultrasonico Izquierda
    pinMode(ultrasonicoIzquierdaEcho, INPUT);
    pinMode(ultrasonicoIzquierdaTrig, OUTPUT);
    
    // Motores Lado Izquierdo
    pinMode(ENA_Izquierdo, OUTPUT);
    pinMode(INA1_Izquierdo, OUTPUT);
    pinMode(INA2_Izquierdo, OUTPUT);
    pinMode(INB1_Izquierdo, OUTPUT);
    pinMode(INB2_Izquierdo, OUTPUT);
    pinMode(ENB_Izquierdo, OUTPUT);

    // Motores Lado Derecho
    pinMode(ENA_Derecho, OUTPUT);
    pinMode(INA1_Derecho, OUTPUT);
    pinMode(INA2_Derecho, OUTPUT);
    pinMode(INB1_Derecho, OUTPUT);
    pinMode(INB2_Derecho, OUTPUT);
    pinMode(ENB_Derecho, OUTPUT);
    
    // Actuadores
    pinMode(R, OUTPUT);
    pinMode(B, OUTPUT);
    pinMode(G, OUTPUT);
    pinMode(buzzer, OUTPUT);
    
    // Motores con encoders
    // Encoder Superior Izquierdo
    pinMode(encoderSuperiorIzquierdo_A, INPUT);
    pinMode(encoderSuperiorIzquierdo_B, INPUT);
    
    // Encoder Superior Derecho
    pinMode(encoderSuperiorDerecho_A, INPUT);
    pinMode(encoderSuperiorDerecho_B, INPUT);
    
    // Encoder Inferior Izquierdo
    pinMode(encoderInferiorIzquierdo_A, INPUT);
    pinMode(encoderInferiorIzquierdo_B, INPUT);

        // Encoder Inferior Derecho
    pinMode(encoderInferiorDerecho_A, INPUT);
    pinMode(encoderInferiorDerecho_B, INPUT);
    
    // Sensor de Linea
    pinMode(sensorLineaD8, INPUT);
    pinMode(sensorLineaD7, INPUT);
    pinMode(sensorLineaD6, INPUT);
    pinMode(sensorLineaD5, INPUT);
    pinMode(sensorLineaD4, INPUT);
    pinMode(sensorLineaD3, INPUT);
    pinMode(sensorLineaD2, INPUT);
    pinMode(sensorLineaD1, INPUT);
    
}

void loop() {

    long distanceFrontal = distanciaUltrasonico(ultrasonicoFrontalTrig, ultrasonicoFrontalEcho);
    long distanceDerecha = distanciaUltrasonico(ultrasonicoDerechaTrig, ultrasonicoDerechaEcho);
    long distanceIzquierda = distanciaUltrasonico(ultrasonicoIzquierdaTrig, ultrasonicoIzquierdaEcho);

    Serial.print("Frontal: ");
    Serial.print(distanceFrontal);
    Serial.print(" cm, Derecha: ");
    Serial.print(distanceDerecha);
    Serial.print(" cm, Izquierda: ");
    Serial.print(distanceIzquierda);
    Serial.println(" cm");

    delay(500);


}

void adelante(){

}

void atras(){

}

void giroDerecha(){

}

void giroIzquierda(){

}

void detener(){

}

void encenderBuzzer(){

}

void apagarBuzzer(){

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