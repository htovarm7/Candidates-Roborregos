// Ultrasonicos
const int ultrasonicoFrontalEcho = 37;
const int ultrasonicoFrontalTrig = 38;

// Ultrasonico Derecha
const int ultrasonicoDerechaEcho = 52;
const int ultrasonicoDerechaTrig = 53;

// Ultrasonico Izquierda
const int ultrasonicoIzquierdaEcho = 22;
const int ultrasonicoIzquierdaTrig = 24;


void setup(){
    // Ultrasonico Frontal
    pinMode(ultrasonicoFrontalEcho, INPUT);
    pinMode(ultrasonicoFrontalTrig, OUTPUT);

    // Ultrasonico Derecha
    pinMode(ultrasonicoDerechaEcho, INPUT);
    pinMode(ultrasonicoDerechaTrig, OUTPUT);
    
    // Ultrasonico Izquierda
    pinMode(ultrasonicoIzquierdaEcho, INPUT);
    pinMode(ultrasonicoIzquierdaTrig, OUTPUT);
}

void loop(){
    long distanciaFrontal = distanciaUltrasonico(ultrasonicoFrontalTrig, ultrasonicoFrontalEcho);
    long distanciaDerecha = distanciaUltrasonico(ultrasonicoDerechaTrig, ultrasonicoDerechaEcho);
    long distanciaIzquierda = distanciaUltrasonico(ultrasonicoIzquierdaTrig, ultrasonicoIzquierdaEcho);

    // Distancia de los ultrasonicos
    Serial.print("Frontal: ");
    Serial.print(distanciaFrontal);
    Serial.print(" cm, Derecha: ");
    Serial.print(distanciaDerecha);
    Serial.print(" cm, Izquierda: ");
    Serial.print(distanciaIzquierda);
    Serial.println(" cm");
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