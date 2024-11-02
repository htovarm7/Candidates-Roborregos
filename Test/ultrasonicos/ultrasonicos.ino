// Pines ultrasonico Izquierdo
const int leftEcho = 42;
const int leftTrig = 43;

// Pines ultrasonico Frontal
const int frontEcho = 15;
const int frontTrig = 14;

// Pines ultrasonico Derecha
const int rightEcho = 2;
const int rightTrig = 3;

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

void setup(){
  Serial.begin(9600);

    // Ultrasonico Izquierdo
    pinMode(leftEcho, INPUT);
    pinMode(leftTrig, OUTPUT);

    // Ultrasonico Frontal
    pinMode(frontEcho, INPUT);
    pinMode(frontTrig, OUTPUT);

    // Ultrasonico Derecha
    pinMode(rightEcho, INPUT);
    pinMode(rightTrig, OUTPUT);
}

void loop(){
    long distanciaIzquierdo = distanciaUltrasonico(leftTrig, leftEcho);

    long distanciaFrontal = distanciaUltrasonico(frontTrig, frontEcho);
    
    long distanciaDerecha = distanciaUltrasonico(rightTrig, rightEcho);

    // Distancia de los ultrasonicos
    //Izquierdo
    Serial.print("Izquierdo: ");
    Serial.print(distanciaIzquierdo);
    Serial.println(" cm");
    //Frontal
    Serial.print("Frontal: ");
    Serial.print(distanciaFrontal);
    Serial.println(" cm");
    //Derecha
    Serial.print("Derecha: ");
    Serial.print(distanciaDerecha);
    Serial.println(" cm");
    delay(1000);
}
