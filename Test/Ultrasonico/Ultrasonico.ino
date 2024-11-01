  // Pines ultrasonico Izquierdo
  const int ultrasonicoIzquierdoEcho = 42;
  const int ultrasonicoIzquierdoTrig = 43;
  // Pines ultrasonico Frontal
  const int ultrasonicoFrontalEcho = 12;
  const int ultrasonicoFrontalTrig = 13;
  // Pines ultrasonico Derecha
  const int ultrasonicoDerechaEcho = 38;
  const int ultrasonicoDerechaTrig = 39;
  void setup() {
  // Ultrasonico Izquierdo
  pinMode(ultrasonicoIzquierdoEcho, INPUT);
  pinMode(ultrasonicoIzquierdoTrig, OUTPUT);
  // Ultrasonico Frontal
  pinMode(ultrasonicoFrontalEcho, INPUT);
  pinMode(ultrasonicoFrontalTrig, OUTPUT);
  // Ultrasonico Derecha
  pinMode(ultrasonicoDerechaEcho, INPUT);
  pinMode(ultrasonicoDerechaTrig, OUTPUT);
  Serial.begin(9600);

  }

  void loop() {
  //long distanciaIzquierdo = distanciaUltrasonico(ultrasonicoIzquierdoTrig, ultrasonicoIzquierdoEcho);

  long distanciaFrontal = distanciaUltrasonico(ultrasonicoFrontalTrig, ultrasonicoFrontalEcho);
  
  //long distanciaDerecha = distanciaUltrasonico(ultrasonicoDerechaTrig, ultrasonicoDerechaEcho);

  // Distancia de los ultrasonicos
  //Izquierdo
  // Serial.print("Izquierdo: ");
  // Serial.print(distanciaIzquierdo);
  // Serial.println(" cm");
  //Frontal
  Serial.print("Frontal: ");
  Serial.print(distanciaFrontal);
  Serial.println(" cm");
  //Derecha
  // Serial.print("Derecha: ");
  // Serial.print(distanciaDerecha);
  // Serial.println(" cm");
  delay(999);
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
