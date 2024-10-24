const int ENC_A = 6;
const int ENC_B = 5;

// Motor amarillo
const int IN1 = 2;
const int IN2 = 4;
const int ENA = 3;

volatile int pulseCount = 0;
unsigned long lastTime = 0;
const unsigned long interval = 1000; // Intervalo de tiempo en milisegundos

// Implementar ultrasonico para que al momento de detectar 
// algo se pare el motor si esta libre el camino acelerar
void setup() {
  Serial.begin(9600);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), countPulse, RISING);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;
    float rpm = (pulseCount / 20.0) * 60.0; // Suponiendo 20 pulsos por revolución
    Serial.print("RPM: ");
    Serial.println(rpm);
    pulseCount = 0;
  }
  motor();
}

void countPulse() {
  pulseCount++;
}

void motor() {
  Serial.println("Motor detenido");
  digitalWrite(IN1, LOW); // Detener motor
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);    // Sin velocidad
  delay(2000);

// velocidad mínimo sin peso: 30
  Serial.println("Motor a velocidad media");
  digitalWrite(IN1, HIGH); // Sentido del motor
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 30);   // Velocidad media (50%)
  delay(2000);

  // Serial.println("Motor a velocidad máxima");
  // digitalWrite(IN1, HIGH); // Sentido del motor
  // digitalWrite(IN2, LOW);
  // analogWrite(ENA, 154);   // Velocidad máxima (100%)
  // delay(4000);

  // Serial.println("Motor detenido");
  // digitalWrite(IN1, LOW); // Detener motor
  // digitalWrite(IN2, LOW);
  // analogWrite(ENA, 0);    // Sin velocidad
  // delay(2000);

// Cambio de dirección
//  Serial.println("Motor cambiando de dirección");
//  digitalWrite(IN1, LOW);  // Cambiar sentido del motor
//  digitalWrite(IN2, HIGH);
//  analogWrite(ENA, 60);   // Mantener velocidad máxima
//  delay(2000);
}

// Ultrasonicos
const int ultrasonicoFrontalEcho = 8;
const int ultrasonicoFrontalTrig = 9;

void setup(){
    // Ultrasonico Frontal
    pinMode(ultrasonicoFrontalEcho, INPUT);
    pinMode(ultrasonicoFrontalTrig, OUTPUT);
}

void loop(){
    long distanciaFrontal = distanciaUltrasonico(ultrasonicoFrontalTrig, ultrasonicoFrontalEcho);

    // Distancia de los ultrasonicos
    Serial.print("Frontal: ");
    Serial.print(distanciaFrontal);
    Serial.print(" cm, Derecha: ");
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