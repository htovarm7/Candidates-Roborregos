// Motor
const int ENC_A = 6;
const int ENC_B = 5;
const int IN1 = 2;
const int IN2 = 4;
const int ENA = 3;

// Ultrasonicos
const int ultrasonicoFrontalEcho = 8;
const int ultrasonicoFrontalTrig = 9;

volatile int pulseCount = 0;
unsigned long lastTime = 0;
const unsigned long interval = 1000; // Intervalo de tiempo en milisegundos

// Implementar ultrasonico para que al momento de detectar 
// algo se pare el motor si esta libre el camino acelerar
void setup() {
  Serial.begin(9600);

  // Motor
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), countPulse, RISING);

  // Ultrasonico Frontal
  pinMode(ultrasonicoFrontalEcho, INPUT);
  pinMode(ultrasonicoFrontalTrig, OUTPUT);
}

void loop() {
  // Para checar los rpm
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;
    float rpm = (pulseCount / 20.0) * 60.0; // Suponiendo 20 pulsos por revolución
    Serial.print("RPM: ");
    Serial.println(rpm);
    pulseCount = 0;
  }
  motor();
  long distanciaFrontal = distanciaUltrasonico(ultrasonicoFrontalTrig, ultrasonicoFrontalEcho);
    
  // Distancia de los ultrasonicos
  Serial.print("Frontal: ");
  Serial.print(distanciaFrontal);
  Serial.print(" cm, Derecha: ");

  if(distanciaFrontal < 10){
    detener();
  }else{
    avanzar();
  }
}

void countPulse() {
  pulseCount++;
}

// Funciones de los motores
void detener(){
   Serial.println("Motor detenido");
   digitalWrite(IN1, LOW);  // Detener motor
   digitalWrite(IN2, LOW);
   analogWrite(ENA, 0);     // Sin velocidad
}

void avanzar(){
  Serial.println("Motor a velocidad media");
  digitalWrite(IN1, HIGH); // Sentido del motor
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150);   // Velocidad media (50%)
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