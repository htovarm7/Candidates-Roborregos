const int ENC_A = 6;
const int ENC_B = 5;
const int IN1 = 2;
const int IN2 = 3;
const int ENA = 4;

volatile int pulseCount = 0;
unsigned long lastTime = 0;
const unsigned long interval = 1000; // Intervalo de tiempo en milisegundos

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
}

void countPulse() {
  pulseCount++;
}
