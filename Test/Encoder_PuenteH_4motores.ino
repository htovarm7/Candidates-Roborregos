// Pines para los encoders de cada motor

// Enconders de la parte superior
// Encoder superior izquierdo
const int ENC_SI_A = 2;
const int ENC_SI_B = 3;

// Enconder superior derecho
const int ENC_SD_A = 39;
const int ENC_SD_B = 40;

// Encoders de la parte inferior
// Enconder inferior izquierdo
const int ENC_II_A = 19;
const int ENC_II_B = 18;

// Enconder inferior derecho
const int ENC_ID_A = 42;
const int ENC_ID_B = 41;

// Motores superiores
//Pines motor superior izquierdo
const int IN1_SI = 2;
const int IN2_SI = 4;
const int ENA_SI = 3;

// Pines motor superior derecho
const int IN1_SD = 13;
const int IN2_SD = 14;
const int ENA_SD = 2;

// Motores inferiores
// Pines motor inferior izquierdo
const int IN1_II = 52;
const int IN2_II = 53;
const int ENA_II = 5;

// Pines motor inferior derecho
const int IN1_ID = 50;
const int IN2_ID = 49;
const int ENA_ID = 4;

// Para el encoder
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
volatile int pulseCount3 = 0;
volatile int pulseCount4 = 0;

// Variables para el control de tiempo
unsigned long lastTime = 0;
const unsigned long interval = 1000; // Intervalo de tiempo en milisegundos

void setup() {
  Serial.begin(9600);
  
  // Configurar pines de encoder como entrada
  pinMode(ENC_A1, INPUT);
  pinMode(ENC_B1, INPUT);
  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B2, INPUT);
  pinMode(ENC_A3, INPUT);
  pinMode(ENC_B3, INPUT);
  pinMode(ENC_A4, INPUT);
  pinMode(ENC_B4, INPUT);

  // Configurar pines de los motores como salida
  pinMode(IN1_M1, OUTPUT);
  pinMode(IN2_M1, OUTPUT);
  pinMode(ENA_M1, OUTPUT);

  pinMode(IN1_M2, OUTPUT);
  pinMode(IN2_M2, OUTPUT);
  pinMode(ENA_M2, OUTPUT);

  pinMode(IN1_M3, OUTPUT);
  pinMode(IN2_M3, OUTPUT);
  pinMode(ENA_M3, OUTPUT);

  pinMode(IN1_M4, OUTPUT);
  pinMode(IN2_M4, OUTPUT);
  pinMode(ENA_M4, OUTPUT);

  // Interrupciones para contar pulsos de encoders
  attachInterrupt(digitalPinToInterrupt(ENC_A1), countPulse1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), countPulse2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A3), countPulse3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A4), countPulse4, RISING);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;

    // Cálculo de RPM de cada motor
    float rpm1 = (pulseCount1 / 20.0) * 60.0;
    float rpm2 = (pulseCount2 / 20.0) * 60.0;
    float rpm3 = (pulseCount3 / 20.0) * 60.0;
    float rpm4 = (pulseCount4 / 20.0) * 60.0;

    Serial.print("RPM Motor 1: "); Serial.println(rpm1);
    Serial.print("RPM Motor 2: "); Serial.println(rpm2);
    Serial.print("RPM Motor 3: "); Serial.println(rpm3);
    Serial.print("RPM Motor 4: "); Serial.println(rpm4);

    // Resetear contadores de pulsos
    pulseCount1 = 0;
    pulseCount2 = 0;
    pulseCount3 = 0;
    pulseCount4 = 0;
  }
  
  // Controlar los motores
  motorControl(1, 30); // Motor 1 a velocidad 30
  motorControl(2, 30); // Motor 2 a velocidad 30
  motorControl(3, 30); // Motor 3 a velocidad 30
  motorControl(4, 30); // Motor 4 a velocidad 30
}

// Función para contar pulsos de cada encoder
void countPulse1() { pulseCount1++; }
void countPulse2() { pulseCount2++; }
void countPulse3() { pulseCount3++; }
void countPulse4() { pulseCount4++; }

// Función de control para los motores
void motorControl(int motor, int velocidad) {
  int in1, in2, ena;
  
  // Seleccionar los pines del motor correspondiente
  switch (motor) {
    case 1:
      in1 = IN1_M1; in2 = IN2_M1; ena = ENA_M1;
      break;
    case 2:
      in1 = IN1_M2; in2 = IN2_M2; ena = ENA_M2;
      break;
    case 3:
      in1 = IN1_M3; in2 = IN2_M3; ena = ENA_M3;
      break;
    case 4:
      in1 = IN1_M4; in2 = IN2_M4; ena = ENA_M4;
      break;
    default:
      return;
  }
  
  // Control de dirección y velocidad
  digitalWrite(in1, HIGH);  // Sentido de rotación
  digitalWrite(in2, LOW);   // Sentido de rotación
  analogWrite(ena, velocidad); // Velocidad
}
