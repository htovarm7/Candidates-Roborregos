// Pines ultrasonico Izquierdo
const int ultrasonicoIzquierdoEcho = 42;
const int ultrasonicoIzquierdoTrig = 43;
// Pines ultrasonico Frontal
const int ultrasonicoFrontalEcho = 40;
const int ultrasonicoFrontalTrig = 41;
// Pines ultrasonico Derecha
const int ultrasonicoDerechaEcho = 38;
const int ultrasonicoDerechaTrig = 39;

// Motores
//  Pines motor superior izquierdo
const int IN1_SI = 47;
const int IN2_SI = 46;
const int ENA_SI = 7;
const int ENC_A_SI = 9;
const int ENC_B_SI = 8;

// Pines motor superior derecho
const int IN1_SD = 48;
const int IN2_SD = 49;
const int ENB_SD = 6;
const int ENC_A_SD = 3;
const int ENC_B_SD = 2;

// Pines motor inferior izquierdo
const int IN1_II = 52;
const int IN2_II = 53;
const int ENA_II = 5;
const int ENC_A_II = 13;
const int ENC_B_II = 12;

// Pines motor inferior derecho
const int IN1_ID = 50;
const int IN2_ID = 49;
const int ENB_ID = 4;
const int ENC_A_ID = 11;
const int ENC_B_ID = 10;

const int pwmIzq = 255;
const int pmwDer = 255;

// Actuadores

// Servomotores
Servo_SI = 22;
Servo_SD = 23;

// Sensor de color
SCL_COLOR = A0;
SDA_COLOR = A1; 

// Giroscopio
SCL_GIRO = A2;
SDA_GIRO = A3;

// Sensor de linea

void setup()
{
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

void loop()
{
  long distanciaIzquierdo = distanciaUltrasonico(ultrasonicoIzquierdoTrig, ultrasonicoIzquierdoEcho);

  long distanciaFrontal = distanciaUltrasonico(ultrasonicoFrontalTrig, ultrasonicoFrontalEcho);

  long distanciaDerecha = distanciaUltrasonico(ultrasonicoDerechaTrig, ultrasonicoDerechaEcho);

  // Distancia de los ultrasonicos
  // Izquierdo
  Serial.print("Izquierdo: ");
  Serial.print(distanciaIzquierdo);
  Serial.println(" cm");
  // Frontal
  Serial.print("Frontal: ");
  Serial.print(distanciaFrontal);
  Serial.println(" cm");
  // Derecha
  Serial.print("Derecha: ");
  Serial.print(distanciaDerecha);
  Serial.println(" cm");

  delay(50);
}

long distanciaUltrasonico(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH);
  long distancia = duracion * 0.034 / 2;
  return distancia;
}
