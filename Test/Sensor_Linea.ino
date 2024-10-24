// Definir pines para los 4 motores
const int motorIzquierdoAdelante = 5;
const int motorIzquierdoAtras = 6;
const int motorDerechoAdelante = 9;
const int motorDerechoAtras = 10;

// Definir el pin del sensor de línea
const int sensorLinea = A0;

// Definir el umbral para detectar la línea
const int umbral = 500;  // Ajustar cuando se calibre

// Función para avanzar
void avanzar() {
  // Activar motores para avanzar recto
  digitalWrite(motorIzquierdoAdelante, HIGH);
  digitalWrite(motorIzquierdoAtras, LOW);
  digitalWrite(motorDerechoAdelante, HIGH);
  digitalWrite(motorDerechoAtras, LOW);
}

// Función para girar
void girar() {
  // Gira hacia la izquierda
  analogWrite(motorIzquierdoAdelante, 100);  // Reducir velocidad en un lado
  analogWrite(motorDerechoAdelante, 255);    // Mantener velocidad en el otro lado
  digitalWrite(motorIzquierdoAtras, HIGH);
  digitalWrite(motorDerechoAdelante, HIGH);
  digitalWrite(motorDerechoAtras, LOW);
}


void setup() {
  // Configurar pines de los motores como salidas
  pinMode(motorIzquierdoAdelante, OUTPUT);
  pinMode(motorIzquierdoAtras, OUTPUT);
  pinMode(motorDerechoAdelante, OUTPUT);
  pinMode(motorDerechoAtras, OUTPUT);

  // Configurar el sensor de línea como entrada
  pinMode(sensorLinea, INPUT);

  // Inicializar la comunicación serial para depuración
  Serial.begin(9600);
}

void loop() {
  // Leer el valor del sensor de línea
  int valorSensor = analogRead(sensorLinea);

  // Mostrar el valor leído para calibración
  Serial.println(valorSensor);

  if (valorSensor < umbral) {
    // Si el sensor detecta la línea (valor bajo), avanza
    avanzar();
  } else {
    // Si el sensor no detecta la línea (valor alto), gira para encontrarla
    girar();
  }

  delay(100);  // Pequeña pausa para estabilidad
}
