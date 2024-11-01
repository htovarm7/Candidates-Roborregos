const int pinD1 = 30;
const int pinD2 = 31;
const int pinD3 = 32;
const int pinD4 = 33; 
const int pinD5 = 34;
const int pinD6 = 35;
const int pinD7 = 36;
const int pinD8 = 37;

void setup() {
  Serial.begin(9600);
  pinMode(pinD1, INPUT);
  pinMode(pinD2, INPUT);
  pinMode(pinD3, INPUT);
  pinMode(pinD4, INPUT);
  pinMode(pinD5, INPUT);
  pinMode(pinD6, INPUT);
  pinMode(pinD7, INPUT);
  pinMode(pinD8, INPUT);
  
  // Configurar pines de los motores aquí
}

void loop() {
  // Leer el estado de cada sensor de línea
  int valorD1 = digitalRead(pinD1);
  int valorD2 = digitalRead(pinD2);
  int valorD3 = digitalRead(pinD3);
  int valorD4 = digitalRead(pinD4);
  int valorD5 = digitalRead(pinD5);
  int valorD6 = digitalRead(pinD6);
  int valorD7 = digitalRead(pinD7);
  int valorD8 = digitalRead(pinD8);

  // Imprimir el valor para depuración
  Serial.print(valorD1);
  Serial.print(valorD2);
  Serial.print(valorD3);
  Serial.print(valorD4);
  Serial.print(valorD5);
  Serial.print(valorD6);
  Serial.print(valorD7);
  Serial.println(valorD8); 

  // Lógica básica de seguimiento de línea
  if (valorD4 == 1 && valorD5 == 1) {  // Línea centrada
    avanzar();
  }
  else if (valorD1 == 1 || valorD2 == 1 || valorD3 == 1) {  // Giro a la izquierda
    girarIzquierda();
  }
  else if (valorD6 == 1 || valorD7 == 1 || valorD8 == 1) {  // Giro a la derecha
    girarDerecha();
  }
  else {  // Ningún sensor detecta la línea
    detener();
  }

  delay(100); // Pequeña pausa para estabilidad
}

// Funciones de movimiento
void avanzar() {
  Serial.println("Avanzando");
  // Código para avanzar el robot
}

void girarIzquierda() {
  Serial.println("Girando a la izquierda");
  // Código para girar el robot a la izquierda
}

void girarDerecha() {
  Serial.println("Girando a la derecha");
  // Código para girar el robot a la derecha
}

void detener() {
  Serial.println("Detenido");
  // Código para detener el robot
}
