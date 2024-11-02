const int pinD1 = 30;
const int pinD2 = 31;
const int pinD3 = 32;
const int pinD4 = 33; 
const int pinD5 = 34;
const int pinD6 = 35;
const int pinD7 = 36;
const int pinD8 = 37;

// Motores
//  Pines motor superior izquierdo
const int IN1_SI = 47;
const int IN2_SI = 46;
const int ENA_SI = 7;
const int ENC_A_SI = 11;
const int ENC_B_SI = 10;

// Pines motor superior derecho
const int IN1_SD = 48;
const int IN2_SD = 49;
const int ENB_SD = 6;
const int ENC_A_SD = 9;
const int ENC_B_SD = 8;

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
const int ENC_A_ID = 7;
const int ENC_B_ID = 6;

// Funciones de movimiento
void adelante(){
    // Motor superior derecho JALA 
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENA_SI,240);

    // Motor inferior izquierdo JALA CM
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENA_II,170);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENB_SD,130);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,220);
    delay(1200);
}

void girarIzquierda() {

}

void girarDerecha() {
    // Motor superior derecho JALA 
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENA_SI,0);

    // Motor inferior izquierdo JALA CM
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENA_II,120);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENB_SD,120);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENB_ID,120);
    delay(500);
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
    adelante();
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

