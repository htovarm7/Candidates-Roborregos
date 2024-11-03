// Sensores de línea
const int sensorLineaD8 = 46;
const int sensorLineaD7 = 47;
const int sensorLineaD6 = 48;
const int sensorLineaD5 = 49;
const int sensorLineaD4 = 50;
const int sensorLineaD3 = 51;
const int sensorLineaD2 = 52;
const int sensorLineaD1 = 53;


// Motores
//  Pines motor superior izquierdo
const int IN1_SI = 43;
const int IN2_SI = 42;
const int ENB_SI = 4;
const int ENC_A_SI = 29;
const int ENC_B_SI = 28;

// Pines motor inferior izquierdo
const int IN1_II = 41;
const int IN2_II = 40;
const int ENB_II = 3;
const int ENC_A_II = 27;
const int ENC_B_II = 26;

// Pines motor superior derecho
const int IN1_SD = 45;
const int IN2_SD = 44;
const int ENA_SD = 5;
const int ENC_A_SD = 25;
const int ENC_B_SD = 24;

// Pines motor inferior derecho
const int IN1_ID = 39;
const int IN2_ID = 38;
const int ENA_ID = 2;
const int ENC_A_ID = 31;
const int ENC_B_ID = 30;

// Funciones de movimiento

// Esta PERFECTO por cuadrante
void avanzar() {
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,210);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENA_ID,210);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENB_II,130);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,130);

    //delay(710); // Este delay jalara por cuadrante de 30 cm centrado en medio

    delay(3000); // Para la pista del sensor de linea
}

void girarDerecha() {
    // Motor superior derecho
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,HIGH);
    analogWrite(ENA_SD,255);

    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENA_ID,255);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrite(ENB_II,255);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrite(ENB_SI,255);

    //delay(710); // Este delay jalara por cuadrante de 30 cm centrado en medio

    delay(3000); // Para la pista del sensor de linea
}

void girarIzquierda() {
    
    // Moto superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrite(ENA_SD,170);

    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrite(ENA_ID,170);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENB_II,255);
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENB_SI,220);


    delay(400);
}

void reversa(){
      // Motor superior derecho
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,HIGH);
    analogWrite(ENA_SD,40);

    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrite(ENA_ID,40);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrite(ENB_II,200); 

    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrite(ENB_SI,200);

    //delay(1000);
    delay(1000);
}

void detener() {
  // Motor superior izquierdo
  digitalWrite(IN1_SD,LOW);
  digitalWrite(IN2_SD,LOW);
  analogWrite(ENA_SD,0);

  // Motor inferior izquierdo
  digitalWrite(IN1_II,LOW);
  digitalWrite(IN2_II,LOW);
  analogWrite(ENB_II,0);
  
  // Motor superior derecho
  digitalWrite(IN1_SI,LOW);
  digitalWrite(IN2_SI,LOW);
  analogWrite(ENB_SI,0);

  // Motor inferior derecho
  digitalWrite(IN1_ID,LOW);
  digitalWrite(IN2_ID,LOW);
  analogWrite(ENA_ID,0);
  delay(1000);
}

// void giro90(){
//   // Motor superior derecho
//     digitalWrite(IN1_SD,HIGH);
//     digitalWrite(IN2_SD,LOW);
//     analogWrite(ENA_SD,150);

//     // Motor inferior derecho
//     digitalWrite(IN1_ID,LOW);
//     digitalWrite(IN2_ID,HIGH);
//     analogWrite(ENA_ID,150);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,LOW);
//     digitalWrite(IN2_II,HIGH);
//     analogWrite(ENB_II,120);
    
//     // Motor superior izquierdo
//     digitalWrite(IN1_SI,HIGH);
//     digitalWrite(IN2_SI,LOW);
//     analogWrite(ENB_SI,120);

//     delay(710); // Este delay jalara por cuadrante de 30 cm centrado en medio
// }

// void movLateral(){
//   // superior derecho e inferior izquierdo
//   // superior izquierdo e inferior derecho

//   // Motor superior derecho
//     digitalWrite(IN1_SD,HIGH);
//     digitalWrite(IN2_SD,LOW);
//     analogWrite(ENA_SD,190);

//     // Motor inferior izquierdo
//     digitalWrite(IN1_II,LOW);
//     digitalWrite(IN2_II,HIGH);
//     analogWrite(ENB_II,190);

//     // Motor superior izquierdo
//     digitalWrite(IN1_SI,HIGH);
//     digitalWrite(IN2_SI,LOW);
//     analogWrite(ENB_SI,190);
    
//     // Motor inferior derecho
//     digitalWrite(IN1_ID,HIGH);
//     digitalWrite(IN2_ID,LOW);
//     analogWrite(ENA_ID,190);

//     delay(710); // Este delay jalara por cuadrante de 30 cm centrado en medio
// }

void setup() {
  Serial.begin(9600);
  pinMode(sensorLineaD1, INPUT);
  pinMode(sensorLineaD2, INPUT);
  pinMode(sensorLineaD3, INPUT);
  pinMode(sensorLineaD4, INPUT);
  pinMode(sensorLineaD5, INPUT);
  pinMode(sensorLineaD6, INPUT);
  pinMode(sensorLineaD7, INPUT);
  pinMode(sensorLineaD8, INPUT);
  
  // Encoders
  // Encoder Superior Izquierdo
  pinMode(ENC_A_SD, INPUT);
  pinMode(ENC_B_SD, INPUT);
  
  // Encoder Superior Derecho
  pinMode(ENC_A_SI, INPUT);
  pinMode(ENC_B_SI, INPUT);
  
  // Encoder Inferior Izquierdo
  pinMode(ENC_A_II, INPUT);
  pinMode(ENC_B_II, INPUT);

  // Encoder Inferior Derecho
  pinMode(ENC_A_ID, INPUT);
  pinMode(ENC_B_ID, INPUT);
}

void loop() {
  // Leer el estado de cada sensor de línea
  int valorD1 = digitalRead(sensorLineaD1);
  int valorD2 = digitalRead(sensorLineaD2);
  int valorD3 = digitalRead(sensorLineaD3);
  int valorD4 = digitalRead(sensorLineaD4);
  int valorD5 = digitalRead(sensorLineaD5);
  int valorD6 = digitalRead(sensorLineaD6);
  int valorD7 = digitalRead(sensorLineaD7);
  int valorD8 = digitalRead(sensorLineaD8);

  //Imprimir el valor para depuración
  Serial.print(valorD1);
  Serial.print(valorD2);
  Serial.print(valorD3);
  Serial.print(valorD4);
  Serial.print(valorD5);
  Serial.print(valorD6);
  Serial.print(valorD7);
  Serial.println(valorD8); 

  //Lógica báSDca de seguimiento de línea
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
    reversa();
  }

  // Movimientos que ya jalan
  
  // avanzar();
  // detener();

  // girarDerecha();
  // detener();
  
  // girarIzquierda();
  // detener();
  
  //giro90();
  // movLateral();

  delay(500); // Pequeña pausa para estabilidad
}

