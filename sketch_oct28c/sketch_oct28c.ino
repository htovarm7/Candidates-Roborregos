// Puente H (motores a la izquierda)
// Motor superior izquierdo
const int ENA_Izquierdo = 5; 
const int INA1_Izquierdo = 28;
const int INA2_Izquierdo = 27;

// Motor inferior izquierdo
const int ENB_Izquierdo = 6;
const int INB1_Izquierdo = 26;
const int INB2_Izquierdo = 25;

// Puente H (motores a la derecha)
// Motor superior derecho
const int ENA_Derecho = 7;
const int INA1_Derecho = 32;
const int INA2_Derecho = 31;

// Motor inferior derecho
const int ENB_Derecho = 8;
const int INB1_Derecho = 30;
const int INB2_Derecho = 29;

void setup() {
  // Motores Lado Izquierdo
    pinMode(ENA_Izquierdo, OUTPUT);
    pinMode(INA1_Izquierdo, OUTPUT);
    pinMode(INA2_Izquierdo, OUTPUT);
    pinMode(INB1_Izquierdo, OUTPUT);
    pinMode(INB2_Izquierdo, OUTPUT);
    pinMode(ENB_Izquierdo, OUTPUT);

    // Motores Lado Derecho
    pinMode(ENA_Derecho, OUTPUT);
    pinMode(INA1_Derecho, OUTPUT);
    pinMode(INA2_Derecho, OUTPUT);
    pinMode(INB1_Derecho, OUTPUT);
    pinMode(INB2_Derecho, OUTPUT);
    pinMode(ENB_Derecho, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  avanzar();
  delay(2000);
  detener();

  giroIzquierda();
  delay(2000);
  giroDerecha();
  giroDerecha();
  giroDerecha();
  giroDerecha();
}

void giroDerecha(){
    // Giro de los motores lado izquierdo
    digitalWrite(INA1_Izquierdo,HIGH); 
    digitalWrite(INA2_Izquierdo,LOW);
    digitalWrite(INB1_Izquierdo,HIGH);
    digitalWrite(INB2_Izquierdo,LOW);

    //Giro de los motores lado derecho
    digitalWrite(INA1_Derecho,HIGH); 
    digitalWrite(INA2_Derecho,LOW);
    digitalWrite(INB1_Derecho,HIGH);
    digitalWrite(INB2_Derecho,LOW);

    // Energia/potencia
    analogWrite(ENA_Izquierdo,0);
    analogWrite(ENB_Izquierdo,255);
    analogWrite(ENA_Derecho,255);
    analogWrite(ENB_Derecho,0);
}

void giroIzquierda(){
    digitalWrite(INA1_Izquierdo,HIGH); 
    digitalWrite(INA2_Izquierdo,LOW);
    digitalWrite(INB1_Izquierdo,HIGH);
    digitalWrite(INB2_Izquierdo,LOW);

    //Giro de los motores lado derecho
    digitalWrite(INA1_Derecho,HIGH); 
    digitalWrite(INA2_Derecho,LOW);
    digitalWrite(INB1_Derecho,HIGH);
    digitalWrite(INB2_Derecho,LOW);

    // Energia/potencia
    analogWrite(ENA_Izquierdo,255);
    analogWrite(ENB_Izquierdo,0);
    analogWrite(ENA_Derecho,0);
    analogWrite(ENB_Derecho,255);
}

void detener(){
    analogWrite(ENA_Izquierdo,0);
    analogWrite(ENB_Izquierdo,0);
    analogWrite(ENA_Derecho,0);
    analogWrite(ENB_Derecho,0);
}

void avanzar(){
    analogWrite(ENA_Izquierdo,100);
    analogWrite(ENB_Izquierdo,100);
    analogWrite(ENA_Derecho,100);
    analogWrite(ENB_Derecho,100);
}
