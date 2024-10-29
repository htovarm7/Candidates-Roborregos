// Auto omni Prueba  ESP32
int FI_in1 = 32;  // Pin control giro Motor Frontal Izquierdo
int FI_in2 = 33;  // Pin control giro Motor Frontal Izquierdo
int TI_in3 = 25;  // Pin control giro Motor Trasero Izquierdo
int TI_in4 = 26;  // Pin control giro Motor Trasero Izquierdo
int TD_in1 = 27;  // Pin control giro Motor Trasero Derecho
int TD_in2 = 14;  // Pin control giro Motor Trasero Derecho
int FD_in3 = 12;  // Pin control giro Motor Frontal Derecho
int FD_in4 = 13;  // Pin control giro Motor Frontal Derecho

int tiempo = 1500;


void setup ()
{
    pinMode(FI_in1, OUTPUT);    // Configura  los pines como salida
    pinMode(FI_in2, OUTPUT);
    pinMode(TI_in3, OUTPUT);    // Configura  los pines como salida
    pinMode(TI_in4, OUTPUT);
    pinMode(TD_in1, OUTPUT);    // Configura  los pines como salida
    pinMode(TD_in2, OUTPUT);
    pinMode(FD_in3, OUTPUT);    // Configura  los pines como salida
    pinMode(FD_in4, OUTPUT);

    //Setups
    digitalWrite(FI_in1, LOW);  // PARA
    digitalWrite(FI_in2, LOW);
    digitalWrite(TI_in3, LOW);  // PARA
    digitalWrite(TI_in4, LOW);
    digitalWrite(TD_in1, LOW);  // PARA
    digitalWrite(TD_in2, LOW);
    digitalWrite(FD_in3, LOW);  // PARA
    digitalWrite(FD_in4, LOW);
    delay(5000);
}
void loop()
{
  Frente();
  delay(500);
  Parar();
  delay(tiempo);
  Reversa();
  delay(500 );
  Parar();
  delay(tiempo);
  Izquierda();
  delay(tiempo);
  Parar();
  delay(tiempo);
  Derecha();
  delay(tiempo);
  Parar();
  delay(tiempo);
  GiroDerecha();
  delay(tiempo);
  Parar();
  delay(tiempo);
  GiroIzquierda();
  delay(tiempo);
   Parar();
  delay(tiempo); 
}

void Izquierda(){
  digitalWrite(FI_in1, LOW);  // GIRO Reversa
  digitalWrite(FI_in2, HIGH);
  digitalWrite(TI_in3, LOW);  // GIRO Frente
  digitalWrite(TI_in4, HIGH);
  digitalWrite(TD_in1, HIGH);  // GIRO Frente
  digitalWrite(TD_in2, LOW);
  digitalWrite(FD_in3, HIGH);  // GIRO Revers
  digitalWrite(FD_in4, LOW);
}
void Derecha(){
  digitalWrite(FI_in1, HIGH);  // GIRO Frente
  digitalWrite(FI_in2, LOW);
  digitalWrite(TI_in3, HIGH);  // GIRO Reversa
  digitalWrite(TI_in4, LOW);
  digitalWrite(TD_in1, LOW);  // GIRO Reversa
  digitalWrite(TD_in2, HIGH);
  digitalWrite(FD_in3, LOW);  // GIRO Frente
  digitalWrite(FD_in4, HIGH);
}
void Parar(){
  digitalWrite(FI_in1, LOW);  // PARA
  digitalWrite(FI_in2, LOW);
  digitalWrite(TI_in3, LOW);  // PARA
  digitalWrite(TI_in4, LOW);
  digitalWrite(TD_in1, LOW);  // PARA
  digitalWrite(TD_in2, LOW);
  digitalWrite(FD_in3, LOW);  // PARA
  digitalWrite(FD_in4, LOW);
}

void Frente(){
  digitalWrite(FI_in1, HIGH);  // GIRO Frente
  digitalWrite(FI_in2, LOW);
  digitalWrite(TI_in3, LOW);  // GIRO Frente
  digitalWrite(TI_in4, HIGH);
  digitalWrite(TD_in1, HIGH);  // GIRO Frente
  digitalWrite(TD_in2, LOW);
  digitalWrite(FD_in3, LOW);  // GIRO Frente
  digitalWrite(FD_in4, HIGH);
}
void Reversa(){
  digitalWrite(FI_in1, LOW);  // GIRO Reversa
  digitalWrite(FI_in2, HIGH);
  digitalWrite(TI_in3, HIGH);  // GIRO Reversa
  digitalWrite(TI_in4, LOW);
  digitalWrite(TD_in1, LOW);  // GIRO Reversa
  digitalWrite(TD_in2, HIGH);
  digitalWrite(FD_in3, HIGH);  // GIRO Reversa
  digitalWrite(FD_in4, LOW);
}
void GiroDerecha(){
  digitalWrite(FI_in1, HIGH);  // GIRO Frente
  digitalWrite(FI_in2, LOW);
  digitalWrite(TI_in3, LOW);  // GIRO Frente
  digitalWrite(TI_in4, HIGH);
  digitalWrite(TD_in1, LOW);  // GIRO Reversa
  digitalWrite(TD_in2, HIGH);
  digitalWrite(FD_in3, HIGH);  // GIRO Reversa
  digitalWrite(FD_in4, LOW);
}
void GiroIzquierda(){
  digitalWrite(FI_in1, LOW);  // GIRO Reversa
  digitalWrite(FI_in2, HIGH);
  digitalWrite(TI_in3, HIGH);  // GIRO Reversa
  digitalWrite(TI_in4, LOW);
  digitalWrite(TD_in1, HIGH);  // GIRO Frente
  digitalWrite(TD_in2, LOW);
  digitalWrite(FD_in3, LOW);  // GIRO Frente
  digitalWrite(FD_in4, HIGH);
}
void Diag_Der(){
  digitalWrite(FI_in1, HIGH);  // GIRO Frente
  digitalWrite(FI_in2, LOW);
  digitalWrite(TI_in3, LOW);  // Paro
  digitalWrite(TI_in4, LOW);
  digitalWrite(TD_in1, LOW);  // Paro
  digitalWrite(TD_in2, LOW);
  digitalWrite(FD_in3, LOW);  // GIRO Frente
  digitalWrite(FD_in4, HIGH);
}
void Rever_Diag_Der(){
  digitalWrite(FI_in1, LOW);  // GIRO Frente
  digitalWrite(FI_in2, HIGH);
  digitalWrite(TI_in3, LOW);  // Paro
  digitalWrite(TI_in4, LOW);
  digitalWrite(TD_in1, LOW);  // Paro
  digitalWrite(TD_in2, LOW);
  digitalWrite(FD_in3, HIGH);  // GIRO Frente
  digitalWrite(FD_in4, LOW);
}
void Rever_Diag_Izq(){
  digitalWrite(FI_in1, LOW);  // GIRO Frente
  digitalWrite(FI_in2, LOW);
  digitalWrite(TI_in3, HIGH);  // Paro
  digitalWrite(TI_in4, LOW);
  digitalWrite(TD_in1, LOW);  // Paro
  digitalWrite(TD_in2, HIGH);
  digitalWrite(FD_in3, LOW);  // GIRO Frente
  digitalWrite(FD_in4, LOW);
}
void Diag_Izq(){
  digitalWrite(FI_in1, LOW);  // GIRO Frente
  digitalWrite(FI_in2, LOW);
  digitalWrite(TI_in3, LOW);  // Paro
  digitalWrite(TI_in4, HIGH);
  digitalWrite(TD_in1, HIGH);  // Paro
  digitalWrite(TD_in2, LOW);
  digitalWrite(FD_in3, LOW);  // GIRO Frente
  digitalWrite(FD_in4, LOW);
}