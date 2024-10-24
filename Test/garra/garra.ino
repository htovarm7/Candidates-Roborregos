#include <Servo.h> 

Servo garra;  

// Definir ángulos
int anguloCerrado = 0;   // Ángulo en que la garra está cerrada (agarrando la pelota)
int anguloAbierto = 90;  // Ángulo en que la garra está abierta (lista para agarrar)

void setup() {
  garra.attach(9);  // Conectar el servo de la garra al pin digital 9
}

void loop() {
  // Abrir la garra
  abrirGarra();
  delay(2000); 

  // Cerrar la garra
  cerrarGarra();
  delay(2000); 
}

// Abrir la garra
void abrirGarra() {
  garra.write(anguloAbierto);  
  delay(500); 
}

// Cerrar la garra
void cerrarGarra() {
  garra.write(anguloCerrado);
  delay(500);  
}
