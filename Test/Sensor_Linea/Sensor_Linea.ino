const int pinD4 = 2;  // Ajusta según tu conexión

void setup() {
  Serial.begin(9600);
  pinMode(pinD4, INPUT);
}

void loop() {
  int valorD4 = digitalRead(pinD4); // Leer valor del pin
  Serial.println(valorD4); // Imprimir el valor
  delay(500); // Espera medio segundo
}
