const int R = 9;
const int G = 10;
const int B = 11;

void differentColor() {
  // Yellow
  Serial.println("Yellow");
  analogWrite(R, 255);
  analogWrite(G, 255);
  analogWrite(B, 0);
  delay(2000);

  // Black
  Serial.println("Black");
  analogWrite(R, 0);
  analogWrite(G, 0);
  analogWrite(B, 0);
  delay(2000);

  // Purple
  Serial.println("Purple");
  analogWrite(R, 31);
  analogWrite(G, 0);
  analogWrite(B, 56);
  delay(2000);
  
  // Blue
  Serial.println("Blue");
  analogWrite(R, 0);
  analogWrite(G, 0);
  analogWrite(B, 255);
  delay(2000);

  // Pink
  Serial.println("Pink");
  analogWrite(R, 255);
  analogWrite(G, 0);
  analogWrite(B, 200);
  delay(2000);

  // Red
  Serial.println("Red");
  analogWrite(R, 255);
  analogWrite(G, 0);
  analogWrite(B, 0);
  delay(2000);

  // Green
  Serial.println("Green");
  analogWrite(R, 0);
  analogWrite(G, 255);
  analogWrite(B, 0);
  delay(2000);
}

void setup() {
  Serial.begin(9600);
  pinMode(R,OUTPUT);
  pinMode(G,OUTPUT);
  pinMode(B,OUTPUT);
}

void loop() {
  differentColor();
}
