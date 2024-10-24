const int ENC_A = 6;
const int ENC_B = 5;
char op = '0';
char vel[] = {' ', ' ', ' '};
const int IN1 = 2;
const int IN2 = 3;
const int ENA = 4;


void setup() {
  Serial.begin(9600);
  pinMode(ENC_A,INPUT);
  pinMode(ENC_B,INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ENA,OUTPUT);
}

void loop() {
  
}
