// Pines motor superior izquierdo
const int IN1_SI = 47;
const int IN2_SI = 46;
const int ENA_SI = 7;
const int ENC_A_SI = 9;
const int ENC_B_SI = 8;

// Pines motor superior derecho
const int IN1_SD = 48;
const int IN2_SD = 49;
const int ENB_SD = 6;
const int ENC_A_SD = 3;
const int ENC_B_SD = 2;

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
const int ENC_A_ID = 11;
const int ENC_B_ID = 10;

const int pwmIzq = 255;
const int pmwDer = 255;


void setup() {
    // Configurar pines como salidas

    // Motores superiores
    pinMode(IN1_SI, OUTPUT);
    pinMode(IN2_SI, OUTPUT);
    pinMode(ENA_SI, OUTPUT);

    pinMode(IN1_SD, OUTPUT);
    pinMode(IN2_SD, OUTPUT);
    pinMode(ENB_SD, OUTPUT);

    //Motores inferiores
    pinMode(IN1_II, OUTPUT);
    pinMode(IN2_II, OUTPUT);
    pinMode(ENA_II, OUTPUT);
    
    pinMode(IN1_ID, OUTPUT);
    pinMode(IN2_ID, OUTPUT);
    pinMode(ENB_ID, OUTPUT);
}

void loop() {
    // Activar motores para ir hacia adelante
    Serial.println("Adelante");

    // Motores hacia adelante
    digitalWrite(IN1_SI, HIGH);
    digitalWrite(IN2_SI, LOW);
    analogWrite(ENA_SI, pwmIzq);
    
    digitalWrite(IN1_SD, HIGH);
    digitalWrite(IN2_SD, LOW);
    analogWrite(ENB_SD, pmwDer);

    digitalWrite(IN1_II, LOW);
    digitalWrite(IN2_II, HIGH);
    analogWrite(ENA_II, 240); // Aqui debe ir FORZOSAMENTE 
    
    digitalWrite(IN1_ID, LOW);
    digitalWrite(IN2_ID, HIGH);
    analogWrite(ENB_ID, pmwDer); // Este debe ser el valor predeterminado, 4 bits mayor que el A. 
    
    delay(3000); // Mantenerse en movimiento por 1 segundo
    
    // Detener motores
    Serial.println("Detenido");

    // Se detienen Motores superiores
    digitalWrite(IN1_SI, LOW);
    digitalWrite(IN2_SI, LOW);
    analogWrite(ENA_SI, 0);
    
    digitalWrite(IN1_SD, LOW);
    digitalWrite(IN2_SD, LOW);
    analogWrite(ENB_SD, 0);

    // Se detienen Motores inferiores
    digitalWrite(IN1_II, LOW);
    digitalWrite(IN2_II, LOW);
    analogWrite(ENA_II, 0);
    
    digitalWrite(IN1_ID, LOW);
    digitalWrite(IN2_ID, LOW);
    analogWrite(ENB_ID, 0);
    delay(5000);
}