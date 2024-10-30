/* INCLUDES */

// Arduino includes
#include <ArduinoSTL.h>
#include <vector>
#include <set>
#include <queue>
#include <map>

// Own function includes
#include "PID.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "ColorConverterLib.h"
#include "Ultrasonic.h"
#include "Motors.h"

/* CONSTANTS */

// Pines ultrasonico Izquierdo
const int leftEcho = 42;
const int leftTrig = 43;

// Pines ultrasonico Frontal
const int frontEcho = 40;
const int frontTrig = 41;

// Pines ultrasonico Derecha
const int rightEcho = 38;
const int rightTrig = 39;

// Motores
//  Pines motor superior izquierdo
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

// Actuadores

// Servomotores
const int Servo_SI = 22;
const int Servo_SD = 23;

// Sensor de color
const char SCL_COLOR = A0;
const char SDA_COLOR = A1; 

// Giroscopio
const char SCL_GIRO = A2;
const char SDA_GIRO = A3;

// Sensor de linea
const int sensorLineaD1 = ;
const int sensorLineaD2 = ;
const int sensorLineaD3 = ;
const int sensorLineaD4 = ;
const int sensorLineaD5 = ;
const int sensorLineaD6 = ;
const int sensorLineaD7 = ;
const int sensorLineaD8 = ;

// LED RGB
const int R = 9;
const int G = 11;
const int B = 10;

// Enconders
const int encoderSuperiorIzquierdo_A = 2;
const int encoderSuperiorIzquierdo_B = 3;
const int encoderSuperiorDerecho_A = 40;
const int encoderSuperiorDerecho_B = 41;
const int encoderInferiorIzquierdo_A = 19;
const int encoderInferiorIzquierdo_B = 18;
const int encoderInferiorDerecho_A = 43;
const int encoderInferiorDerecho_B = 42;

// Giroscopio PLACEHOLDER
const int giroSCL = 1; 
const int giroSDA = 2; 

// Sensor de color PLACEHOLDER
const int colorSDA = 4; 
const int colorSCL = 5;

// Sensores de línea
const int sensorLineaD8 = 50;
const int sensorLineaD7 = 49;
const int sensorLineaD6 = 48;
const int sensorLineaD5 = 47;
const int sensorLineaD4 = 46;
const int sensorLineaD3 = 45;
const int sensorLineaD2 = 44;
const int sensorLineaD1 = 43;

// Servos
const int servo1 = 35;
const int servo2 = 34;

/* OBJECTS (SENSORS) */

// Motor object instantiation.
Motors myMotors(
    IN1_SI, IN2_SI, ENA_SI, 
    IN1_II, IN2_II, ENA_II,
    IN1_SD, IN2_SI, ENB_SD, 
    IN1_ID, IN2_ID, ENB_ID);

// Ultrasonic sensor object instantiation.
Ultrasonic frontUltrasonic(frontEcho, frontTrig);
Ultrasonic rightUltrasonic(rightEcho, rightTrig);
Ultrasonic leftUltrasonic(leftEcho, leftTrig);

// Color sensor.
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

/* LOGIC VARIABLES */

// Keeps track of current track, based on starting color conditions.
std::string track = "";

// std::Vector that std::maps the four directions —up, left, down, right— respectively.
std::vector<std::pair<int, int>> directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

// Values based on "directions" std::vector.
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// Robot's current orientation; starts east.
int orientation = EAST;

/// Track C setup.

// Color mapping.
std::vector<std::vector<std::string>> colorMap(3, std::vector<std::string> (5, ""));

// Walls
std::vector<std::vector<bool>> verticalWalls(3, std::vector<bool> (4, 0));
std::vector<std::vector<bool>> horizontalWalls(2, std::vector<bool> (5, 0));

// std::Set that holds the currently visited cells.
std::set<std::pair<int, int>> visited;

// Adjacency list of the area.
std::map<std::pair<int, int>, std::set<std::pair<int, int>>> AL;

// std::Map containing the ocurrences of each color.
std::map<std::string, int> detectedColors;

// Robot always starts at (1, 4).
std::pair<int, int> currentPosition = {1, 4};

// Movements to perform for "steps" std::vector.
enum Steps {
    FORWARD = 0,
    RIGHT = 1,
    LEFT = 2
};

std::vector<std::vector<int>> steps = {{FORWARD}, {RIGHT, FORWARD}, {RIGHT, RIGHT, FORWARD}, {LEFT, FORWARD}};

/* CONTROL FUNCTIONS */

void giroDerecha(){
    // Giro de los motores lado izquierdo
    digitalWrite(INA1L,HIGH); 
    digitalWrite(INA2L,LOW);
    digitalWrite(INB1L,HIGH);
    digitalWrite(INB2L,LOW);

    //Giro de los motores lado derecho
    digitalWrite(INA1R,HIGH); 
    digitalWrite(INA2R,LOW);
    digitalWrite(INB1R,HIGH);
    digitalWrite(INB2R,LOW);

    // Energia/potencia
    analogWrite(ENAL,0);
    analogWrite(ENBL,255);
    analogWrite(ENAR,255);
    analogWrite(ENBR,0);
}

void giroIzquierda(){
    digitalWrite(INA1L,HIGH); 
    digitalWrite(INA2L,LOW);
    digitalWrite(INB1L,HIGH);
    digitalWrite(INB2L,LOW);

    //Giro de los motores lado derecho
    digitalWrite(INA1R,HIGH); 
    digitalWrite(INA2R,LOW);
    digitalWrite(INB1R,HIGH);
    digitalWrite(INB2R,LOW);

    // Energia/potencia
    analogWrite(ENAL,255);
    analogWrite(ENBL,0);
    analogWrite(ENAR,0);
    analogWrite(ENBR,255);
}

void stop(){
    analogWrite(ENAL,0);
    analogWrite(ENBL,0);
    analogWrite(ENAR,0);
    analogWrite(ENBR,0);
}

void encenderRGB(){
    // Hay que ver que colores van haber y crear un if de que lo que dio 
    // el sensor de color pues colocar un color, son
    // 4 cuadros rosas
    // 4 cuadros morados
    // 1 cuadro azul que es el inicio
    // 1 cuadro rojo que es el cheeckpoint
    // 1 cuadro verde que es el final
    // 1 cuadro negro que no se debe tocar
    // 5 cuadros amarrillos
    analogWrite(R,125);
    analogWrite(G,125);
    analogWrite(B,125);
}

void handleMove(int movement) {
    if (movement == FORWARD) {
        myMotors.forward(100);
    }
    else if (movement == RIGHT) {
        //MovimientosLocos::GiroDerecha();
    }
    else if (movement == LEFT) {
        //MovimientosLocos::GiroIzquierda();
    }
}

void doMove(std::pair<int, int> currentPosition, std::pair<int, int> nextPosition) {
    int cx = currentPosition.first;
    int cy = currentPosition.second;
    int nx = nextPosition.first;
    int ny = nextPosition.second;

    if (nx == cx - 1) {
        for (auto i : steps[orientation]) {
            handleMove(i);
        }
    }
    else if (ny == cy - 1) {
        for (auto i : steps[(orientation + 1) % 4]) {
            handleMove(i);
        }
    }
    else if (nx == cx + 1) {
        for (auto i : steps[(orientation + 2) % 4]) {
            handleMove(i);
        }
    }
    else if (ny == cy + 1) {
        for (auto i : steps[(orientation + 3) % 4]) {
            handleMove(i);
        }
    }
}

/* LOGIC FUNCTIONS */

/// Track C

// bool checkWall(int direction, std::pair<int, int> currentNode) {
//     // Cartesian notation for readbility's sake.
//     int x = currentNode.first;
//     int y = currentNode.second;

//     if (direction == NORTH) {
//         // Check horizontal wall at (x - 1, y).
//         return horizontalWalls[x - 1][y];
//     }
//     else if (direction == EAST) {
//         // Check vertical wall at (x, y - 1).
//         return verticalWalls[x][y - 1];
//     }
//     else if (direction == SOUTH) {
//         // Check horizontal wall at (x, y).
//         return horizontalWalls[x][y];
//     }
//     else { // if (direction == WEST)
//         // Check vertical wall at (x, y).
//         return verticalWalls[x][y];
//     }
    
// }

// Finds the most frequent color in a std::map of colors.
std::string findMostFrequentColor(std::map<std::string, int> colorMap) {
    // Iterate through the std::map of detected colors to find the most seen color.
    std::pair<std::string, int> mostFrequent = {"Red", 0};
    for (auto i : colorMap) {
        if (i.second > mostFrequent.second) {
            mostFrequent = i;
        }
    }

    return mostFrequent.first;
}

std::map<std::pair<int, int>, std::pair<int, int>> bfs(std::pair<int, int> start) {
    // Declare needed data structures.
    std::map<std::pair<int, int>, std::pair<int, int>> parents; // Stores parents for each node.
    std::queue<std::pair<int, int>> q; // Node processing std::queue.
    std::map<std::pair<int, int>, int> dist; // Stores distances to each node.
    
    // Initialize structures from start node.
    parents[start] = start; // This helps to recognize the end of the path, by way of parent[x] == x.
    q.push(start);
    dist[start] = 0;

    // BFS logic.
    while (!q.empty()) {
        std::pair<int, int> u = q.front();
        q.pop();

        // For every possible connection from node u.
        for (auto v : AL[u]) {
            // Additional check on v == start as we don't initialize all distances on infinity, as per usual in BFS.
            if (dist[v] != 0 || v == start) continue;

            // Update shortest distance, add node to std::queue, and update its parent.
            parents[v] = u;
            q.push(v);
            dist[v] = dist[u] + 1;
        }
    }

    // Return parent std::map.
    return parents;
}

void moveToNewPosition(std::pair<int, int> newPosition, std::pair<int, int>& currentPosition) {
    // Call bfs to get the path.
    std::map<std::pair<int, int>, std::pair<int, int>> parents = bfs(newPosition);

    while (parents[currentPosition] != currentPosition) {
        // Physically move towards the parent.
        doMove(currentPosition, parents[currentPosition]);
        currentPosition = parents[currentPosition];
    }
}

void dfs(std::pair<int, int> node) {
    visited.insert(node);
    if (AL[node].find(currentPosition) == AL[node].end()) {
        Serial.println("Call BFS!");
        moveToNewPosition(node, currentPosition);

        Serial.println("BFS Done.");
    }
    Serial.print(node.first);
    Serial.print(" ");
    Serial.println(node.second);
    currentPosition = node;

    // Detect color in cell, show, and save, only if it had not been std::set before.
    if (colorMap[node.first][node.second] == ""){
        //Find color based on samples.
        std::map<std::string, int> detectedColorCount;
        
        for (int i = 0; i < 10; i++) {
            detectedColorCount[getColor(tcs)]++;
            delay(50);
        }
        std::string detectedColor = findMostFrequentColor(detectedColorCount);
        // LEDRBG: std::setColor(detectedColor);

        colorMap[node.first][node.second] = detectedColor;
        detectedColors[detectedColor]++;
    }

    // Keep going only if it's not a black square.
    if (colorMap[node.first][node.second] == "Black") {
        // TODO: girar!
        // MovimientosLocos::GiroIzquierda();
        // MovimientosLocos::GiroIzquierda();
        delay(1000);
        myMotors.forward(100);
        delay(1000);
        myMotors.stop();
        return;
    }
    else {
        myMotors.forward(100);
        delay(1000);
        myMotors.stop();
    }

    // If all cells are visited, move to checkpoint.
    if (visited.size() == 15) {
        moveToNewPosition({0, 0}, currentPosition);

        // girar(90); // to face the checkpoint.
        myMotors.forward(100);
        delay(1000);

        std::string mostSeenColor = findMostFrequentColor(detectedColors);
        // LEDRBG::std::setColor(mostSeenColor);
    }

    for (int i = 0; i < 4; i++) {
        int dx = directions[i].first;
        int dy = directions[i].second;

        int nx = node.first + dx;
        int ny = node.second + dy;

        // If coordinates are out of bounds, skip.
        if (nx < 0 || ny < 0 || nx > 2 || ny > 4) continue;

        // Turn to direction
        // girar(90) o algo así

        // If there's a wall, skip.
        // Physical:
        if (frontUltrasonic.getDistance() < 10) continue;

        // If there's no wall, update adjacency list. 
        AL[node].insert({nx, ny});
        AL[{nx, ny}].insert(node);

        // Call dfs with new node only if it's not been visited yet.
        if (!visited.count({nx, ny})) {
            // Move in that direction, then call dfs.
            myMotors.forward(100);
            delay(1000);
            myMotors.stop();

            dfs({nx, ny});
        }
    }
}

// Function to get color.
std::string getColor(Adafruit_TCS34725 tcs) {
    uint16_t clear, red, green, blue;
    tcs.setInterrupt(false);
    delay(60); 
    
    // Capture color.
    tcs.getRawData(&red, &green, &blue, &clear);
    tcs.setInterrupt(true);

    // Normalize RGB values.
    uint32_t sum = clear;
    float r = red / (float)sum;
    float g = green / (float)sum;
    float b = blue / (float)sum;
    r *= 256; g *= 256; b *= 256;

    // Convert to HSV.
    double hue, saturation, value;
    ColorConverter::RgbToHsv(r, g, b, hue, saturation, value);

    // Return color based on tested values.
    if (hue > 30 && hue < 105) {
        return "Yellow";
    }
    else if (hue > 222 && hue < 265) {
        return "Purple";
    }
    else if (hue > 195 && hue < 222) {
        return "Blue";
    }
    else if (hue > 300 && hue < 350) {
        return "Pink";
    }
    else if (hue > 0 && hue < 25) {
        return "Red";
    }
    else if (hue > 120 && hue < 160) {
        return "Green";
    }
    else {
        return "None of the above";
    }
}

// Control de motores
void adelante(){
    
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrie(ENA_SI,pwmIzq);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrie(ENA_II,pwmIzq);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrie(ENB_SD,pmwDer);


    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrie(ENB_ID,pmwDer);
    delay(2000);
}

void detener(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,LOW);
    analogWrie(ENA_SI,zero);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,LOW);
    analogWrie(ENA_II,zero);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,LOW);
    analogWrie(ENB_SD,zero);


    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,LOW);
    analogWrie(ENB_ID,zero);
    delay(2000);
}

void giroDerecha(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrie(ENA_SI,pwmIzq);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,HIGH);
    analogWrie(ENA_II,pwmIzq);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrie(ENB_SD,zero);


    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrie(ENB_ID,zero);
    
    delay(2000);
}

void giroIzquierda(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,HIGH);
    digitalWrite(IN2_SI,LOW);
    analogWrie(ENA_SI,zero);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,HIGH);
    analogWrie(ENA_II,zero);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,HIGH);
    digitalWrite(IN2_SD,LOW);
    analogWrie(ENB_SD,pmwDer);


    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrie(ENB_ID,pmwDer);

    delay(2000);
}

void reversa(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrie(ENA_SI,pwmIzq);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,LOW);
    digitalWrite(IN2_II,HIGH);
    analogWrie(ENA_II,pwmIzq);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,HIGH);
    analogWrie(ENB_SD,pmwDer);


    // Motor inferior derecho
    digitalWrite(IN1_ID,LOW);
    digitalWrite(IN2_ID,HIGH);
    analogWrie(ENB_ID,pmwDer);

    delay(2000);
}

void movLateral(){
    // Motor superior izquierdo
    digitalWrite(IN1_SI,LOW);
    digitalWrite(IN2_SI,HIGH);
    analogWrie(ENA_SI,pwmIzq);

    // Motor inferior izquierdo
    digitalWrite(IN1_II,HIGH);
    digitalWrite(IN2_II,LOW);
    analogWrie(ENA_II,pwmIzq);
    
    // Motor superior derecho
    digitalWrite(IN1_SD,LOW);
    digitalWrite(IN2_SD,HIGH);
    analogWrie(ENB_SD,pmwDer);

    // Motor inferior derecho
    digitalWrite(IN1_ID,HIGH);
    digitalWrite(IN2_ID,LOW);
    analogWrie(ENB_ID,pmwDer);\
    
    delay(2000);
}

/* ARDUINO SETUP */

void setup() {
    Serial.begin(9600);

    // Motors and ultrasonic sensors.
    myMotors.init();
    frontUltrasonic.init();
    rightUltrasonic.init();
    leftUltrasonic.init();

    // Ensure color sensor is on.
    while (!tcs.begin()) delay(1000);
 
    // Actuadores
    pinMode(R, OUTPUT);
    pinMode(B, OUTPUT);
    pinMode(G, OUTPUT);
    
    // Encoders
    // Encoder Superior Izquierdo
    pinMode(ENC_A_SI, INPUT);
    pinMode(ENC_B_SI, INPUT);
    
    // Encoder Superior Derecho
    pinMode(ENC_A_SD, INPUT);
    pinMode(ENC_B_SD, INPUT);
    
    // Encoder Inferior Izquierdo
    pinMode(ENC_A_II, INPUT);
    pinMode(ENC_B_II, INPUT);

    // Encoder Inferior Derecho
    pinMode(ENC_A_ID, INPUT);
    pinMode(ENC_B_ID, INPUT);
    
    // Sensor de Linea
    pinMode(sensorLineaD8, INPUT);
    pinMode(sensorLineaD7, INPUT);
    pinMode(sensorLineaD6, INPUT);
    pinMode(sensorLineaD5, INPUT);
    pinMode(sensorLineaD4, INPUT);
    pinMode(sensorLineaD3, INPUT);
    pinMode(sensorLineaD2, INPUT);
    pinMode(sensorLineaD1, INPUT);
}

/* ARDUINO LOOP */

void loop() {
    
    float distanciaFrontal = frontUltrasonic.getDistance();
    float distanciaDerecha = rightUltrasonic.getDistance();
    float distanciaIzquierda = leftUltrasonic.getDistance();


    Serial.print("Frontal: ");
    Serial.print(distanciaFrontal);
    Serial.print(" cm, Derecha: ");
    Serial.print(distanciaDerecha);
    Serial.print(" cm, Izquierda: ");
    Serial.print(distanciaIzquierda);
    Serial.println(" cm");  
    
    // Probar esto que no sé si jalaría, especialmente con lo del PID.
    if (track == "") {
        std::string color = getColor(tcs);

        if (color == "Green") {
            track = "A";
        }
        else if (color == "Blue") {
            track = "C";
        }
        else {
            track = "B";
        }
    }

    // Start track C logic.
    if (track == "C") {
        dfs(currentPosition);
    }

    adelante();
    detener();
    giroDerecha();
    giroIzquierda();
    reversa();
    movLateral();

}