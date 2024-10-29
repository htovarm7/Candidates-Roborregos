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
#include <Adafruit_TCS34725.h>
#include <ColorConverterLib.h>
#include "Ultrasonico.h"
#include "Motors.h"

/* CONSTANTS */

// Ultrasonic sensors.
const int frontEcho = 37;
const int frontTrig = 38;

const int rightEcho = 52;
const int rightTrig = 53;

const int leftEcho = 22;
const int leftTrig = 24;

// Puente H (motores a la izquierda)
// Motor superior izquierdo
const int ENAL = 5; 
const int INA1L = 28;
const int INA2L = 27;

// Motor inferior izquierdo
const int ENBL = 6;
const int INB1L = 26;
const int INB2L = 25;

// Puente H (motores a la derecha)
// Motor superior derecho
const int ENAR = 7;
const int INA1R = 32;
const int INA2R = 31;

// Motor inferior derecho
const int ENBR = 8;
const int INB1R = 30;
const int INB2R = 29;

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

// Giroscopio
const int giroSCL = A5; 
const int giroSDA = A4; 

// Sensor de color
const int colorSDA = A4; 
const int colorSCL = A5;

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
    INA1L, INA2L, ENAL, 
    INB1L, INB2L, ENBL,
    INA1R, INA2R, ENAR, 
    INB1R, INB2R, ENBR);

// Ultrasonic sensor object instantiation.
Ultrasonic frontUltrasonic(frontEcho, frontTrig);
Ultrasonic rightUltrasonic(rightEcho, rightTrig);
Ultrasonic leftUltrasonic(leftEcho, leftTrig);

// Color sensor.
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

/* LOGIC VARIABLES */

// Keeps track of current track, based on starting color conditions.
string track = "";

// Vector that maps the four directions —up, left, down, right— respectively.
vector<pair<int, int>> directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

// Values based on "directions" vector.
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
vector<vector<string>> colorMap(3, vector<string> (5, ""));

// Walls
vector<vector<bool>> verticalWalls(3, vector<bool> (4, 0));
vector<vector<bool>> horizontalWalls(2, vector<bool> (5, 0));

// Set that holds the currently visited cells.
set<pair<int, int>> visited;

// Adjacency list of the area.
map<pair<int, int>, set<pair<int, int>>> AL;

// Map containing the ocurrences of each color.
map<string, int> detectedColors;

// Robot always starts at (1, 4).
pair<int, int> currentPosition = {1, 4};

// Movements to perform for "steps" vector.
enum Steps {
    FORWARD = 0,
    RIGHT = 1,
    LEFT = 2
};

vector<vector<int>> steps = {{FORWARD}, {RIGHT, FORWARD}, {RIGHT, RIGHT, FORWARD}, {LEFT, FORWARD}};

/* CONTROL FUNCTIONS */

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

void stop(){
    analogWrite(ENA_Izquierdo,0);
    analogWrite(ENB_Izquierdo,0);
    analogWrite(ENA_Derecho,0);
    analogWrite(ENB_Derecho,0);
}

// Funciones de los actuadores
void encenderBuzzer(){
    // Pin 51
    digitalWrite(buzzer,HIGH);

}

void apagarBuzzer(){
    // Pin 51
    digitalWrite(buzzer,LOW);
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
    if()
    analogWrite(R,125);
    analogWrite(G,125);
    analogWrite(B,125);
}

void handleMove(int movement) {
    if (i == FORWARD) {
        myMotors.forward(100);
    }
    else if (i == RIGHT) {
        MovimientosLocos::GiroDerecha();
    }
    else if (i == LEFT) {
        MovimientosLocos::GiroIzquierda();
    }
}

void doMove(pair<int, int> currentPosition, pair<int, int> nextPosition) {
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

// bool checkWall(int direction, pair<int, int> currentNode) {
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

// Finds the most frequent color in a map of colors.
string findMostFrequentColor(map<string, int> colorsMap) {
    // Iterate through the map of detected colors to find the most seen color.
    pair<string, int> mostFrequent = {"Red", 0};
    for (auto i : colorsMap) {
        if (i.second > mostFrequent.second) {
            mostFrequent = i;
        }
    }

    return mostFrequent.first;
}

map<pair<int, int>, pair<int, int>> bfs(pair<int, int> start) {
    // Declare needed data structures.
    map<pair<int, int>, pair<int, int>> parents; // Stores parents for each node.
    queue<pair<int, int>> q; // Node processing queue.
    map<pair<int, int>, int> dist; // Stores distances to each node.
    
    // Initialize structures from start node.
    parents[start] = start; // This helps to recognize the end of the path, by way of parent[x] == x.
    q.push(start);
    dist[start] = 0;

    // BFS logic.
    while (!q.empty()) {
        pair<int, int> u = q.front();
        q.pop();

        // For every possible connection from node u.
        for (auto v : AL[u]) {
            // Additional check on v == start as we don't initialize all distances on infinity, as per usual in BFS.
            if (dist[v] != 0 || v == start) continue;

            // Update shortest distance, add node to queue, and update its parent.
            parents[v] = u;
            q.push(v);
            dist[v] = dist[u] + 1;
        }
    }

    // Return parent map.
    return parents;
}

void moveToNewPosition(pair<int, int> newPosition, pair<int, int>& currentPosition) {
    // Call bfs to get the path.
    map<pair<int, int>, pair<int, int>> parents = bfs(newPosition);

    while (parents[currentPosition] != currentPosition) {
        // Physically move towards the parent.
        doMove(currentPostion, parents[currentPosition]);
        currentPosition = parents[currentPosition];
    }
}

void dfs(pair<int, int> node) {
    visited.insert(node);
    if (AL[node].find(currentPosition) == AL[node].end()) {
        cout << "Call BFS!" << endl;
        moveToNewPosition(node, currentPosition);

        cout << "BFS done." << endl;
    }
    cout << node.first << " " << node.second << endl;
    currentPosition = node;

    // Detect color in cell, show, and save, only if it had not been set before.
    if (colorMap[node.first][node.second] == ""){
        Find color based on samples.
        map<string, int> detectedColorCount;
        
        for (int i = 0; i < 10; i++) {
            detectedColorCount[getColor(tcs)]++;
            delay(50);
        }
        string detectedColor = findMostFrequentColor(detectedColorCount);
        // LEDRBG: setColor(detectedColor);

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

        string mostSeenColor = findMostFrequentColor(detectedColors);
        // LEDRBG::setColor(mostSeenColor);
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
string getColor(Adafruit_TCS34725 tcs) {
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
    ColorConverter::RGBtoHSV(r, g, b, hue, saturation, value);

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

/* ARDUINO SETUP */

void setup() {
    Serial.begin(9600);

    // Motors and ultrasonic sensors.
    myMotors.init();
    frontUltrasonic.init();
    rightUltrasonic.init();
    leftUltrasonic.init();
 
    // Actuadores
    pinMode(R, OUTPUT);
    pinMode(B, OUTPUT);
    pinMode(G, OUTPUT);
    
    // Encoders
    // Encoder Superior Izquierdo
    pinMode(encoderSuperiorIzquierdo_A, INPUT);
    pinMode(encoderSuperiorIzquierdo_B, INPUT);
    
    // Encoder Superior Derecho
    pinMode(encoderSuperiorDerecho_A, INPUT);
    pinMode(encoderSuperiorDerecho_B, INPUT);
    
    // Encoder Inferior Izquierdo
    pinMode(encoderInferiorIzquierdo_A, INPUT);
    pinMode(encoderInferiorIzquierdo_B, INPUT);

    // Encoder Inferior Derecho
    pinMode(encoderInferiorDerecho_A, INPUT);
    pinMode(encoderInferiorDerecho_B, INPUT);
    
    // Sensor de Linea
    pinMode(sensorLineaD8, INPUT);
    pinMode(sensorLineaD7, INPUT);
    pinMode(sensorLineaD6, INPUT);
    pinMode(sensorLineaD5, INPUT);
    pinMode(sensorLineaD4, INPUT);
    pinMode(sensorLineaD3, INPUT);
    pinMode(sensorLineaD2, INPUT);
    pinMode(sensorLineaD1, INPUT);

    // Sensor de Color
    colorSensor.begin();
}

/* ARDUINO LOOP */

void loop() {
    
    long distanciaFrontal = distanciaUltrasonico(frontTrig, frontEcho);
    long distanciaDerecha = distanciaUltrasonico(rightTrig, rightEcho);
    long distanciaIzquierda = distanciaUltrasonico(leftTrig, leftEcho);


    Serial.print("Frontal: ");
    Serial.print(distanciaFrontal);
    Serial.print(" cm, Derecha: ");
    Serial.print(distanciaDerecha);
    Serial.print(" cm, Izquierda: ");
    Serial.print(distanciaIzquierda);
    Serial.println(" cm");  
    
    // codigo de laberinto
    /*
    while (distanciaFrontal >= 10){
        adelante();
    }else{
        if(distanciaDerecha == 10){
            giroIzquierda();
        }else if(distanciaDerecha <= 10 && distanciaIzquierda <= 10 && distanciaFrontal <= 10 ){
            atras();
        }
    }
    */
    
    if (track == "") {
        string color = getColor(tcs);

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

    if (track == "C") {
        dfs(currentPosition);
    }

}