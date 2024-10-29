// Includes para Arduino
#include <ArduinoSTL.h>
#include <vector>
#include <set>
#include <queue>
#include <map>

// Los include de nuestras funciones
#include "PID.h"
#include "Clases/ColorSensing.h"
#include "Clases/Ultrasonico.h"
#include "Clases/Motor.h"

// Ultrasonicos
const int ultrasonicoFrontalEcho = 37;
const int ultrasonicoFrontalTrig = 38;

// Ultrasonico Derecha
const int ultrasonicoDerechaEcho = 52;
const int ultrasonicoDerechaTrig = 53;

// Ultrasonico Izquierda
const int ultrasonicoIzquierdaEcho = 22;
const int ultrasonicoIzquierdaTrig = 24;

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

// LED RGB
const int R = 9;
const int B = 10;
const int G = 11;

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
ColorSensing colorSensor;

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

// Orientación del robot; empieza al este.
// 0: Norte
// 1: Este
// 2: Sur
// 3: Oeste
int orientacion = 1;

// Setup pista C

// Color mapping.
vector<vector<string>> colorMap(3, vector<string> (5, ""));

// Walls
vector<vector<bool>> verticalWalls(3, vector<bool> (4, 0));
vector<vector<bool>> horizontalWalls(2, vector<bool> (5, 0));

// Set that holds the currently visited cells.
set<pair<int, int>> visited;

// Vector that maps the four directions —up, left, down, right— respectively.
vector<pair<int, int>> directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

// Adjacency list of the area.
map<pair<int, int>, set<pair<int, int>>> AL;

// Map containing the ocurrences of each color.
map<string, int> detectedColors;

// Robot always starts at (1, 4).
pair<int, int> currentPosition = {1, 4};

// Values based on "directions" vector
    enum Direction {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3
    };

bool checkWall(int direction, pair<int, int> currentNode) {
    // Cartesian notation for readbility's sake.
    int x = currentNode.first;
    int y = currentNode.second;

    if (direction == NORTH) {
        // Check horizontal wall at (x - 1, y).
        return horizontalWalls[x - 1][y];
    }
    else if (direction == EAST) {
        // Check vertical wall at (x, y - 1).
        return verticalWalls[x][y - 1];
    }
    else if (direction == SOUTH) {
        // Check horizontal wall at (x, y).
        return horizontalWalls[x][y];
    }
    else { // if (direction == WEST)
        // Check vertical wall at (x, y).
        return verticalWalls[x][y];
    }
    
}

// BONUS!
string findMostSeenColor(map<string, int> detectedColors) {
    // Iterate through the map of detected colors to find the most seen color.
    for (auto i : detectedColors) {
        if (i.second == 5) {
            return i.first;
        }
    }
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

    // BFS logic <3
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

// Movements to perform.
// 0: Forward
// 1: Right
// 2: Left
vector<vector<int>> steps = {{0}, {1, 0}, {1, 1, 0}, {2, 0}};

void handleMove(int movement) {
    if (i == FORWARD) {
        Motor::avanzar(100);
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

    enum Steps {
        FORWARD = 0,
        RIGHT = 1,
        LEFT = 2
    };

    if (nx == cx - 1) {
        for (auto i : steps[orientacion]) {
            handleMove(i);
        }
    }
    else if (ny == cy - 1) {
        for (auto i : steps[(orientacion + 1) % 4]) {
            handleMove(i);
        }
    }
    else if (nx == cx + 1) {
        for (auto i : steps[(orientacion + 2) % 4]) {
            handleMove(i);
        }
    }
    else if (ny == cy + 1) {
        for (auto i : steps[(orientacion + 3) % 4]) {
            handleMove(i);
        }
    }
}

void moveToNewPosition(pair<int, int> newPosition, pair<int, int>& currentPosition) {
    // Call bfs to get the path.
    map<pair<int, int>, pair<int, int>> parents = bfs(newPosition);

    while (parents[currentPosition] != currentPosition) {
        // Physically move towards the parent.
        // TODO: create function to do this.
        doMove(currentPostion, parents[currentPosition]);

        // Virtual test:
        currentPosition = parents[currentPosition];
        cout << currentPosition.first << " " << currentPosition.second << endl;
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
    // if (colorMap[node.first][node.second] == ""){
        // Find color based on samples.
        // map<string, int> detectedColorCount;
        
        // for (int i = 0; i < 10; i++) {
        //     detectedColorCount[ColorSensing::getColor()]++;
        //     delay(50);
        // }
        // string detectedColor = findMostSeenColor(detectedColorCount);
        // LEDRGB::setColor(detectedColor);

        // colorMap[node.first][node.second] = detectedColor;
        // detectedColors[detectedColor]++;
    // }

    // Keep going only if it's not a black square.
    if (colorMap[node.first][node.second] == "Black") {
        MovimientosLocos::GiroIzquierda();
        MovimientosLocos::GiroIzquierda();
        delay(1000);
        Motor::avanzar(100);
        delay(1000);
        Motor::detener();
        return;
    }
    else {
        Motor::avanzar(100);
        delay(1000);
        Motor::detener();
    }

    // If all cells are visited, move to checkpoint.
    if (visited.size() == 15) {
        moveToNewPosition({0, 0}, currentPosition);

        // girar(90); // to face the checkpoint.
        Motor::avanzar(100);
        delay(1000);

        string mostSeenColor = findMostSeenColor(detectedColors);
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
        if (Ultrasonico::distancia() < 10) continue;
        // Virutal (test):
        // bool wall = checkWall(i, node);
        // if (wall) continue;

        // If there's no wall, update adjacency list. 
        AL[node].insert({nx, ny});
        AL[{nx, ny}].insert(node);

        // Call dfs with new node only if it's not been visited yet.
        if (!visited.count({nx, ny})) {
            // Move in that direction, then call dfs.
            // adelante(15) o algo así
            dfs({nx, ny});
        }
    }
}

void setup() {
  
    // Ultrasonico Frontal
    pinMode(ultrasonicoFrontalEcho, INPUT);
    pinMode(ultrasonicoFrontalTrig, OUTPUT);

    // Ultrasonico Derecha
    pinMode(ultrasonicoDerechaEcho, INPUT);
    pinMode(ultrasonicoDerechaTrig, OUTPUT);
    
    // Ultrasonico Izquierda
    pinMode(ultrasonicoIzquierdaEcho, INPUT);
    pinMode(ultrasonicoIzquierdaTrig, OUTPUT);
    
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

string pista = "";

void loop() {
    
    long distanciaFrontal = distanciaUltrasonico(ultrasonicoFrontalTrig, ultrasonicoFrontalEcho);
    long distanciaDerecha = distanciaUltrasonico(ultrasonicoDerechaTrig, ultrasonicoDerechaEcho);
    long distanciaIzquierda = distanciaUltrasonico(ultrasonicoIzquierdaTrig, ultrasonicoIzquierdaEcho);


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
    
    if (pista == "") {
        string color = ColorSensing::getColor();

        if (color == "Green") {
            pista = "A";
        }
        else if (color == "Blue") {
            pista = "C";
        }
        else {
            pista = "B";
        }
    }

    if (pista == "C") {
        dfs(currentPosition);
    }

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

// Funcion para la distancia del ultrasonico
long distanciaUltrasonico(int trigPin, int echoPin) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long duracion = pulseIn(echoPin, HIGH);
        long distancia = duracion * 0.034 / 2;
        return distancia;
}