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

// Valores de constante
const int zero = 0;
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
    
    // Probar esto que no sé si jalaría, especialmente con lo del PID.
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