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
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include "ColorConverterLib.h"
#include "Ultrasonic.h"
#include "Motors.h"

/* CONSTANTS */

// Pines ultrasonico Izquierdo
const int leftEcho = 33;
const int leftTrig = 32;

// Pines ultrasonico Frontal
const int frontEcho = 35;
const int frontTrig = 34;

// Pines ultrasonico Derecha
const int rightEcho = 37;
const int rightTrig = 36;

// Motores
//  Pines motor superior izquierdo
const int IN1_SI = 45;
const int IN2_SI = 44;
const int ENA_SI = 5;
const int ENC_A_SI = 25;
const int ENC_B_SI = 24;

// Pines motor superior derecho
const int IN1_SD = 43;
const int IN2_SD = 42;
const int ENB_SD = 4;
const int ENC_A_SD = 29;
const int ENC_B_SD = 28;

// Pines motor inferior izquierdo
const int IN1_II = 41;
const int IN2_II = 40;
const int ENA_II = 3;
const int ENC_A_II = 27;
const int ENC_B_II = 26;

// Pines motor inferior derecho
const int IN1_ID = 39;
const int IN2_ID = 38;
const int ENB_ID = 2;
const int ENC_A_ID = 31;
const int ENC_B_ID = 30;

const int pwmIzq = 255;
const int pmwDer = 255;

// Actuadores

// Servomotores
const int Servo_SI = 22;
const int Servo_SD = 23;

// Sensor de color
const char SCL_COLOR = A5;
const char SDA_COLOR = A4; 

// Giroscopio
// const char SCL_GIRO = A2;
// const char SDA_GIRO = A3;

// LED RGB Falta poner los pines al arduino
const int R = 6;
const int G = 8;
const int B = 7;

// Gyroscopes (burnt, my bad)
// const int giroSCL = 1; 
// const int giroSDA = 2; 

// Line sensor
const int lineSensorD8 = 46;
const int lineSensorD7 = 47;
const int lineSensorD6 = 48;
const int lineSensorD5 = 49;
const int lineSensorD4 = 50;
const int lineSensorD3 = 51;
const int lineSensorD2 = 52;
const int lineSensorD1 = 53;

// Servos
const int servo1 = 22;
const int servo2 = 23;

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

// Servos.
Servo Servo1;
Servo Servo2;

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

// Robot's current orientation.
int orientation;

/// Track A setup

// Grid for the area.
std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> ALA;

// Control variables.
bool ballFound = false;
bool lineFound = false;

// Fixes the adjacency list according to what we can know.
// This circumvents the use of directions, yet it's not hardcoding!!
void fixGridAL() {
    ALA[{1, 0}].push_back({1, 1});

    ALA[{1, 1}].push_back({1, 0});
    ALA[{1, 1}].push_back({0, 1});
    ALA[{1, 1}].push_back({2, 1});

    ALA[{0, 1}].push_back({1, 1});
    ALA[{0, 1}].push_back({0, 2});
    
    ALA[{2, 1}].push_back({1, 1});
    ALA[{2, 1}].push_back({2, 2});

    ALA[{0, 2}].push_back({0, 1});
    ALA[{0, 2}].push_back({0, 3});

    ALA[{2, 2}].push_back({2, 1});
    ALA[{2, 2}].push_back({2, 3});
    
    ALA[{0, 3}].push_back({0, 2});
    ALA[{0, 3}].push_back({1, 3});

    ALA[{1, 3}].push_back({0, 3});
    ALA[{1, 3}].push_back({2, 3});
    // ALA[{1, 3}].push_back({1, 4}); // This needs to be "unlocked"

    ALA[{2, 3}].push_back({2, 2});
    ALA[{2, 3}].push_back({1, 3});
    
    ALA[{1, 4}].push_back({1, 3});
}

std::set<std::pair<int, int>> visitedA;

/// Track C setup.

// Color mapping.
std::vector<std::vector<std::string>> colorMap(3, std::vector<std::string> (5, ""));

// Walls
std::vector<std::vector<bool>> verticalWalls(3, std::vector<bool> (4, 0));
std::vector<std::vector<bool>> horizontalWalls(2, std::vector<bool> (5, 0));

// std::Set that holds the currently visited cells.
std::set<std::pair<int, int>> visitedC;

// Adjacency list of the area.
std::map<std::pair<int, int>, std::set<std::pair<int, int>>> ALC;

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

void openDome(){
    Servo1.write(60);
    Servo2.write(120);
}

void closeDome(){
    Servo1.write(150); 
    Servo2.write(30);
}

void showColor(String color){
    if(color == "Yellow"){
        analogWrite(R,0);
        analogWrite(G,0);
        analogWrite(B,255);
    }else if(color == "Black"){
        analogWrite(R,0);
        analogWrite(G,0);
        analogWrite(B,0);
    }else if(color == "Purple"){
        analogWrite(R,184);
        analogWrite(G,37);
        analogWrite(B,174);
    }else if(color == "Blue"){
        analogWrite(R,0);
        analogWrite(G,0);
        analogWrite(B,255);
    }else if(color == "Pink"){
        analogWrite(R,255);
        analogWrite(G,23);
        analogWrite(B,185);
    }else if(color == "Red"){
        analogWrite(R,255);
        analogWrite(G,0);
        analogWrite(B,0);
    }else if(color == "Green"){
        analogWrite(R,0);
        analogWrite(G,0);
        analogWrite(B,255);
    }else{
        analogWrite(R,255);
        analogWrite(G,143);
        analogWrite(B,23);
    }
}

void handleMove(int movement) {
    if (movement == FORWARD) {
        myMotors.forward();
    }
    else if (movement == RIGHT) {
        myMotors.turnRight();
    }
    else if (movement == LEFT) {
        myMotors.turnLeft();
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

std::map<std::pair<int, int>, std::pair<int, int>> bfsA(std::pair<int, int> start) {
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
        for (auto v : ALA[u]) {
            // Additional check on v == start as we don't initialize all distances on infinity, as per usual in BFS.
            if (dist[v] != 0 || v == start) continue;

            // Update shortest distance, add node to std::queue, and update its parent.
            parents[v] = u;
            q.push(v);
            dist[v] = dist[u] + 1;
        }
    }

    // Return parent map.
    return parents;
}


std::map<std::pair<int, int>, std::pair<int, int>> bfsC(std::pair<int, int> start) {
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
        for (auto v : ALC[u]) {
            // Additional check on v == start as we don't initialize all distances on infinity, as per usual in BFS.
            if (dist[v] != 0 || v == start) continue;

            // Update shortest distance, add node to std::queue, and update its parent.
            parents[v] = u;
            q.push(v);
            dist[v] = dist[u] + 1;
        }
    }

    // Return parent map.
    return parents;
}

void moveToNewPositionC(std::pair<int, int> newPosition, std::pair<int, int>& currentPosition) {
    // Call bfs to get the path.
    std::map<std::pair<int, int>, std::pair<int, int>> parents = bfsC(newPosition);

    while (parents[currentPosition] != currentPosition) {
        // Physically move towards the parent.
        doMove(currentPosition, parents[currentPosition]);
        currentPosition = parents[currentPosition];
    }
}

void moveToNewPositionA(std::pair<int, int> newPosition, std::pair<int, int>& currentPosition) {
    // Call bfs to get the path.
    std::map<std::pair<int, int>, std::pair<int, int>> parents = bfsA(newPosition);

    while (parents[currentPosition] != currentPosition) {
        // Physically move towards the parent.
        doMove(currentPosition, parents[currentPosition]);
        currentPosition = parents[currentPosition];
    }

    if (newPosition == std::make_pair(1, 2)) {
        openDome();
        closeDome();
        ALA[{1, 3}].push_back({1, 4});
        ALA[{1, 4}].push_back({1, 3});
        moveToNewPositionA({1, 4}, newPosition);
    }
}

void dfsA(std::pair<int, int> node) {
    if (lineFound && ballFound) return;
    visitedA.insert(node);

    if (!ballFound){
        if (node == std::make_pair(1, 1)) {
            if (frontUltrasonic.getDistance() > 10) {
                ballFound = true;
                ALA[{1, 2}].push_back(node);
                ALA[node].push_back({1, 2});
            }
        }
        if (node == std::make_pair(0, 2) || node == std::make_pair(1, 3) || node == std::make_pair(2, 2)) {
            if (rightUltrasonic.getDistance() > 10) {
                ballFound = true;
                ALA[node].push_back({1, 2});
                ALA[{1, 2}].push_back(node);
            }
        }
    }

    if (lineFound && ballFound) { 
        // Regresar a donde está la pelota, sabiendo que está en {1, 2};
        moveToNewPositionA({1, 2}, node);
        return;
    }

    for (auto v : ALA[node]) {
        if (visitedA.count(v)) continue;

        // Move according to current position and next position.
        // NORTH
        // NORTH = -
        // EAST = left turn
        // WEST = right turn

        // EAST
        // NORTH = right turn
        // EAST = -
        // SOUTH = left turn

        // SOUTH
        // EAST = right turn
        // SOUTH = -
        // WEST = left turn

        // WEST 
        // NORTH = left turn
        // WEST = -
        // SOUTH = right turn

        if (!lineFound) {
            // If 5/8 sensors sense black, a line is detected.
            int lineCounter = 0;
            int valueD1 = digitalRead(lineSensorD1);
            int valueD2 = digitalRead(lineSensorD2);
            int valueD3 = digitalRead(lineSensorD3);
            int valueD4 = digitalRead(lineSensorD4);
            int valueD5 = digitalRead(lineSensorD5);
            int valueD6 = digitalRead(lineSensorD6);
            int valueD7 = digitalRead(lineSensorD7);
            int valueD8 = digitalRead(lineSensorD8);

            std::vector<int> lineValues = {valueD1, valueD2, valueD3, valueD4, valueD5, valueD6, valueD7, valueD8};
            
            for (auto i : lineValues) {
                if (i) lineCounter++;
            }

            if (v == std::make_pair(2,1)) {
                lineFound = true;
                for (auto it = ALA[{2, 1}].begin(); it != ALA[{2, 1}].end(); it++) {
                    if (*it == node) {
                        ALA[{2, 1}].erase(it);
                        break;
                    }
                }
                if (lineFound && ballFound) { 
                    moveToNewPositionA({1, 2}, node);
                    return;
                }
                continue;
            }
        }

        dfsA(v);
    } 
}

void dfsC(std::pair<int, int> node) {
    visitedC.insert(node);
    if (ALC[node].find(currentPosition) == ALC[node].end()) {
        Serial.println("Call BFS!");
        moveToNewPositionC(node, currentPosition);

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
        myMotors.turnLeft();
        myMotors.turnLeft();
        delay(1000);
        myMotors.forward();
        delay(800);
        myMotors.stop();
        return;
    }
    else {
        myMotors.forward();
        delay(800);
        myMotors.stop();
    }

    // If all cells are visited, move to checkpoint.
    if (visitedC.size() == 15) {
        moveToNewPositionC({0, 0}, currentPosition);

        // Face the checkpoint.
        if (orientation == NORTH) {
            myMotors.turnLeft();
        }
        myMotors.forward();
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

        // If there's a wall, skip.
        // Physical:
        if (i == 0){
            if (frontUltrasonic.getDistance() < 10) continue;
        }
        else if (i == 1) {
            if (leftUltrasonic.getDistance() < 10) continue;
        }
        else if (i == 2) {
            continue;
        }
        else {
            if (rightUltrasonic.getDistance() < 10) continue;
        }

        // If there's no wall, update adjacency list. 
        ALC[node].insert({nx, ny});
        ALC[{nx, ny}].insert(node);

        // Call dfs with new node only if it's not been visited yet.
        if (!visitedC.count({nx, ny})) {
            // Move in that direction, then call dfs.
            if (i == 0) {
                myMotors.forward();
                delay(1000);
            }
            else if (i == 1) {
                myMotors.turnLeft();
                myMotors.forward();
                delay(1000);
            }
            else {
                myMotors.turnRight();
                myMotors.forward();
            }
            myMotors.stop();

            dfsC({nx, ny});
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
    Serial.println(hue);

    // Return color based on tested values.
    if (hue > 30 && hue < 50) {
        return "Yellow";
    }
    else if (hue > 55 && hue < 65) {
        return "Black";
    }
    else if (hue > 222 && hue < 265) {
        return "Purple";
    }
    else if (hue > 195 && hue < 222) {
        return "Blue";
    }
    else if (hue > 300 && hue < 360) {
        return "Pink";
    }
    else if (hue > 0 && hue < 25) {
        return "Red";
    }
    else if (hue > 85 && hue < 110) {
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

    // Ensure color sensor is on.
    while (!tcs.begin()) delay(1000);
    
    // Servos
    Servo1.attach(Servo_SI);
    Servo2.attach(Servo_SD);

    // RGB LED
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
    
    // Line Sensor
    pinMode(lineSensorD8, INPUT);
    pinMode(lineSensorD7, INPUT);
    pinMode(lineSensorD6, INPUT);
    pinMode(lineSensorD5, INPUT);
    pinMode(lineSensorD4, INPUT);
    pinMode(lineSensorD3, INPUT);
    pinMode(lineSensorD2, INPUT);
    pinMode(lineSensorD1, INPUT);
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
    if (track == "A") {
        orientation == WEST;
        currentPosition = {1, 0};
        dfsA(currentPosition);
    }
    else if (track == "C") {
        orientation == EAST;
        currentPosition = {1, 4};
        dfsC(currentPosition);
    }

    // adelante();
    // detener();
    // giroDerecha();
    // giroIzquierda();
    // reversa();
    // movLateral();
}