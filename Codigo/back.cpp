#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <map>

//#include "Ultrasonico.h"
//#include "ColorSensing.ino" // Is this even a thing...?

using namespace std;

vector<vector<string>> colorMap(3, vector<string> (5, ""));
// .....
// .....
// .....

vector<vector<bool>> verticalWalls(3, vector<bool> (4, 0));

void setVerticalWalls(){
    verticalWalls[0][0] = 0;
    verticalWalls[0][1] = 0;
    verticalWalls[0][2] = 0;
    verticalWalls[0][3] = 0;

    verticalWalls[1][0] = 0;
    verticalWalls[1][1] = 0;
    verticalWalls[1][2] = 0;
    verticalWalls[1][3] = 0;

    verticalWalls[2][0] = 1;
    verticalWalls[2][1] = 0;
    verticalWalls[2][2] = 0;
    verticalWalls[2][3] = 0;
    // These values translate to the following situation:
    // ....
    // ....
    // #...
}

vector<vector<bool>> horizontalWalls(2, vector<bool> (5, 0));

void setHorizontalWalls(){
    horizontalWalls[0][0] = 0;
    horizontalWalls[0][1] = 1;
    horizontalWalls[0][2] = 1;
    horizontalWalls[0][3] = 1;
    horizontalWalls[0][4] = 1;

    horizontalWalls[1][0] = 0;
    horizontalWalls[1][1] = 1;
    horizontalWalls[1][2] = 1;
    horizontalWalls[1][3] = 0;
    horizontalWalls[1][4] = 0;
    // These values translate to the following situation:
    // .####
    // .##..
}

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

bool checkWall(int direction, pair<int, int> currentNode) {
    // Values based on "directions" vector
    enum Direction {
        UP = 0,
        LEFT = 1,
        DOWN = 2,
        RIGHT = 3
    };

    // Cartesian notation for readbility's sake.
    int x = currentNode.first;
    int y = currentNode.second;

    if (direction == UP) {
        // Check horizontal wall at (x - 1, y).
        return horizontalWalls[x - 1][y];
    }
    else if (direction == LEFT) {
        // Check vertical wall at (x, y - 1).
        return verticalWalls[x][y - 1];
    }
    else if (direction == DOWN) {
        // Check horizontal wall at (x, y).
        return horizontalWalls[x][y];
    }
    else { // if (direction == RIGHT)
        // Check vertical wall at (x, y).
        return verticalWalls[x][y];
    }
    
}

int manhattanDistance(pair<int, int> cell1, pair<int, int> cell2) {
    return (abs(cell2.first - cell1.first) + abs(cell2.second - cell1.second));
}

map<pair<int, int>, pair<int, int>> bfs(pair<int, int> start) {
    // Inicializar estructuras necesarias.
    map<pair<int, int>, pair<int, int>> parents; // Guarda padres de cada nodo.
    queue<pair<int, int>> q; // Cola de procesamiento de nodos.
    map<pair<int, int>, int> dist; // Guarda las distancias a cada nodo.
    
    // Inicializar las estructuras a partir del nodo inicial.
    parents[start] = start; // Esto ayuda a reconocer el final, al ser el único que cumple parent[x] == x.
    q.push(start);
    dist[start] = 0;

    // BFS
    while (!q.empty()) {
        pair<int, int> u = q.front();
        q.pop();

        // Para cada uno de los nodos conectados a u.
        for (auto v : AL[u]) {
            // Revisión adicional de v == start porque no inicializamos en infinito, como de costumbre.
            if (dist[v] != 0 || v == start) continue;

            // Actualizar distancia más corta, añadir el nodo a la cola, y actualizar su padre.
            dist[v] = dist[u] + 1;
            q.push(v);
            parents[v] = u;
        }
    }

    // Retornar mapa de padres
    return parents;
}

void dfs(pair<int, int> node) {
    visited.insert(node);
    if (manhattanDistance(node, currentPosition) > 1) {
        cout << "Call BFS!" << endl;
        map<pair<int, int>, pair<int, int>> parents = bfs(node);

        while (parents[currentPosition] != currentPosition) {
            // Physically move towards the parent.

            // Virtual test:
            currentPosition = parents[currentPosition];
            cout << currentPosition.first << " " << currentPosition.second << endl;
        }
        cout << "BFS done." << endl;
    }
    cout << node.first << " " << node.second << endl;
    currentPosition = node;
    // Detect color in cell, show, and save.
    // string detectedColor = ColorSensing::getColor(); // Maybe adjust to take multiple samples and keep most frequent
    // LEDRGB::setColor(detectedColor);

    // colorMap[node.first][node.second] = detectedColor;
    // detectedColors[detectedColor]++;

    // Keep going only if it's not a black square.
    // if (detectedColor == "black") {
    //     atras(15);
    //     return
    // }
    // else {
    //     adelante(15);
    // }

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
        // if (Ultrasonico::medirDistancia() < 10) continue;
        // Virutal (test):
        bool wall = checkWall(i, node);

        if (wall) continue;

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

int main() {
    // Testing:
    setVerticalWalls();
    setHorizontalWalls();

    // DFS to reach (0,0), then BFS to know the path to reach the ones that are missing...
        // Counter cases? 
    
    dfs(currentPosition);

    for (auto i : AL) {
        cout << "{" << i.first.first << ", " << i.first.second << "}: ";
        for (auto j : i.second) {
            cout << "{" << j.first << ", " << j.second << "}, ";
        }
        cout << "\n";
    }
    // in it, call BFS to the missing ones to find the shortest distance.
    // for (int i = 0; i < 3; i++) {
    //     for (int j = 0; j < 4; j++) {
    //         if (!visited.count({i, j})) {
    //             vector<pair<int, int>> path = bfs({i, j});

    //             // Follow path and update current position.
    //         }
    //     }
    // }
}