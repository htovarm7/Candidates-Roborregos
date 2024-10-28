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

void moveToNewPosition(pair<int, int> newPosition, pair<int, int>& currentPosition) {
    // Call bfs to get the path.
    map<pair<int, int>, pair<int, int>> parents = bfs(newPosition);

    while (parents[currentPosition] != currentPosition) {
        // Physically move towards the parent.
        // TODO: create function to do this.

        // Virtual test:
        currentPosition = parents[currentPosition];
        cout << currentPosition.first << " " << currentPosition.second << endl;
    }
}

void dfs(pair<int, int> node) {
    visited.insert(node);
    if (manhattanDistance(node, currentPosition) > 1) {
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
    // if (colorMap[node.first][node.second] == "black") {
    //     atras(15);
    //     return
    // }
    // else {
    //     adelante(15);
    // }

    // If all cells are visited, move to checkpoint.
    if (visited.size() == 15) {
        moveToNewPosition({0, 0}, currentPosition);

        // girar(90); // to face the checkpoint.
        // avanzar(30);

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

    // DFS to traverse the maze, using BFS to reach other cells.
    dfs(currentPosition);

    for (auto i : AL) {
        cout << "{" << i.first.first << ", " << i.first.second << "}: ";
        for (auto j : i.second) {
            cout << "{" << j.first << ", " << j.second << "}, ";
        }
        cout << "\n";
    }
}