#include <iostream>
#include <map>
#include <vector>
#include <set>
#include <queue>

using namespace std;

map<pair<int, int>, vector<pair<int, int>>> ALgrid;
bool ballFound = false;
bool lineFound = false;
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};
int orientacion = WEST;

void fixGridAL() {
    ALgrid[{1, 0}].push_back({1, 1});

    ALgrid[{1, 1}].push_back({1, 0});
    ALgrid[{1, 1}].push_back({0, 1});
    ALgrid[{1, 1}].push_back({2, 1});

    ALgrid[{0, 1}].push_back({1, 1});
    ALgrid[{0, 1}].push_back({0, 2});
    
    ALgrid[{2, 1}].push_back({1, 1});
    ALgrid[{2, 1}].push_back({2, 2});

    ALgrid[{0, 2}].push_back({0, 1});
    ALgrid[{0, 2}].push_back({0, 3});

    ALgrid[{2, 2}].push_back({2, 1});
    ALgrid[{2, 2}].push_back({2, 3});
    
    ALgrid[{0, 3}].push_back({0, 2});
    ALgrid[{0, 3}].push_back({1, 3});

    ALgrid[{1, 3}].push_back({0, 3});
    ALgrid[{1, 3}].push_back({2, 3});
    // ALgrid[{1, 3}].push_back({1, 4});

    ALgrid[{1, 3}].push_back({2, 2});
    ALgrid[{2, 3}].push_back({1, 3});
    
    ALgrid[{1, 4}].push_back({1, 3});
}

set<pair<int, int>> visited;

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
        for (auto v : ALgrid[u]) {
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
    cout << "BFS called.\n";
    map<pair<int, int>, pair<int, int>> parents = bfs(newPosition);

    while (parents[currentPosition] != currentPosition) {
        // Physically move towards the parent.
        // TODO: create function to do this.

        // Virtual test:
        currentPosition = parents[currentPosition];
        cout << currentPosition.first << " " << currentPosition.second << endl;
    }

    if (newPosition == make_pair(1, 2)) {
        ALgrid[{1, 3}].push_back({1, 4});
        ALgrid[{1, 4}].push_back({1, 3});
        moveToNewPosition({1, 4}, newPosition);
    }
}

void dfs(pair<int, int> node) {
    if (lineFound && ballFound) return;
    visited.insert(node);

    cout << node.first << " " << node.second << "\n";

    if (!ballFound){
        if (node == make_pair(1, 1)) {
            // if (ultrafront.getDistance() > 10) {
            //    ballFound = true;
            //    ALgrid[node].push_back({1, 2});
            //}
        }
        if (node == make_pair(0, 2)) {
            // if (ultraright.getDistance() > 10 || ultrafront.getDistance() > 10) {
            //    ballFound = true;
            //    ALgrid[node].push_back({1, 2});
            //}
            ballFound = true;
            ALgrid[node].push_back({1, 2});
            ALgrid[{1, 2}].push_back(node);
        }
        if (node == make_pair(1, 3)) {
            // if (ultraright.getDistance() > 10 || ultrafront.getDistance() > 10) {
            //    ballFound = true;
            //    ALgrid[node].push_back({1, 2});
            //}
        }
        if (node == make_pair(2, 2)) {
            // if (ultraright.getDistance() > 10 || ultrafront.getDistance() > 10) {
            //    ballFound = true;
            //    ALgrid[node].push_back({1, 2});
            //}
        }
    }

    if (lineFound && ballFound) { 
        moveToNewPosition({1, 2}, node);
        return;
    }

    for (auto v : ALgrid[node]) {
        if (visited.count(v)) continue;

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
            // if (sensorLineaD0 = 1 && ... D8), lineFound = true;
            if (v == make_pair(2,1)) {
                lineFound = true;
                if (lineFound && ballFound) { 
                    moveToNewPosition({1, 2}, node);
                    return;
                }
                continue;
            }
        }

        dfs(v);
    }

    
}

int main() {
    fixGridAL();

    dfs({1, 0});
    return 0;
}