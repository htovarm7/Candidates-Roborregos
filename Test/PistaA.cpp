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

    ALgrid[{1, 2}].push_back({0, 2});
    ALgrid[{1, 2}].push_back({1, 1});
    ALgrid[{1, 2}].push_back({1, 3});
    ALgrid[{1, 2}].push_back({2, 2});

    ALgrid[{2, 2}].push_back({2, 1});
    ALgrid[{2, 2}].push_back({2, 3});
    
    ALgrid[{0, 3}].push_back({0, 2});
    ALgrid[{0, 3}].push_back({1, 3});

    ALgrid[{1, 3}].push_back({0, 3});
    ALgrid[{1, 3}].push_back({2, 3});
    ALgrid[{1, 3}].push_back({1, 4});

    ALgrid[{1, 3}].push_back({2, 2});
    ALgrid[{2, 3}].push_back({1, 3});
    
    ALgrid[{1, 4}].push_back({1, 3});
}

set<pair<int, int>> visited;

void bfs(pair<int, int> start, pair<int, int> end) {
    queue<pair<int, int>> 
}

void dfs(pair<int, int> node) {
    if (lineFound && ballFound) return;
    visited.insert(node);

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
        
        bfs(node, {1, 2});
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
        }

        dfs(v);
    }
}

int main() {
    fixGridAL();

    dfs({1, 0});
    return 0;
}