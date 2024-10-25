#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <map>

using namespace std;

vector<vector<string>> colorMap(3, vector<string> (5, ""));
// .....
// .....
// .....

vector<vector<bool>> verticalWalls(3, vector<bool> (4, 0));
// ####
// ####
// ####

vector<vector<bool>> horizontalWalls(2, vector<bool> (5, 0));
// #####
// #####

set<pair<int, int>> visited;

vector<pair<int, int>> d = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

map<pair<int, int>, vector<pair<int, int>>> AL;

void dfs(pair<int, int> node) {
    visited.insert(node);

    for (auto [dx, dy] : d) {
        int nx = node.first + dx;
        int ny = node.second + dy;

        if (nx < 0 || ny < 0 || nx > 2 || ny > 4) continue;
        // or wall

        if (visited.count({nx, ny})) {
            bfs({nx, ny});
        }
        else {
            AL[node].push_back({nx, ny});
            dfs({nx, ny});
        }
    }
}

vector<pair<int, int>> bfs(pair<int, int> start) {
    queue<pair<int, int>> q;
    map<pair<int, int>, int> dist;
    
    q.push(start);
    dist[start] = 0;

    while (!q.empty()) {
        pair<int, int> u = q.front();
        q.pop();

        for (auto v : AL[u]) {
            if (dist[v] != 0) continue;

            dist[v] = dist[u] + 1;
            q.push(v);
        }
    }

}


int main() {
    // DFS to reach (0,0), then BFS to know the path to reach the ones that are missing...
        // Counter cases? 
    
    dfs(1, 4);
    // in it, call BFS to the missing ones to find the shortest distance.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            if (!visited.count({i, j})) {
                vector<pair<int, int>> path = bfs({i, j});

                // Follow path and update current position.
            }
        }
    }
}