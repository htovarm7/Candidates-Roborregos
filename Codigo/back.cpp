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