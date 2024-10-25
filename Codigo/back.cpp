#include <iostream>
#include <vector>

using namespace std;

// Dimensions of the grid
const int ROWS = 3;
const int COLS = 5;

// Directions: up, down, left, right
const int dRow[] = {-1, 1, 0, 0};
const int dCol[] = {0, 0, -1, 1};

// Function to check if movement from (row, col) to (newRow, newCol) is valid
bool isValidMove(int row, int col, int newRow, int newCol, 
                 const vector<vector<bool>>& hWalls, const vector<vector<bool>>& vWalls, 
                 const vector<vector<bool>>& visited) {
    // Check if the new cell is within bounds
    if (newRow < 0 || newRow >= ROWS || newCol < 0 || newCol >= COLS || visited[newRow][newCol]) {
        return false;
    }
    
    // Check for horizontal walls
    if (newRow == row + 1 && hWalls[row][col]) return false; // Moving down
    if (newRow == row - 1 && hWalls[newRow][col]) return false; // Moving up
    
    // Check for vertical walls
    if (newCol == col + 1 && vWalls[row][col]) return false; // Moving right
    if (newCol == col - 1 && vWalls[row][newCol]) return false; // Moving left
    
    return true;
}

// Depth-first search function
void dfs(int row, int col, const vector<vector<bool>>& hWalls, const vector<vector<bool>>& vWalls, 
         vector<vector<bool>>& visited) {
    // Mark the current cell as visited
    visited[row][col] = true;
    cout << "Visited cell: (" << row << ", " << col << ")\n";

    // Explore all four directions
    for (int i = 0; i < 4; ++i) {
        int newRow = row + dRow[i];
        int newCol = col + dCol[i];
        if (isValidMove(row, col, newRow, newCol, hWalls, vWalls, visited)) {
            dfs(newRow, newCol, hWalls, vWalls, visited);
        }
    }
}

int main() {
    // Horizontal walls between rows (ROWS-1 x COLS grid)
    vector<vector<bool>> hWalls = {
        {false, false, true, false, false}, // Walls between row 0 and row 1
        {false, true, false, false, false}  // Walls between row 1 and row 2
    };

    // Vertical walls between columns (ROWS x COLS-1 grid)
    vector<vector<bool>> vWalls = {
        {false, false, false, true}, // Walls in row 0
        {false, true, false, false}, // Walls in row 1
        {false, false, true, false}  // Walls in row 2
    };

    // Visited array to keep track of visited cells
    vector<vector<bool>> visited(ROWS, vector<bool>(COLS, false));

    // Start DFS from position (1, 4)
    dfs(1, 4, hWalls, vWalls, visited);

    return 0;
}
