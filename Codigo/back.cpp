#include <bits/stdc++.h>
using namespace std;

#define ln "\n"
#define fast_cin() \
    ios_base::sync_with_stdio(false); \
    cin.tie(NULL)
#define iofiles() \
    freopen("input.txt", "r", stdin); \
    freopen("output.txt", "w", stdout)
#define dbg(...) __f(#__VA_ARGS__, __VA_ARGS__)
template <typename Arg1>
void __f(const char *name, Arg1 &&arg1) { cout << name << ": " << arg1 << endl; }
template <typename Arg1, typename... Args>
void __f(const char *names, Arg1 &&arg1, Args &&... args) {
    const char *comma = strchr(names + 1, ',');
    cout.write(names, comma - names) << ": " << arg1 << " |";
    __f(comma + 1, args...);
}

#define ll long long
#define ld long double
#define pb push_back
#define INF 2e18
#define PI 3.14159265358979323846
#define MOD 1000000007

double eps = 1e-9;

set<pair<int, int>> visited;

vector<pair<int, int>> d = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

void backtrack(vector<vector<int>>& maze, pair<int, int> curpos) {
    if (curpos == make_pair(0, 0) && visited.size() >= 15) {
        cout << "Success!" << ln;
        return;
    }

    for (auto [dx, dy] : d) {
        int nx = curpos.first + dx;
        int ny = curpos.second + dy;

        if (nx < 0 || ny < 0 || nx > 2 || ny > 4 || visited.count({nx, ny})) continue;

        visited.insert({nx, ny});
        backtrack(maze, {nx, ny});
        visited.erase({nx, ny});
    }

}

int main() {
    fast_cin();
    //iofiles();

    vector<vector<int>> maze(3, vector<int> (5, 0));

    pair<int, int> curpos = {1, 4};
    visited.insert(curpos);
    backtrack(maze, curpos);

    return 0;
}