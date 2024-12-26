#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>

using namespace std;

class Node {
public:
  int data;
  int weight;

  Node(int data, int weight) {
    this->data = data;
    this->weight = weight;
  }
};

class ListGraph {
private:
  unordered_map<int, vector<Node>> m;

public:
  void addEdge(int start, int dest, int weight) {
    m[start].emplace_back(dest, weight); //if cant find start vertex it will create new vertex
  }

  void delEdge(int start, int dest) {
    if (m.find(start) == m.end()) {  // Check if start vertex exists
      cout << "Start vertex not found!" << endl;
      return;
    }

    for (int i = 0; i < m[start].size(); i++) {
      if (m[start][i].data == dest) {
        m[start].erase(m[start].begin() + i);
        return;
      }
    }
    cout << "Edge not found!" << endl;
  }

  void print() {
    for (auto pair : m) { //use auto p instead of std::pair<std::string, std::vector<Node>> p
      cout << pair.first << "-> ";

      for (Node node : pair.second) {
        cout << "(" << node.data << ", " << node.weight << ") ";
      }
      cout << endl;
    }
  }

  //dfs
  void dfs(int start) {
    unordered_map<int, bool> visited;
    dfsHelper(start, visited);
  }

  void dfsHelper(int current, unordered_map<int, bool> visited) {
    visited[current] = true;
    std::cout << current << " ";

    for (Node neighbor : m[current]) {
      if (!visited[neighbor.data]) {
        dfsHelper(neighbor.data, visited);
      }
    }
  }
  //dfs

  //bfs
  void bfs(int start) {
    unordered_map<int, bool> visited;
    queue<int> q;

    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
      int current = q.front();
      q.pop();
      cout << current << " ";

      for (Node neighbor : m[current]) {
        if (!visited[neighbor.data]) {
          visited[neighbor.data] = true;
          q.push(neighbor.data);
        }
      }
    }
  }
  //bfs

};

int main() {
  ListGraph graph;

  // Adding edges
  graph.addEdge(1, 2, 10);
  graph.addEdge(1, 3, 20);
  graph.addEdge(2, 4, 30);
  graph.addEdge(3, 4, 40);

  cout << "Graph after adding edges:\n";
  graph.print();

  // Deleting an edge
  graph.delEdge(1, 3); // Deleting an existing edge
  cout << "\nGraph after deleting edge (1 -> 3):\n";
  graph.print();

  // Attempting to delete a non-existing edge
  graph.delEdge(1, 5); // Edge not found

  // Adding more edges to test dynamic behavior
  graph.addEdge(5, 6, 50);
  cout << "\nGraph after adding edge (5 -> 6):\n";
  graph.print();

  return 0;
}

// for (auto pair = myMap.begin(); pair != myMap.end(); pair++) { //another way to loop, using iterator
//  something
// }