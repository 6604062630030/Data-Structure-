#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <climits>

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

class Graph {
public:
    unordered_map<int, vector<Node>> g;

    // Add undirected edge to the graph
    void addEdge(int start, int end, int weight) {
        g[start].emplace_back(end, weight);
        g[end].emplace_back(start, weight);
    }

    // Dijkstra's algorithm to find shortest path from a source node
    void dijkstra(int source) {
        unordered_map<int, int> dist;
        for (const auto& pair : g) {
            dist[pair.first] = INT_MAX;  // Initialize all distances to infinity
        }
        dist[source] = 0; //(the node from which Dijkstra starts)

        // Priority queue to pick the node with the smallest distance
        // The priority queue stores a pair (distance, node)
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, source});

        while (!pq.empty()) {
            int distance = pq.top().first;
            int node = pq.top().second;
            pq.pop();

            // If the distance in the queue is already larger, skip it
            if (distance > dist[node]) {
                continue;
            }

            // Process each adjacent node
            for (const Node& neighbor : g[node]) {
                int nextNode = neighbor.data;
                int weight = neighbor.weight;

                // Relax the edge (node -> nextNode)
                if (dist[node] + weight < dist[nextNode]) {
                    dist[nextNode] = dist[node] + weight;
                    pq.push({dist[nextNode], nextNode});
                }
            }
        }

        // Output the shortest distances from the source node
        for (const auto& d : dist) {
            cout << "Node " << d.first << " has distance: " << d.second << endl;
        }
    }
};

int main() {
    Graph g;
    // Adding edges to the graph
    g.addEdge(1, 2, 4);
    g.addEdge(1, 3, 1);
    g.addEdge(3, 2, 2);
    g.addEdge(2, 4, 5);
    g.addEdge(3, 4, 8);

    // Run Dijkstra starting from node 1
    g.dijkstra(1);

    return 0;
}
