#include "graph.hpp"

void Graph::addNode(float x, float y) {
  nodes.push_back(Node(nodes.size(), x, y));
  adj.push_back(vector<int>());
}

void Graph::addEdge(int u, int v) {
  adj[u].push_back(v);
  adj[v].push_back(u);
}

void Graph::drawGraph() {
  // Draw black edges
  for (int i = 0; i < adj.size(); i++) {
    Vector2 start = {nodes[i].x, nodes[i].y};
    for (int j = 0; j < adj[i].size(); j++) {
      Vector2 end = {nodes[adj[i][j]].x, nodes[adj[i][j]].y};
      DrawLineEx(start, end, 5, BLACK);
    }
  }

  // Draw orange edges
  for(Edge e : markedEdges) {
    e.drawEdge(Vector2{nodes[e.u].x, nodes[e.u].y}, Vector2{nodes[e.v].x, nodes[e.v].y});
  }

  for (int i = 0; i < nodes.size(); i++) {
    nodes[i].drawNode();
  }
}

void Graph::restartAlgorithms() {
  startedDFS = false;
  startedBFS = false;

  dfsStack = stack<int>();
  bfsQueue = queue<int>();

  markedEdges.clear();

  for (int i = 0; i < nodes.size(); i++) {
    nodes[i].marked = false;
  }
}

void Graph::dfsStep() {
  // Make sure we always actually visit a new node
  while (!dfsStack.empty()) {
    int current = dfsStack.top();
    for (int neighbour : adj[current]) {
      if (!nodes[neighbour].marked) {
        nodes[neighbour].marked = true;
        markedEdges.push_back(Edge(current, neighbour));
        dfsStack.push(neighbour);
        return; // Exit after adding a new node
      }
    }
    // If all neighbors are visited, pop the current node and continue
    dfsStack.pop();
  }
}

void Graph::bfsStep() {
  if (bfsQueue.empty()) return;

  std::queue<int> currentQueue;

  // Move all nodes from bfsQueue to currentQueue
  while (!bfsQueue.empty()) {
    currentQueue.push(bfsQueue.front());
    bfsQueue.pop();
  }

  // Process all nodes in currentQueue
  while (!currentQueue.empty()) {
    int current = currentQueue.front();
    currentQueue.pop();

    for (int neighbour : adj[current]) {
      if (!nodes[neighbour].marked) {
        nodes[neighbour].marked = true;
        markedEdges.push_back(Edge(current, neighbour));
        bfsQueue.push(neighbour); // These will form the next steps queue
      }
    }
  }
}

int Graph::getNode(float x, float y) {
  for (int i = 0; i < nodes.size(); i++) {
    if (CheckCollisionPointCircle({x, y}, {nodes[i].x, nodes[i].y}, nodeR)) {
      return i;
    }
  }
  return -1;
}

void Graph::removeNode(int id) {
  for (int i = 0; i < adj.size(); i++) {
    for (auto it = adj[i].begin(); it != adj[i].end(); ) {
      if (*it == id) it = adj[i].erase(it);
      else {
        if (*it > id) (*it)--;
        ++it;
      }
    }
  }
  nodes.erase(nodes.begin() + id);
  adj.erase(adj.begin() + id);
  
  for (int i = id; i < nodes.size(); i++) {
    nodes[i].id--;
  }
}
