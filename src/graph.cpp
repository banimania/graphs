#include "graph.hpp"
#include <raylib.h>

void Graph::addNode(float x, float y) {
  nodes.push_back(Node(nodes.size(), x, y));
  adj.push_back(vector<int>());
}

void Graph::addEdge(int u, int v) {
  adj[u].push_back(v);
  if (!directed) adj[v].push_back(u);
}

void Graph::drawGraph() {
  // Draw black edges
  for (int i = 0; i < adj.size(); i++) {
    Vector2 start = {nodes[i].x, nodes[i].y};
    for (int j = 0; j < adj[i].size(); j++) {      
      Vector2 end = {nodes[adj[i][j]].x, nodes[adj[i][j]].y};
      if (directed) {

        //fixear luego la arista y el triangulito :)

        float theta = atan2(nodes[adj[i][j]].y - nodes[i].y, nodes[adj[i][j]].x - nodes[i].x);
        Vector2 new_point = {-(nodeR - 2) * cos(theta) + nodes[adj[i][j]].x, -(nodeR - 2) * sin(theta) + nodes[adj[i][j]].y};
        Vector2 p1 = {-(nodeR + 10) * cos(theta) + nodes[adj[i][j]].x, -(nodeR +10) * sin(theta) + nodes[adj[i][j]].y};
        Vector2 p2 = {-(nodeR + 10) * cos(theta) + nodes[adj[i][j]].x, -(nodeR + 10) * sin(theta) + nodes[adj[i][j]].y};

        float alpha = -M_PI/2 + theta;
        p1.x -= -(10.0) * cos(alpha);
        p1.y -= -(10.0) * sin(alpha);

        p2.x += -(10.0) * cos(alpha);
        p2.y += -(10.0) * sin(alpha);
        
        DrawTriangle(p2, new_point, p1, BLACK);

        new_point = {-(nodeR + 10) * cos(theta) + nodes[adj[i][j]].x, -(nodeR + 10) * sin(theta) + nodes[adj[i][j]].y};
        Vector2 old_point = {(nodeR) * cos(theta) + nodes[i].x, (nodeR) * sin(theta) + nodes[i].y};

        DrawLineEx(old_point, new_point, 5, BLACK);
      } else {
        DrawLineEx(start, end, 5, BLACK);
      }
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
