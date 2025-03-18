#include "graph.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <raylib.h>
#include <sstream>
#include <string>
#include <limits>
#define M_PI 3.141592

void Graph::addNode(float x, float y) {
  nodes.push_back(Node(nodes.size(), x, y));
  adj.push_back(vector<pair<int, float>>());
}

void Graph::addEdge(int u, int v, float c) {
  adj[u].push_back({v, c});
  if (!directed) adj[v].push_back({u, c});
}

void Graph::drawEdgeWeight(Vector2 from, Vector2 to, float cost, bool directed) {
  float theta = atan2(to.y - from.y, to.x - from.x);
  if (!directed && theta <= 0) theta += M_PI;
  float alpha = -M_PI / 2 + theta;

  Vector2 pos = {
    (to.x + from.x) / 2.0f + cos(alpha) * 30.0f, 
    (to.y + from.y) / 2.0f + sin(alpha) * 30.0f
  };

  string costString = formatNum(cost);

  pos.x -= MeasureTextEx(font, costString.c_str(), 30, 0.0f).x / 2.0f;
  pos.y -= MeasureTextEx(font, costString.c_str(), 30, 0.0f).y / 2.0f;

  DrawTextEx(font, costString.c_str(), pos, 30.0f, 0.0f, BLACK);
}

void Graph::drawDirectedEdge(Vector2 from, Vector2 to, bool adjustFrom, bool adjustTo, Color color, float thickness) {
  float theta = atan2(to.y - from.y, to.x - from.x);

  // Draw arrowhead
  Vector2 arrowTip = {-(nodeR - 1) * cos(theta) + to.x, -(nodeR - 1) * sin(theta) + to.y};
  Vector2 leftWing = {-(nodeR + 10) * cos(theta) + to.x, -(nodeR + 10) * sin(theta) + to.y};
  Vector2 rightWing = leftWing;

  float alpha = -M_PI / 2 + theta;

  leftWing.x -= -(10.0f) * cos(alpha);
  leftWing.y -= -(10.0f) * sin(alpha);

  rightWing.x += -(10.0f) * cos(alpha);
  rightWing.y += -(10.0f) * sin(alpha);

  DrawTriangle(rightWing, arrowTip, leftWing, color);

  // Adjust points conditionally (like original)
  Vector2 adjustedFrom = from;
  Vector2 adjustedTo = to;

  if (adjustTo)
      adjustedTo = {-(nodeR + 10) * cos(theta) + to.x, -(nodeR + 10) * sin(theta) + to.y};

  if (adjustFrom)
      adjustedFrom = {(nodeR + 10) * cos(theta) + from.x, (nodeR + 10) * sin(theta) + from.y};

  DrawLineEx(adjustedFrom, adjustedTo, thickness, color);
}

void Graph::drawMarkedEdges() {
  for (const Edge &e : markedEdges) {
    Vector2 start = {nodes[e.u].x, nodes[e.u].y};
    Vector2 end = {nodes[e.v].x, nodes[e.v].y};

    if (directed) {
      bool adjustFrom = areNeighbours(e.v, e.u);
      bool adjustTo = areNeighbours(e.u, e.v);
      drawDirectedEdge(start, end, adjustFrom, adjustTo, ORANGE);
    } else {
      DrawLineEx(start, end, 5, ORANGE);
    }
  }
}

void Graph::drawGraph() {
  // Draw black edges
  for (size_t i = 0; i < adj.size(); i++) {
    Vector2 start = {nodes[i].x, nodes[i].y};
    for (size_t j = 0; j < adj[i].size(); j++) {
      Vector2 end = {nodes[adj[i][j].first].x, nodes[adj[i][j].first].y};

      if (weighted) {
        drawEdgeWeight(start, end, adj[i][j].second, directed);
      }    

      if (directed) {
        bool adjustFrom = areNeighbours(adj[i][j].first, i);
        bool adjustTo = areNeighbours(i, adj[i][j].first);
        drawDirectedEdge(start, end, adjustFrom, adjustTo, BLACK);
      } else {
        DrawLineEx(start, end, 5, BLACK);
      }
    }
  }

  // Draw orange edges
  drawMarkedEdges();

  // Draw nodes
  for (size_t i = 0; i < nodes.size(); i++) {
    nodes[i].drawNode();
  }
}

void Graph::resetSearchStates() {
  for (auto &node : nodes) {
      node.marked = false;
  }
  markedEdges.clear();
}

void Graph::resetDFS() {
  startedDFS = false;
  dfsStack = stack<int>();
}

void Graph::resetBFS() {
  startedBFS = false;
  bfsQueue = queue<int>();
}

void Graph::resetDijkstra() {
  startedDijkstra = false;
  bestDist.assign(nodes.size(), numeric_limits<float>::infinity());
  dijkstraPrev.assign(nodes.size(), -1);

  dijkstraPq = priority_queue<
      pair<int, float>, 
      vector<pair<int, float>>, 
      function<bool(pair<int, float>, pair<int, float>)>
  >([](const pair<int, float>& a, const pair<int, float>& b) {
      return a.second > b.second;
  });
}

void Graph::resetKruskal() {
  startedKruskal = false;
  kruskalPq = priority_queue<Edge, vector<Edge>, function<bool(Edge, Edge)>>([](const Edge& a, const Edge &b) {
    return a.cost > b.cost;
  });
  
  for (size_t i = 0; i < adj.size(); i++) {
    for (auto [j, cost] : adj[i]) {
      kruskalPq.push({(int) i, j, cost});
    }
  }

  kruskalCost = 0;

  parent.assign(nodes.size(), 0);
  for (size_t i = 0; i < nodes.size(); i++) parent[i] = i;
  setSize.assign(nodes.size(), 1);
}

void Graph::resetHierholzer() {
  startedHierholzer = false;
  hierholzerStack = stack<int>();
  eulerianPath = vector<int>();
}

int Graph::findSet(int v) {
  if (v == parent[v]) return v;
  return parent[v] = findSet(parent[v]);
}

bool Graph::unionSet(int a, int b) {
  a = findSet(a);
  b = findSet(b);
  if (a != b) {
    if (setSize[a] < setSize[b]) swap(a, b);
    parent[b] = a;
    setSize[a] += setSize[b];
    return true;
  } 
  return false;
}

void Graph::calculateDSU() {
  parent.assign(nodes.size(), 0);
  for (size_t i = 0; i < nodes.size(); i++) parent[i] = i;
  setSize.assign(nodes.size(), 1);

  for (size_t i = 0; i < adj.size(); i++){
    for (auto [v, cost] : adj[i]){
      unionSet(i, v);
    }
  }
}

void Graph::calculateDegrees() {
  inDegree.assign(nodes.size(), 0);
  outDegree.assign(nodes.size(), 0);
  for (auto [u, v, cost] : edges) {
    inDegree[v]++;
    outDegree[u]++;
  }
}

int Graph::checkDegrees() {
  calculateDegrees();
  int inDegOne = 0;
  int outDegOne = 0;
  bool flag = true;
  int oddDegree = 0;
  int source = 0;
  for (size_t i = 0; i < nodes.size(); i++) {
    if (directed) {
      if (inDegree[i] - outDegree[i] == 1) inDegOne++;
      else if (outDegree[i] - inDegree[i] == 1) {
        outDegOne++;
        source = i;
      }
      else if (inDegree[i] != outDegree[i]) flag = false;
    } else {
      if (inDegree[i] % 2 == 1) {
        oddDegree++;
        source = i;
      }
    }
  }
  if (directed) {
    if (inDegOne == 1 && outDegOne == 1 && flag) return source;
    else return -1;
  } else {
    if (oddDegree == 0 || oddDegree == 2) return source;
    else return -1;
  }
}

void Graph::restartAlgorithms() {
  resetDFS();
  resetBFS();
  resetDijkstra();
  resetSearchStates();
  calculateDSU();
  resetKruskal();
  resetHierholzer();
}

void Graph::dfsStep() {
  // Make sure we always actually visit a new node
  while (!dfsStack.empty()) {
    int current = dfsStack.top();
    for (auto [neighbour, cost] : adj[current]) {
      if (!nodes[neighbour].marked) {
        nodes[neighbour].marked = true;
        markedEdges.push_back(Edge(current, neighbour, cost));
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

  queue<int> currentQueue;

  // Move all nodes from bfsQueue to currentQueue
  while (!bfsQueue.empty()) {
    currentQueue.push(bfsQueue.front());
    bfsQueue.pop();
  }

  // Process all nodes in currentQueue
  while (!currentQueue.empty()) {
    int current = currentQueue.front();
    currentQueue.pop();

    for (auto [neighbour, cost] : adj[current]) {
      if (!nodes[neighbour].marked) {
        nodes[neighbour].marked = true;
        markedEdges.push_back(Edge(current, neighbour, cost));
        bfsQueue.push(neighbour); // These will form the next steps queue
      }
    }
  }
}

void Graph::dijkstraStep() {
  if (dijkstraPq.empty()) return;

  bool flag = 0;

  while (!dijkstraPq.empty()) {
    pair<int, float> current = dijkstraPq.top();

    dijkstraPq.pop();

    if (current.second > bestDist[current.first]) continue;

    for (auto [neighbour, cost] : adj[current.first]) {
      float newDist = current.second + cost;
      if (newDist < bestDist[neighbour]) {
        bestDist[neighbour] = newDist;
        markedEdges.push_back(Edge(current.first, neighbour, cost));
        nodes[neighbour].marked = true;
        dijkstraPrev[neighbour] = current.first;
        dijkstraPq.push({neighbour, newDist});
        flag = 1;
      }
    }
    if (flag) return;
  }
}

vector<int> Graph::getDijkstraPath(int u, int v) {
  vector<int> path;

  int current = v;
  while (current != -1) {
    path.push_back(current);
    current = dijkstraPrev[current];
  }
  reverse(path.begin(), path.end());
  return path;
}

void Graph::kruskalStep() {
  if (kruskalPq.empty()) return;

  bool added = false;

  while (!added && !kruskalPq.empty()) {
    Edge currentEdge = kruskalPq.top();
    kruskalPq.pop();

    if (findSet(currentEdge.u) != findSet(currentEdge.v)) {
      unionSet(currentEdge.u, currentEdge.v);
      kruskalCost += currentEdge.cost;
      markedEdges.push_back(currentEdge);
      added = true;
    }
  }
}

/*void Graph::hierholzerStep() {
  int source = checkDegrees();
}*/

int Graph::getNode(float x, float y) {
  for (size_t i = 0; i < nodes.size(); i++) {
    if (CheckCollisionPointCircle({x, y}, {nodes[i].x, nodes[i].y}, nodeR)) {
      return i;
    }
  }
  return -1;
}

void Graph::removeNode(int id) {
  for (size_t i = 0; i < adj.size(); i++) {
    for (auto it = adj[i].begin(); it != adj[i].end(); ) {
      if ((*it).first == id) it = adj[i].erase(it);
      else {
        if ((*it).first > id) (*it).first--;
        it++;
      }
    }
  }
  nodes.erase(nodes.begin() + id);
  adj.erase(adj.begin() + id);

  for (size_t i = id; i < nodes.size(); i++) {
    nodes[i].id--;
  }
}

void Graph::removeEdge(int u, int v) {
  for (auto it = adj[u].begin(); it != adj[u].end(); ) {
    if ((*it).first == v) it = adj[u].erase(it);
    else it++;
  }
}

bool Graph::areNeighbours(int u, int v) {
  for (auto [neighbour, cost] : adj[u]) {
    if (neighbour == v) return true;
  }

  return false;
}
