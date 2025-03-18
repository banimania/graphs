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

void drawDirectedEdge(Vector2 from, Vector2 to, bool adjustFrom, bool adjustTo, Color color, float thickness = 5.0f) {
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

void Graph::drawGraph() {
  // Draw black edges
  for (size_t i = 0; i < adj.size(); i++) {
    Vector2 start = {nodes[i].x, nodes[i].y};
    for (size_t j = 0; j < adj[i].size(); j++) {
      Vector2 end = {nodes[adj[i][j].first].x, nodes[adj[i][j].first].y};

      if (weighted) {
        float theta = atan2(end.y - start.y, end.x - start.x);
        if (!directed && theta <= 0) theta += M_PI;
        float alpha = -M_PI / 2 + theta;

        Vector2 pos = {(end.x + start.x) / 2.0f + cos(alpha) * 30, 
                      (end.y + start.y) / 2.0f + sin(alpha) * 30};
        float cost = adj[i][j].second;

        string costString = formatNum(cost);

        pos.x -= MeasureTextEx(font, costString.c_str(), 30, 0.0f).x / 2.0f;
        pos.y -= MeasureTextEx(font, costString.c_str(), 30, 0.0f).y / 2.0f;
        DrawTextEx(font, costString.c_str(), pos, 30.0f, 0.0f, BLACK);
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
  for (Edge e : markedEdges) {
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

  // Draw nodes
  for (size_t i = 0; i < nodes.size(); i++) {
    nodes[i].drawNode();
  }
}


void Graph::restartAlgorithms() {
  startedDFS = false;
  startedBFS = false;
  startedDijkstra = false;

  dfsStack = stack<int>();
  bfsQueue = queue<int>();
  dijkstraPq = priority_queue<pair<int, float>, vector<pair<int, float>>, function<bool(pair<int, float>, pair<int, float>)>> ([](const pair<int, float> &a, const pair<int, float> &b) {
    return a.second > b.second;
  });
  bestDist = vector<float>(nodes.size(), numeric_limits<float>::infinity());
  dijkstraPrev = vector<int>(nodes.size(), -1);

  markedEdges.clear();

  for (size_t i = 0; i < nodes.size(); i++) {
    nodes[i].marked = false;
  }
}

void Graph::dfsStep() {
  // Make sure we always actually visit a new node
  while (!dfsStack.empty()) {
    int current = dfsStack.top();
    for (auto [neighbour, cost] : adj[current]) {
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
        markedEdges.push_back(Edge(current, neighbour));
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
        markedEdges.push_back(Edge(current.first, neighbour));
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
