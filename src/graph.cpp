#include "graph.hpp"
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

void Graph::drawGraph() {
  // Draw black edges
  for (int i = 0; i < adj.size(); i++) {
    Vector2 start = {nodes[i].x, nodes[i].y};
    for (int j = 0; j < adj[i].size(); j++) {      
      Vector2 end = {nodes[adj[i][j].first].x, nodes[adj[i][j].first].y};

      if (weighted) {
        // fixear tp de peso (cuando cambia la orientación Y) glhf
        float theta = atan2(nodes[adj[i][j].first].y - nodes[i].y, nodes[adj[i][j].first].x - nodes[i].x);

        if (!directed && theta <= 0) theta += M_PI;
        float alpha = - M_PI / 2 + theta;

        Vector2 pos = {(end.x + start.x) / 2.0f + cos(alpha) * 30, (end.y + start.y) / 2.0f + sin(alpha) * 30};
        float cost = adj[i][j].second;
        std::ostringstream oss;
        oss << std::fixed; 
    
        // If it is an integer, show without decimals
        if (cost == static_cast<int>(cost)) {
            oss << static_cast<int>(cost);
        } else {
            // Get how many decimals are needed
            double rounded = static_cast<int>(cost * 100) / 100.0;
            int decimals = (rounded * 10 == static_cast<int>(rounded * 10)) ? 1 : 2;
            oss << setprecision(decimals) << cost;
        }

        string costString = oss.str();

        pos.x -= MeasureTextEx(font, costString.c_str(), 30, 0.0f).x / 2.0f;
        pos.y -= MeasureTextEx(font, costString.c_str(), 30, 0.0f).y / 2.0f;
        DrawTextEx(font, costString.c_str(), pos, 30.0f, 0.0f, BLACK);
      }


      if (directed) {
        float theta = atan2(nodes[adj[i][j].first].y - nodes[i].y, nodes[adj[i][j].first].x - nodes[i].x);
        Vector2 new_point = {-(nodeR - 1) * cos(theta) + nodes[adj[i][j].first].x, -(nodeR - 1) * sin(theta) + nodes[adj[i][j].first].y};
        Vector2 p1 = {-(nodeR + 10) * cos(theta) + nodes[adj[i][j].first].x, -(nodeR + 10) * sin(theta) + nodes[adj[i][j].first].y};
        Vector2 p2 = {-(nodeR + 10) * cos(theta) + nodes[adj[i][j].first].x, -(nodeR + 10) * sin(theta) + nodes[adj[i][j].first].y};

        float alpha = -M_PI / 2 + theta;
        p1.x -= -(10.0) * cos(alpha);
        p1.y -= -(10.0) * sin(alpha);

        p2.x += -(10.0) * cos(alpha);
        p2.y += -(10.0) * sin(alpha);
        
        DrawTriangle(p2, new_point, p1, BLACK);

        new_point = {nodes[adj[i][j].first].x, nodes[adj[i][j].first].y};
        Vector2 old_point = {nodes[i].x, nodes[i].y};
        if (areNeighbours(i, adj[i][j].first)) new_point = {-(nodeR + 10) * cos(theta) + nodes[adj[i][j].first].x, -(nodeR + 10) * sin(theta) + nodes[adj[i][j].first].y};
        if (areNeighbours(adj[i][j].first, i)) old_point = {(nodeR + 10) * cos(theta) + nodes[i].x, (nodeR + 10) * sin(theta) + nodes[i].y};

        DrawLineEx(old_point, new_point, 5, BLACK);
      } else DrawLineEx(start, end, 5, BLACK);
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
  startedDijkstra = false;

  dfsStack = stack<int>();
  bfsQueue = queue<int>();
  dijkstraPq = priority_queue<pair<int, float>, vector<pair<int, float>>, greater<pair<int, float>>>();
  bestDist = vector<float>(nodes.size(), numeric_limits<float>::infinity());

  markedEdges.clear();

  for (int i = 0; i < nodes.size(); i++) {
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

  priority_queue<pair<int, float>, vector<pair<int, float>>, greater<pair<int, float>>> currentQueue;

  // Move all nodes from bfsQueue to currentQueue
  while (!dijkstraPq.empty()) {
    currentQueue.push(dijkstraPq.top());
    dijkstraPq.pop();
  }

  // Process all nodes in currentQueue
  while (!currentQueue.empty()) {
    auto [current, currentDist] = currentQueue.top();
    currentQueue.pop();

    for (auto [neighbour, weight] : adj[current]) {
      if (bestDist[neighbour] > currentDist + weight) {
        bestDist[neighbour] = currentDist + weight;
        nodes[neighbour].marked = true;
        markedEdges.push_back(Edge(current, neighbour));
        dijkstraPq.emplace(neighbour, bestDist[neighbour]); // These will form the next steps queue
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
      if ((*it).first == id) it = adj[i].erase(it);
      else {
        if ((*it).first > id) (*it).first--;
        it++;
      }
    }
  }
  nodes.erase(nodes.begin() + id);
  adj.erase(adj.begin() + id);
  
  for (int i = id; i < nodes.size(); i++) {
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
