#ifndef GRAPH_HPP
#define GRAPH_HPP 

#include "globals.hpp"
#include "edge.hpp"
#include "node.hpp"

class Graph {
public:
  vector<Node> nodes;
  vector<vector<pair<int, float>>> adj;
  vector<Edge> markedEdges;

  bool directed = false, weighted = false;

  bool startedDFS = false, startedBFS = false, startedDijkstra = false;
  stack<int> dfsStack;
  queue<int> bfsQueue;
  priority_queue<pair<int, float>, vector<pair<int, float>>, function<bool(pair<int, float>, pair<int, float>)>> dijkstraPq;
  vector<float> bestDist;

  void addNode(float x, float y);
  void addEdge(int u, int v, float cost);
  void drawGraph();
  void restartAlgorithms();
  void dfsStep();
  void bfsStep();
  void dijkstraStep();
  int getNode(float x, float y);
  void removeNode(int id);
  void removeEdge(int u, int v);
  bool areNeighbours(int u, int v);
};

#endif
