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

  bool startedDFS = false, startedBFS = false, startedDijkstra = false, startedKruskal = false;
  stack<int> dfsStack;
  queue<int> bfsQueue;
  priority_queue<pair<int, float>, vector<pair<int, float>>, function<bool(pair<int, float>, pair<int, float>)>> dijkstraPq;
  vector<float> bestDist;
  vector<int> dijkstraPrev;
  vector<int> parent;
  vector<int> setSize;
  priority_queue<Edge, vector<Edge>, function<bool(Edge, Edge)>> kruskalPq;
  float kruskalCost;

  void addNode(float x, float y);
  void addEdge(int u, int v, float cost);
  void drawGraph();
  void restartAlgorithms();
  void dfsStep();
  void bfsStep();
  void dijkstraStep();
  vector<int> getDijkstraPath(int u, int v);
  void kruskalStep();
  int getNode(float x, float y);
  void removeNode(int id);
  void removeEdge(int u, int v);
  int findSet(int a);
  bool unionSet(int a, int b);
  bool areNeighbours(int u, int v);

private:
  void resetDFS();
  void resetBFS();
  void resetDijkstra();
  void resetSearchStates();
  void resetKruskal();
  void calculateDSU();

  void drawEdgeWeight(Vector2 from, Vector2 to, float cost, bool directed);
  void drawDirectedEdge(Vector2 from, Vector2 to, bool adjustFrom, bool adjustTo, Color color, float thickness = 5.0f);
  void drawMarkedEdges();
};

#endif
