#ifndef EDGE_HPP
#define EDGE_HPP

#include "globals.hpp"

class Edge {
public:
  int u, v;
  float cost;

  Edge(int u, int v, float cost) : u(u), v(v), cost(cost) {};

  void drawEdge(Vector2 p1, Vector2 p2);
};

#endif
