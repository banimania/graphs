#ifndef EDGE_HPP
#define EDGE_HPP

#include "globals.hpp"

class Edge {
public:
  int u, v;

  Edge(int u, int v) : u(u), v(v) {};

  void drawEdge(Vector2 p1, Vector2 p2);
};

#endif
