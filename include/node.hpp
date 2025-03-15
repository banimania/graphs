#ifndef NODE_HPP
#define NODE_HPP

#include "globals.hpp"

class Node {
public:
  int id;
  float x, y;

  bool marked = false;

  Node(int id, float x, float y) : x(x), y(y), id(id) {};

  void drawNode();
};

#endif
