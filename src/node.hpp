#ifndef NODE_HPP
#define NODE_HPP

#include "globals.hpp"

class Node {
public:
  int id;
  float x, y;

  bool marked = false;

  Node(int id, float x, float y) : id(id), x(x), y(y) {};

  void drawNode();
};

#endif
