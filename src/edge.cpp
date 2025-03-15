#include "edge.hpp"
  
void Edge::drawEdge(Vector2 p1, Vector2 p2) {
  DrawLineEx(p1, p2, 5, ORANGE);
}
