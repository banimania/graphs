#include "node.hpp"

void Node::drawNode() {
  DrawCircle(x, y, nodeR, marked ? ORANGE : BLACK);
  DrawCircle(x, y, nodeR * 0.9, WHITE);

  string text = to_string(id+1).c_str();
  float textX = x - MeasureTextEx(font, text.c_str(), 30.0f, 0.0f).x / 2.0f;
  float textY = y - MeasureTextEx(font, text.c_str(), 30.0f, 0.0f).y / 2.0f;
  DrawTextEx(font, text.c_str(), {textX, textY}, 30.0f, 0.0f, BLACK);
}
