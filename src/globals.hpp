#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <iomanip>
#include <raylib.h>
#include <iostream>
#include <queue>
#include <rlgl.h>
#include <raymath.h>
#include <stack>
#include <string>
#include <vector>
#include <functional>

using namespace std;

static const int WINDOW_WIDTH = 1280;
static const int WINDOW_HEIGHT = 720;
static const char* WINDOW_NAME = "Graphs";

static const int GAME_SCREEN_WIDTH = 1280;
static const int GAME_SCREEN_HEIGHT = 720;

static const float nodeR = 20.0f;

inline Font font;

inline Vector2 GetMouseWorldPosition(float scale) {
  Vector2 mouse = GetMousePosition();
  mouse.x = (mouse.x - (GetScreenWidth() - (GAME_SCREEN_WIDTH * scale)) * 0.5f) / scale;
  mouse.y = (mouse.y - (GetScreenHeight() - (GAME_SCREEN_HEIGHT * scale)) * 0.5f) / scale;
  mouse = Vector2Clamp(mouse, (Vector2){ 0, 0 }, (Vector2){ (float) GAME_SCREEN_WIDTH, (float) GAME_SCREEN_HEIGHT });
  return mouse;
}

inline string formatNum(float num) {
  ostringstream oss;
  oss << fixed; 

  // If it is an integer, show without decimals
  if (num == static_cast<int>(num)) {
    oss << static_cast<int>(num);
  } else {
    // Get how many decimals are needed
    double rounded = static_cast<int>(num * 100) / 100.0;
    int decimals = (rounded * 10 == static_cast<int>(rounded * 10)) ? 1 : 2;
    oss << setprecision(decimals) << num;
  }

  return oss.str();
}

#endif
