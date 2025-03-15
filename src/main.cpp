#include "../include/globals.hpp"
#include "../include/graph.hpp"
#include <raylib.h>
#if defined(__EMSCRIPTEN__)
  #include <emscripten/emscripten.h>
#endif

#define RAYGUI_IMPLEMENTATION
#include "../lib/raygui.h"

using namespace std;

Camera2D cam;

Vector2 mouse;

RenderTexture2D target;

Graph g;

int mode = 0; // 0 = "build graph" 1 = "dfs" 2 = "bfs"
int stNode = -1;
int lastStNode = -1;

int lastNodeMoved = -1;
bool wasMovingCamera = false;

Rectangle optionsRectangle = {20, WINDOW_HEIGHT - 60, 100, 40};
bool inOptionsMenu = false;

void mainLoop() {
  GuiSetStyle(DEFAULT, TEXT_SIZE, 20);
  
  float scale = fmin((float) GetScreenWidth() / GAME_SCREEN_WIDTH, (float) GetScreenHeight() / GAME_SCREEN_HEIGHT);

  mouse = GetMousePosition();
  mouse.x = (mouse.x - (GetScreenWidth() - (GAME_SCREEN_WIDTH * scale)) * 0.5f) / scale;
  mouse.y = (mouse.y - (GetScreenHeight() - (GAME_SCREEN_HEIGHT * scale)) * 0.5f) / scale;
  mouse = Vector2Clamp(mouse, (Vector2){ 0, 0 }, (Vector2){ (float) GAME_SCREEN_WIDTH, (float) GAME_SCREEN_HEIGHT });

  BeginTextureMode(target);
  ClearBackground(WHITE);
  BeginMode2D(cam);

  Rectangle viewRect = { cam.target.x - cam.offset.x / cam.zoom, cam.target.y - cam.offset.y / cam.zoom, GetScreenWidth() / cam.zoom, GetScreenHeight() / cam.zoom };
 
  Vector2 mouseWorldPos = GetScreenToWorld2D(mouse, cam);

  g.drawGraph();

  EndMode2D();

  bool skip = false;
  if (GuiButton(optionsRectangle, inOptionsMenu ? "Close" : "Options")) {
    inOptionsMenu ^= 1;
    g.restartAlgorithms();
    skip = true;
  }

  BeginMode2D(cam);

  if (!inOptionsMenu && !skip) { 
    if (IsKeyPressed(KEY_ONE)) {
      mode = 0;
      g.restartAlgorithms();
    } else if (IsKeyPressed(KEY_TWO)) {
      mode = 1;
      g.restartAlgorithms();
    } else if (IsKeyPressed(KEY_THREE)) {
      mode = 2;
      g.restartAlgorithms();
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      Vector2 delta = GetMouseDelta();
      delta = {delta.x / cam.zoom, delta.y / cam.zoom};

      if (lastNodeMoved != -1) {
        g.nodes[lastNodeMoved].x += delta.x;
        g.nodes[lastNodeMoved].y += delta.y;
      } else {
        int node = g.getNode(mouseWorldPos.x, mouseWorldPos.y);

        int magnitude = sqrt(pow(delta.x, 2) + pow(delta.y, 2));
        if (node != -1) {
          g.nodes[node].x += delta.x;
          g.nodes[node].y += delta.y;
          lastNodeMoved = node;
        } else if (magnitude > 0.2) {
          cam.target = Vector2Subtract(cam.target, delta);
          wasMovingCamera = true;
        }
      }
    }

    float wheelDelta = GetMouseWheelMove();
    if (wheelDelta != 0) {
      Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), cam);

      cam.offset = GetMousePosition();

      cam.target = mouseWorldPos;

      float scaleFactor = 1.0f + (0.25f * fabsf(wheelDelta));
      if (wheelDelta < 0) scaleFactor = 1.0f / scaleFactor;
      cam.zoom = Clamp(cam.zoom * scaleFactor, 0.5f, 16.0f);
    }

    if (IsKeyReleased(KEY_SPACE)) {
      if (mode == 1) g.dfsStep();
      else if (mode == 2) g.bfsStep();
    }

    if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) {
      if (mode == 0 && lastStNode != -1) {
        g.removeNode(lastStNode);
      }
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
      // Unmark the previous node if there was one
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && lastStNode != -1) {
        g.nodes[lastStNode].marked = false;
      }

      stNode = g.getNode(mouseWorldPos.x, mouseWorldPos.y);

      // Mark the new node and update lastSelectedNode
      if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && stNode != -1) {
        g.nodes[stNode].marked = true;
        lastStNode = stNode;
      }
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
      if (stNode != -1 && mode == 0) {

        int fiNode = g.getNode(mouseWorldPos.x, mouseWorldPos.y);

        if (fiNode != -1) {
          // Unmark the previous node if there was one
          if (lastStNode != -1) {
            g.nodes[lastStNode].marked = false;
          }

          g.addEdge(stNode, fiNode);
        }
      } 
      stNode = -1;
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
      lastNodeMoved = -1;
      if (stNode != -1) {
        if (mode == 0) {
          // Unmark the previous node if there was one
          if (lastStNode != -1) {
            g.nodes[lastStNode].marked = false;
          }
          // Mark the new node and update lastSelectedNode
          g.nodes[stNode].marked = true;
          lastStNode = stNode;
        }
        if (mode == 1) {
          g.restartAlgorithms();

          g.startedDFS = true;
          g.nodes[stNode].marked = true;
          g.dfsStack.push(stNode);
        } else if (mode == 2) {
          g.restartAlgorithms();

          g.startedBFS = true;
          g.nodes[stNode].marked = true;
          g.bfsQueue.push(stNode);
        }
      } else {
        if (mode == 0 && !wasMovingCamera) g.addNode(mouseWorldPos.x, mouseWorldPos.y);
      }

      stNode = -1;
      wasMovingCamera = false;
    }
  }

  EndMode2D();

  string modeStr = "Graph building";

  if (mode == 1) {
    modeStr = "Depth-first search";
  } else if (mode == 2) {
    modeStr = "Breadth-first search";
  }

  if (inOptionsMenu) {
    modeStr = "Options";
  }

  if (inOptionsMenu) {
    GuiPanel({20, 50, GAME_SCREEN_WIDTH - 40, GAME_SCREEN_HEIGHT - 120}, "Graph Options");
    GuiSetStyle(DEFAULT, TEXT_SIZE, 30);
    GuiLabel({35, 100, 100, 50}, "Edges: ");
    GuiSetStyle(DEFAULT, TEXT_SIZE, 25);
    if (GuiButton({150, 105, 230, 40}, g.directed ? "Directed" : "Non-directed")) {
      g.directed ^= 1;
    }
    if (GuiButton({150, 160, 230, 40}, g.weighted ? "Weighted" : "Non-weighted")) {
      g.weighted ^= 1;
    }
    
    GuiSetStyle(DEFAULT, TEXT_SIZE, 30);
    GuiLabel({35, 220, 100, 50}, "Modes: ");
    GuiSetStyle(DEFAULT, TEXT_SIZE, 25);
    if (GuiButton({150, 225, 230, 40}, "Graph building")) {
      g.restartAlgorithms();
      inOptionsMenu = false;
      mode = 0;
    }
    if (GuiButton({150, 275, 230, 40}, "Depth-first search")) {
      g.restartAlgorithms();
      inOptionsMenu = false;
      mode = 1;
    }
    if (GuiButton({150, 325, 230, 40}, "Breadth-first search")) {
      g.restartAlgorithms();
      inOptionsMenu = false;
      mode = 2;
    }
  }
  DrawTextEx(font, modeStr.c_str(), {20, 15}, 30.0f, 0.0f, DARKGRAY);

  EndTextureMode();

  BeginDrawing();
  ClearBackground(BLACK);
  DrawTexturePro(target.texture, (Rectangle){ 0.0f, 0.0f, (float) target.texture.width, (float) -target.texture.height },
                 (Rectangle){ (GetScreenWidth() - ((float) GAME_SCREEN_WIDTH * scale)) * 0.5f, (GetScreenHeight() - ((float) GAME_SCREEN_HEIGHT * scale)) * 0.5f,
                 (float) GAME_SCREEN_WIDTH * scale, (float) GAME_SCREEN_HEIGHT * scale }, (Vector2) { 0, 0 }, 0.0f, WHITE);
  EndDrawing();
}

int main() {
  // SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_NAME);
  
  font = LoadFontEx("res/arial.ttf", 30, 0, 0);
  GuiSetFont(font);

  target = LoadRenderTexture(GAME_SCREEN_WIDTH, GAME_SCREEN_HEIGHT);
  SetTextureFilter(target.texture, TEXTURE_FILTER_BILINEAR);

  SetTargetFPS(60);

  cam = {  };
  cam.zoom = 1.0f;
  cam.target.x = 0.0f;
  cam.target.y = 0.0f;
  cam.offset.x = 0.0f;
  cam.offset.y = 0.0f;

#if defined(__EMSCRIPTEN__)
  emscripten_set_main_loop(mainLoop, 240, 1);
#else
  while (!WindowShouldClose()) {
    mainLoop();
  }
#endif
  UnloadRenderTexture(target);
  CloseWindow();
  return 0;
}
