#include "globals.hpp"
#include "graph.hpp"
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

bool isAskingWeight = false;
bool editingWeight = false;
int askNodeSt = -1, askNodeFi = -1;
char weightText[16] = "1.0";

inline void HandleModeSelection(int &mode, Graph &g) {
  if (IsKeyPressed(KEY_ONE)) {
    mode = 0;
    g.restartAlgorithms();
  } else if (IsKeyPressed(KEY_TWO)) {
    mode = 1;
    g.restartAlgorithms();
  } else if (IsKeyPressed(KEY_THREE)) {
    mode = 2;
    g.restartAlgorithms();
  } else if (IsKeyPressed(KEY_FOUR)) {
    mode = 3;
    g.restartAlgorithms();
  }
}

inline void HandleCameraZoom(Camera2D &cam) {
  float wheelDelta = GetMouseWheelMove();
  if (wheelDelta != 0) {
    Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), cam);
    cam.offset = GetMousePosition();
    cam.target = mouseWorldPos;
    float scaleFactor = 1.0f + (0.25f * fabsf(wheelDelta));
    if (wheelDelta < 0) scaleFactor = 1.0f / scaleFactor;
    cam.zoom = Clamp(cam.zoom * scaleFactor, 0.5f, 16.0f);
  }
}

void mainLoop() {
  GuiSetStyle(DEFAULT, TEXT_SIZE, 20);
  
  float scale = fmin((float) GetScreenWidth() / GAME_SCREEN_WIDTH, (float) GetScreenHeight() / GAME_SCREEN_HEIGHT);

  mouse = GetMouseWorldPosition(scale);

  BeginTextureMode(target);
  ClearBackground(WHITE);
  BeginMode2D(cam);

  Rectangle viewRect = { cam.target.x - cam.offset.x / cam.zoom, cam.target.y - cam.offset.y / cam.zoom, GetScreenWidth() / cam.zoom, GetScreenHeight() / cam.zoom };
 
  Vector2 mouseWorldPos = GetScreenToWorld2D(mouse, cam);

  g.drawGraph();

  EndMode2D();

  bool skip = false;
  if (GuiButton(optionsRectangle, inOptionsMenu ? "Close" : "Options") && !isAskingWeight) {
    inOptionsMenu ^= 1;
    g.restartAlgorithms();
    skip = true;
  }

  BeginMode2D(cam);

  if (!inOptionsMenu && !skip && !isAskingWeight) { 
    
    HandleModeSelection(mode, g);

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

    HandleCameraZoom(cam);

    if (IsKeyReleased(KEY_SPACE)) {
      if (mode == 1) g.dfsStep();
      else if (mode == 2) g.bfsStep();
      else if (mode == 3) g.dijkstraStep();
    }

    if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) {
      if (mode == 0 && lastStNode != -1) {
        g.removeNode(lastStNode);
        lastStNode = -1;
      }
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
      stNode = g.getNode(mouseWorldPos.x, mouseWorldPos.y);

      // Mark the new node and update lastSelectedNode
      if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && stNode != -1) {
        g.restartAlgorithms();
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

          if (!g.areNeighbours(stNode, fiNode)) {
            if (g.weighted) {
              isAskingWeight = true;
              askNodeSt = stNode;
              askNodeFi = fiNode;
            }
            else g.addEdge(stNode, fiNode, 1);
          } else {
            if(!g.directed) g.removeEdge(fiNode, stNode);
            g.removeEdge(stNode, fiNode);
          }
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
        } else if (mode == 3) {
          g.restartAlgorithms();

          g.startedDijkstra = true;
          g.dijkstraPq.push({stNode, 0});
          g.bestDist[stNode] = 0;
          g.nodes[stNode].marked = true;
        }
      } else {
        if (mode == 0 && !wasMovingCamera) {
          g.restartAlgorithms();
          g.addNode(mouseWorldPos.x, mouseWorldPos.y);
        }
      }

      stNode = -1;
      wasMovingCamera = false;
    }
  }

  EndMode2D();
  
  if (isAskingWeight) {
    float wx = WINDOW_WIDTH / 2.0f - 150;
    float wy = WINDOW_HEIGHT / 2.0f - 75;
    float ww = 200;
    float wh = 150;

    GuiPanel({wx, wy, ww, wh}, "Enter edge weight");
    // GuiTextInputBox({wx + 50, wy + 50, 100, 50}, "Edge weight", "Edge weight",  , char *text, int textMaxSize, bool *secretViewActive)
    GuiSetStyle(TEXTBOX, BASE_COLOR_PRESSED, ColorToInt(WHITE));
    GuiSetStyle(TEXTBOX, BASE_COLOR_FOCUSED, ColorToInt(WHITE));
    GuiSetStyle(TEXTBOX, BORDER_COLOR_FOCUSED, ColorToInt(BLACK));
    GuiSetStyle(TEXTBOX, BORDER_COLOR_PRESSED, ColorToInt(BLACK));
    GuiSetStyle(TEXTBOX, TEXT_COLOR_PRESSED, ColorToInt(BLACK));
    GuiSetStyle(TEXTBOX, TEXT_COLOR_FOCUSED, ColorToInt(BLACK));
    if (GuiTextBox({wx + 10, wy + 30, 180, 50}, weightText, 20, editingWeight)) {
      editingWeight = !editingWeight;
    }
    if (GuiButton({wx + 10, wy + 90, 180, 50}, "Set weight") || IsKeyReleased(KEY_ENTER)) {
      float cost = atof(weightText);
      g.addEdge(askNodeSt, askNodeFi, cost);
      askNodeSt = -1, askNodeFi = -1;
      editingWeight = false;
      isAskingWeight = false;
      strcpy(weightText, "1.0");
    }
  }

  string modeStr = "Graph building";

  if (mode == 1) {
    modeStr = "Depth-first search";
  } else if (mode == 2) {
    modeStr = "Breadth-first search";
  } else if (mode == 3) {
    modeStr = "Dijkstra";
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
      bool oldValue = g.directed;
      bool weighted = g.weighted;
      g = Graph();
      g.directed = !oldValue;
      g.weighted = weighted;
    }
    if (GuiButton({150, 160, 230, 40}, g.weighted ? "Weighted" : "Non-weighted")) {
      bool oldValue = g.weighted;
      bool directed = g.directed;
      g = Graph();
      g.weighted = !oldValue;
      g.directed = directed;
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
    if (GuiButton({150, 375, 230, 40}, "Dijkstra")) {
      g.restartAlgorithms();
      inOptionsMenu = false;
      mode = 3;
    }
  }

  if (g.startedDijkstra && g.dijkstraPq.empty()) {
    float wx = WINDOW_WIDTH - 400;
    float wy = 20;
    float ww = 350;
    float wh = WINDOW_HEIGHT - 40;
    GuiPanel({wx, wy, ww, wh}, "Dijkstra information");

    for (size_t i = 0; i < g.nodes.size(); i++) {
      GuiLabel({wx + 20, wy + 50 + i * 60, 200, 50}, string("Distance to " + to_string(i + 1) + ": " + formatNum(g.bestDist[i])).c_str());
      vector<int> path = g.getDijkstraPath(stNode, i);
      string pathString;

      for (size_t j = 0; j < path.size(); j++) {
        pathString += to_string(path[j] + 1) + (j == path.size() - 1 ? "" : " -> ");
      }
      GuiLabel({wx + 20, wy + 70 + i * 60, 200, 50}, pathString.c_str()); 
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
  
  font = LoadFontEx("res/arial.ttf", 100, 0, 0);
  GuiSetFont(font);

  target = LoadRenderTexture(GAME_SCREEN_WIDTH, GAME_SCREEN_HEIGHT);
  SetTextureFilter(target.texture, TEXTURE_FILTER_BILINEAR);

  SetConfigFlags(FLAG_VSYNC_HINT);

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
