#include <iostream>
#include <queue>
#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>
#include <stack>
#include <string>
#include <vector>
#if defined(__EMSCRIPTEN__)
  #include <emscripten/emscripten.h>
#endif

using namespace std;

const int WINDOW_WIDTH = 1280;
const int WINDOW_HEIGHT = 720;
const char* WINDOW_NAME = "Graphs";

const int GAME_SCREEN_WIDTH = 1280;
const int GAME_SCREEN_HEIGHT = 720;

Camera2D cam;

Vector2 mouse;

RenderTexture2D target;

Font font;

float nodeR = 20.0f;
class Node {
public:
  int id;
  float x, y;

  bool marked = false;

  Node(int id, float x, float y) : x(x), y(y), id(id) {};

  void drawNode() {
    DrawCircle(x, y, nodeR, marked ? ORANGE : BLACK);
    DrawCircle(x, y, nodeR * 0.9, WHITE);

    string text = to_string(id+1).c_str();
    float textX = x - MeasureTextEx(font, text.c_str(), 30.0f, 0.0f).x / 2.0f;
    float textY = y - MeasureTextEx(font, text.c_str(), 30.0f, 0.0f).y / 2.0f;
    DrawTextEx(font, text.c_str(), {textX, textY}, 30.0f, 0.0f, BLACK);
  }
};

class Edge {
  public:
    int u, v;

    Edge(int u, int v) : u(u), v(v) {};
  
    void drawEdge(Vector2 p1, Vector2 p2) {
      DrawLineEx(p1, p2, 5, ORANGE);
    }
  };

class Graph {
public:
  vector<Node> nodes;
  vector<vector<int>> adj;
  vector<Edge> markedEdges;

  bool startedDFS = false, startedBFS = false;
  stack<int> dfsStack;
  queue<int> bfsQueue;

  void addNode(float x, float y) {
    nodes.push_back(Node(nodes.size(), x, y));
    adj.push_back(vector<int>());
  }

  void addEdge(int u, int v) {
    adj[u].push_back(v);
    adj[v].push_back(u);
  }

  void drawGraph() {

    // Draw black edges
    for (int i = 0; i < adj.size(); i++) {
      Vector2 start = {nodes[i].x, nodes[i].y};
      for (int j = 0; j < adj[i].size(); j++) {
        Vector2 end = {nodes[adj[i][j]].x, nodes[adj[i][j]].y};
        DrawLineEx(start, end, 5, BLACK);
      }
    }

    // Draw orange edges
    for(Edge e : markedEdges) {
      e.drawEdge(Vector2{nodes[e.u].x, nodes[e.u].y}, Vector2{nodes[e.v].x, nodes[e.v].y});
    }

    for (int i = 0; i < nodes.size(); i++) {
      nodes[i].drawNode();
    }
  }

  void restartAlgorithms() {
    startedDFS = false;
    startedBFS = false;

    dfsStack = stack<int>();
    bfsQueue = queue<int>();

    markedEdges.clear();

    for (int i = 0; i < nodes.size(); i++) {
      nodes[i].marked = false;
    }
  }

  void dfsStep() {

    // Make sure we always actually visit a new node
    while (!dfsStack.empty()) {
        int current = dfsStack.top();
        for (int neighbour : adj[current]) {
            if (!nodes[neighbour].marked) {
                nodes[neighbour].marked = true;
                markedEdges.push_back(Edge(current, neighbour));
                dfsStack.push(neighbour);
                return; // Exit after adding a new node
            }
        }

        // If all neighbors are visited, pop the current node and continue
        dfsStack.pop();
    }
  }

  void bfsStep() {
    if (bfsQueue.empty()) return;

    std::queue<int> currentQueue;
    
    // Move all nodes from bfsQueue to currentQueue
    while (!bfsQueue.empty()) {
        currentQueue.push(bfsQueue.front());
        bfsQueue.pop();
    }

    // Process all nodes in currentQueue
    while (!currentQueue.empty()) {
        int current = currentQueue.front();
        currentQueue.pop();

        for (int neighbour : adj[current]) {
            if (!nodes[neighbour].marked) {
                nodes[neighbour].marked = true;
                markedEdges.push_back(Edge(current, neighbour));
                bfsQueue.push(neighbour); // These will form the next step's queue
            }
        }
    }
  }

  int getNode(float x, float y) {
    for (int i = 0; i < nodes.size(); i++) {
      if (CheckCollisionPointCircle({x, y}, {nodes[i].x, nodes[i].y}, nodeR)) {
        return i;
      }
    }
    return -1;
  }
};

Graph g;

int mode = 0; // 0 = "build graph" 1 = "dfs" 2 = "bfs"
int stNode = -1;

int lastNodeMoved = -1;
bool wasMovingCamera = false;

void mainLoop() {
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

  if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
    stNode = g.getNode(mouseWorldPos.x, mouseWorldPos.y);
  }

  if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
    if (stNode != -1 && mode == 0) {
      int fiNode = g.getNode(mouseWorldPos.x, mouseWorldPos.y);

      if (fiNode != -1) {
        g.addEdge(stNode, fiNode);
      }
    } 
    stNode = -1;
  }

  if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
    lastNodeMoved = -1;
    if (stNode != -1) {
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

  g.drawGraph();

  string modeStr = "Build graph";

  if (mode == 1) {
    modeStr = "Depth-first search";
  } else if (mode == 2) {
    modeStr = "Breadth-first search";
  }

  EndMode2D();

  DrawTextEx(font, modeStr.c_str(), {20, 20}, 30.0f, 0.0f, DARKGRAY);

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
