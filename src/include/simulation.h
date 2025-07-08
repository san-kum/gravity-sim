#pragma once

#include "celestialBody.h"
#include "octreeNode.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>

#define DEFAULT_GRAVITATIONAL_CONSTANT 0.1f
#define DEFAULT_CAMERA_DISTANCE 50.0f
#define DEFAULT_TIME_SCALE 1.0f
#define CAMERA_ROTATION_SPEED 0.001f
#define CAMERA_ELEVATION_FACTOR 0.3f
#define TRAJECTORY_UPDATE_INTERVAL 3
#define MIN_CAMERA_DISTANCE 10.0f
#define MAX_CAMERA_DISTANCE 200.0f
#define MIN_TIME_SCALE 0.1f
#define MAX_TIME_SCALE 10.0f
#define TIME_SCALE_FACTOR 1.1f
#define ZOOM_SPEED 1.0f
#define POINT_SCALE_SIZE 500.0f
#define MIN_POINT_SIZE 2.0f
#define MAX_POINT_SIZE 50.0f

enum class GravityAlgorithm { Direct, BarnesHut, FMM, GPU };
enum class ComputeDevice { CPU, GPU };

class Simulation {
private:
  std::vector<CelestialBody> bodies;
  std::unique_ptr<OctreeNode> octreeRoot;

  GLuint VAO, VBO, shaderProgram;
  GLuint trajectoryVAO, trajectoryVBO, trajectoryShaderProgram;
  glm::mat4 view, projection;

  float G;
  float cameraDistance;
  float cameraAngle;
  float targetCameraDistance;
  float targetCameraAngle;
  bool paused;
  float timeScale;
  bool showTrajectories;
  bool useBarnesHut;
  int trajectoryUpdateCounter;
  GravityAlgorithm algorithm; // Add this member
  ComputeDevice computeDevice = ComputeDevice::CPU;
  float lastFPS = 0.0f;
  bool showHUD = true;

  glm::vec3 spaceMin, spaceMax;
  int hoveredBodyIndex = -1;
  int focusedBodyIndex = -1;
  double mouseX = -1, mouseY = -1;
  bool mouseLeftDown = false;

  // Shader sources
  static const char *vertexShaderSource;
  static const char *fragmentShaderSource;
  static const char *trajectoryVertexShaderSource;
  static const char *trajectoryFragmentShaderSource;

  void setupShaders();
  void setupGeometry();
  void setupTrajectoryGeometry();
  void setupScene();
  void updateCamera(int width, int height);
  glm::vec3 getCameraPosition();
  void checkShaderCompilation(GLuint shader, const std::string &type);
  void checkProgramLinking(GLuint program);
  void renderTrajectories();
  void updateGravityFMM();

  void buildOctree();
  void calculateBounds();
  void updateGravityBarnesHut();
  void updateGravityDirect();

  // GPU buffers
  GLuint posSSBO = 0, velSSBO = 0, accSSBO = 0, massSSBO = 0;
  GLuint computeShaderProgram = 0;
  bool gpuSupported = false;
  void updateGravityGPU();
  void initComputeShaderAndBuffers();

  // Cinema
  static constexpr int NUM_STARS = 90;
  glm::vec2 starPositions[NUM_STARS];
  bool starfieldInitialized = false;
  void renderStarfield(int width, int height, float time);

  static char hudTextBuffer[9999];
  void drawText2D(const char *text, float x, float y, float r, float g, float b,
                  float scale, int width, int height) const;

public:
  Simulation();
  ~Simulation();

  void update(float deltaTime);
  void render(int width, int height);
  void handleInput(GLFWwindow *window);
  void setAlgorithm(GravityAlgorithm algo) { algorithm = algo; }
  GravityAlgorithm getAlgorithm() const { return algorithm; }
  void printBodiesInfo() const;
  void setComputeDevice(ComputeDevice dev) { computeDevice = dev; }
  ComputeDevice getComputeDevice() const { return computeDevice; }
  void renderHUD(int width, int height) const;
  void setFPS(float fps) { lastFPS = fps; }
  void toggleHUD() { showHUD = !showHUD; }
  // --- Mouse input ---
  void setMousePosition(double x, double y) {
    mouseX = x;
    mouseY = y;
  }
  void setMouseClick(bool leftDown) { mouseLeftDown = leftDown; }
  void handleCollisions();
};
