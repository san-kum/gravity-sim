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
  bool paused;
  float timeScale;
  bool showTrajectories;
  bool useBarnesHut;
  int trajectoryUpdateCounter;

  glm::vec3 spaceMin, spaceMax;

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

  void buildOctree();
  void calculateBounds();
  void updateGravityBarnesHut();
  void updateGravityDirect();

public:
  Simulation();
  ~Simulation();

  void update(float deltaTime);
  void render(int width, int height);
  void handleInput(GLFWwindow *window);
};
