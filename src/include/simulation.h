#pragma once

#include "celestialBody.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>

class Simulation {
private:
  std::vector<CelestialBody> bodies;
  GLuint VAO, VBO, shaderProgram;
  GLuint trajectoryVAO, trajectoryVBO, trajectoryShaderProgram;
  glm::mat4 view, projection;
  float G;
  float cameraDistance;
  float cameraAngle;
  bool paused;
  float timeScale;
  bool showTrajectories;
  int trajectoryUpdateCounter;
  static const int TRAJECTORY_UPDATE_INTERVAL = 3;

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

public:
  Simulation();
  ~Simulation();

  void update(float deltaTime);
  void render(int width, int height);
  void handleInput(GLFWwindow *window);
};
