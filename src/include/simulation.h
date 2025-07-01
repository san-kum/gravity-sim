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
  glm::mat4 view, projection;
  float G;
  float cameraDistance;
  float cameraAngle;
  bool paused;
  float timeScale;

  // Shader sources
  static const char *vertexShaderSource;
  static const char *fragmentShaderSource;

  void setupShaders();
  void setupGeometry();
  void setupScene();
  void updateCamera(int width, int height);
  glm::vec3 getCameraPosition();
  void checkShaderCompilation(GLuint shader, const std::string &type);
  void checkProgramLinking(GLuint program);

public:
  Simulation();
  ~Simulation();

  void update(float deltaTime);
  void render(int width, int height);
  void handleInput(GLFWwindow *window);
};
