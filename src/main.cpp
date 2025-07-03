#include "include/simulation.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

int main() {
  // GLFW
  if (!glfwInit()) {
    std::cerr << "failed to initialize GLFW\n";
    return -1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow *window =
      glfwCreateWindow(1920, 1080, "Gravity Simulator", NULL, NULL);
  if (!window) {
    std::cerr << "failed to create GLFW window\n";
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);

  if (glewInit() != GLEW_OK) {
    std::cerr << "failed to initialize GLEW" << std::endl;
    return -1;
  }
  // opengl config
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  // simulation
  Simulation simulation;
  std::cout << "========================================\n";
  std::cout << "    Gravity Simulator - Controls\n";
  std::cout << "========================================\n";
  std::cout << "SPACE - Pause/Resume\n";
  std::cout << "W/S - Speed up/slow down time\n";
  std::cout << "A/D - zoom in/out\n";
  std::cout << "T - Toggle trajectory\n";
  std::cout << "B - Toggle algorithm\n";
  std::cout << "R - reset simulation\n";
  std::cout << "Esc - Exit\n";
  std::cout << "========================================\n";

  // rendering loop
  float lastTime = glfwGetTime();
  int frameCount = 0;
  float fpsTimer = 0.0f;

  while (!glfwWindowShouldClose(window)) {
    float currentTime = glfwGetTime();
    float deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    simulation.handleInput(window);
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);
    simulation.update(deltaTime);
    simulation.render(width, height);
    glfwSwapBuffers(window);
    glfwPollEvents();

    // frameCount++;
    // fpsTimer += deltaTime;
    // if (fpsTimer >= 1.0f) {
    //   std::cout << "FPS: " << frameCount << "\n";
    //   frameCount = 0;
    //   fpsTimer = 0.0f;
    // }
  }
  glfwTerminate();
  return 0;
}
