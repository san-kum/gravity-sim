#include "include/simulation.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else

#include <GL/glut.h>
#endif

int main(int argc, char **argv) {
  glutInit(&argc, argv);
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

    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);
    simulation.setMousePosition(mouseX, mouseY);
    static int prevLeft = GLFW_RELEASE;
    int left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    if (left == GLFW_PRESS && prevLeft == GLFW_RELEASE)
      simulation.setMouseClick(true);
    else
      simulation.setMouseClick(false);

    prevLeft = left;
    static int prevRight = GLFW_RELEASE;
    int right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);

    prevRight = right;
    simulation.update(deltaTime);
    simulation.render(width, height);

    // frameCount++;
    // fpsTimer += deltaTime;
    // if (fpsTimer >= 1.0f) {
    //   simulation.setFPS(frameCount / fpsTimer);
    //   std::cout << "FPS: " << frameCount << "\n";
    //   frameCount = 0;
    //   fpsTimer = 0.0f;
    // }
    simulation.renderHUD(width, height);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  glfwTerminate();
  return 0;
}
