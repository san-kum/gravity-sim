#include "include/simulation.h"
#include <cmath>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <random>

const char *Simulation::vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 FragColor;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    FragColor = aColor;
}
)";

const char *Simulation::fragmentShaderSource = R"(
#version 330 core
out vec4 color;
in vec3 FragColor;

void main()
{
    // Create circular particle effect
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    if (dot(circCoord, circCoord) > 1.0) {
        discard;
    }
    
    // Add glow effect based on distance from center
    float dist = length(circCoord);
    float alpha = 1.0 - smoothstep(0.0, 1.0, dist);
    
    color = vec4(FragColor, alpha);
}
)";

Simulation::Simulation()
    : G(0.1f), cameraDistance(50.0f), cameraAngle(0.0f), paused(false),
      timeScale(1.0f) {
  setupShaders();
  setupGeometry();
  setupScene();
}

Simulation::~Simulation() {
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glDeleteProgram(shaderProgram);
}

void Simulation::setupShaders() {
  GLuint vertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex, 1, &vertexShaderSource, NULL);
  glCompileShader(vertex);
  checkShaderCompilation(vertex, "VERTEX");

  GLuint fragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment, 1, &fragmentShaderSource, NULL);
  glCompileShader(fragment);
  checkShaderCompilation(fragment, "FRAGMENT");

  shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertex);
  glAttachShader(shaderProgram, fragment);
  glLinkProgram(shaderProgram);
  checkProgramLinking(shaderProgram);

  glDeleteShader(vertex);
  glDeleteShader(fragment);
}

void Simulation::setupGeometry() {
  float vertices[] = {
      0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f // position + color
  };

  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);

  glBindVertexArray(VAO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
}

void Simulation::setupScene() {
  // central object fixed (e.g., sun)
  bodies.emplace_back(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f), 1000.0f,
                      2.0f, glm::vec3(1.0f, 1.0f, 0.0f), true);

  // random objects to orbit around the central object
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 2.0 * M_PI);

  // inner objects -> faster and closer orbits
  for (int i = 0; i < 30; i++) {
    float distance = 8.0f + i * 4.0f;
    float angle = dis(gen);
    float orbitalSpeed = sqrt(G * 1000.0f / distance) * 0.8f;

    glm::vec3 pos(distance * cos(angle), 0.0f, distance * sin(angle));
    glm::vec3 vel(-orbitalSpeed * sin(angle), 0.0f, orbitalSpeed * cos(angle));

    bodies.emplace_back(pos, vel, 1.0f + i * 0.5f, 0.3f + i * 0.1f,
                        glm::vec3(0.3f + i * 0.2f, 0.5f, 1.0f - i * 0.2f));
  }

  // outer objects -> slower and longer orbits
  for (int i = 0; i < 30; i++) {
    float distance = 25.0f + i * 8.0f;
    float angle = dis(gen);
    float orbitalSpeed = sqrt(G * 1000.0f / distance) * 0.7f;

    glm::vec3 pos(distance * cos(angle), 0.0f, distance * sin(angle));
    glm::vec3 vel(-orbitalSpeed * sin(angle), 0.0f, orbitalSpeed * cos(angle));

    bodies.emplace_back(pos, vel, 0.5f + i * 0.3f, 0.2f + i * 0.1f,
                        glm::vec3(1.0f - i * 0.2f, 0.3f + i * 0.2f, 0.5f));
  }

  // objects between inner and outer objects (small debris)
  for (int i = 0; i < 50; i++) {
    float distance = 15.0f + (i % 3) * 5.0f;
    float angle = dis(gen);
    float orbitalSpeed = sqrt(G * 1000.0f / distance) *
                         (0.6f + 0.2f * (rand() / (float)RAND_MAX));

    glm::vec3 pos(distance * cos(angle),
                  (rand() / (float)RAND_MAX - 0.5f) * 2.0f,
                  distance * sin(angle));
    glm::vec3 vel(-orbitalSpeed * sin(angle), 0.0f, orbitalSpeed * cos(angle));

    bodies.emplace_back(pos, vel, 0.1f, 0.05f, glm::vec3(0.6f, 0.6f, 0.6f));
  }
}

void Simulation::update(float deltaTime) {
  if (paused)
    return;

  float dt = deltaTime * timeScale;

  // gravitational forces between the body pairs
  for (size_t i = 0; i < bodies.size(); i++) {
    for (size_t j = 0; j < bodies.size(); j++) {
      if (i != j) {
        bodies[i].applyGravity(bodies[j], G);
      }
    }
  }

  for (auto &body : bodies) {
    body.update(dt);
  }
}

void Simulation::render(int width, int height) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  updateCamera(width, height);

  glUseProgram(shaderProgram);

  glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE,
                     glm::value_ptr(view));
  glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1,
                     GL_FALSE, glm::value_ptr(projection));

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glBindVertexArray(VAO);

  for (const auto &body : bodies) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, body.position);
    model = glm::scale(model, glm::vec3(body.radius));

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1,
                       GL_FALSE, glm::value_ptr(model));

    float distance = glm::length(body.position - getCameraPosition());
    float pointSize = (body.radius * 500.0f) / distance;
    pointSize = glm::clamp(pointSize, 2.0f, 50.0f);
    glPointSize(pointSize);

    float colorData[] = {0.0f,         0.0f,         0.0f,
                         body.color.r, body.color.g, body.color.b};
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(colorData), colorData);

    glDrawArrays(GL_POINTS, 0, 1);
  }

  glDisable(GL_BLEND);
}

void Simulation::updateCamera(int width, int height) {
  cameraAngle += 0.001f;

  glm::vec3 cameraPos = glm::vec3(cameraDistance * cos(cameraAngle),
                                  cameraDistance * 0.3f, // Slight elevation
                                  cameraDistance * sin(cameraAngle));

  view = glm::lookAt(cameraPos, glm::vec3(0.0f, 0.0f, 0.0f),
                     glm::vec3(0.0f, 1.0f, 0.0f));

  projection = glm::perspective(glm::radians(45.0f),
                                (float)width / (float)height, 0.1f, 1000.0f);
}

glm::vec3 Simulation::getCameraPosition() {
  return glm::vec3(cameraDistance * cos(cameraAngle), cameraDistance * 0.3f,
                   cameraDistance * sin(cameraAngle));
}

void Simulation::handleInput(GLFWwindow *window) {
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
    paused = !paused;
    glfwWaitEventsTimeout(0.2);
  }

  if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
    timeScale = glm::min(timeScale * 1.1f, 10.0f);
  }
  if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
    timeScale = glm::max(timeScale * 0.9f, 0.1f);
  }

  if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
    cameraDistance = glm::max(cameraDistance - 1.0f, 10.0f);
  }
  if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
    cameraDistance = glm::min(cameraDistance + 1.0f, 200.0f);
  }

  if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
    bodies.clear();
    setupScene();
    glfwWaitEventsTimeout(0.2);
  }
}

void Simulation::checkShaderCompilation(GLuint shader,
                                        const std::string &type) {
  GLint success;
  GLchar infoLog[1024];
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shader, 1024, NULL, infoLog);
    std::cerr << "Shader compilation error (" << type << "): " << infoLog
              << std::endl;
  }
}

void Simulation::checkProgramLinking(GLuint program) {
  GLint success;
  GLchar infoLog[1024];
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(program, 1024, NULL, infoLog);
    std::cerr << "Program linking error: " << infoLog << std::endl;
  }
}
