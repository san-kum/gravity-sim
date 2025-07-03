#include "include/simulation.h"
#include "include/octreeNode.h"
#include <GLFW/glfw3.h>
#include <cmath>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <limits>
#include <memory>
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
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    if (dot(circCoord, circCoord) > 1.0) {
        discard;
    }
    
    float dist = length(circCoord);
    float alpha = 1.0 - smoothstep(0.0, 1.0, dist);
    
    color = vec4(FragColor, alpha);
}
)";

const char *Simulation::trajectoryVertexShaderSource = R"(
	#version 330 core
	layout (location = 0) in vec3 aPos;

	uniform mat4 view;
	uniform mat4 projection;
	uniform vec3 color;
	uniform float alpha;

	out vec4 vertexColor;

	void main()
	{
	    gl_Position = projection * view * vec4(aPos, 1.0);
	    vertexColor = vec4(color, alpha);
	}
)";

const char *Simulation::trajectoryFragmentShaderSource = R"(
	#version 330 core
	in vec4 vertexColor;
	out vec4 color;

	void main()
	{
	    color = vertexColor;
	}
)";

Simulation::Simulation()
    : G(DEFAULT_GRAVITATIONAL_CONSTANT),
      cameraDistance(DEFAULT_CAMERA_DISTANCE), cameraAngle(0.0f), paused(false),
      timeScale(DEFAULT_TIME_SCALE), showTrajectories(false),
      useBarnesHut(true), trajectoryUpdateCounter(0), spaceMin(-1000.0f),
      spaceMax(1000.f) {
  setupShaders();
  setupGeometry();
  setupTrajectoryGeometry();
  setupScene();

  calculateBounds();
  glm::vec3 center = (spaceMin + spaceMax) * 0.5f;
  float size = glm::length(spaceMax - spaceMin);
  octreeRoot = std::make_unique<OctreeNode>(center, size);

  std::cout << "Barnes-Hut algorithm initialized\n";
  std::cout << "Press 'B' to toggle between Barnes-Hut and N-body "
               "calculation\n";
}

Simulation::~Simulation() {
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glDeleteProgram(shaderProgram);
  glDeleteVertexArrays(1, &trajectoryVAO);
  glDeleteBuffers(1, &trajectoryVBO);
  glDeleteProgram(trajectoryShaderProgram);
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

  // Trajectory shaders
  GLuint trajectoryVertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(trajectoryVertex, 1, &trajectoryVertexShaderSource, NULL);
  glCompileShader(trajectoryVertex);
  checkShaderCompilation(trajectoryVertex, "TRAJECTORY_VERTEX");

  GLuint trajectoryFragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(trajectoryFragment, 1, &trajectoryFragmentShaderSource, NULL);
  glCompileShader(trajectoryFragment);
  checkShaderCompilation(trajectoryFragment, "TRAJECTORY_FRAGMENT");

  trajectoryShaderProgram = glCreateProgram();
  glAttachShader(trajectoryShaderProgram, trajectoryVertex);
  glAttachShader(trajectoryShaderProgram, trajectoryFragment);
  glLinkProgram(trajectoryShaderProgram);
  checkProgramLinking(trajectoryShaderProgram);

  glDeleteShader(trajectoryVertex);
  glDeleteShader(trajectoryFragment);
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

void Simulation::setupTrajectoryGeometry() {
  glGenVertexArrays(1, &trajectoryVAO);
  glGenBuffers(1, &trajectoryVBO);

  glBindVertexArray(trajectoryVAO);
  glBindBuffer(GL_ARRAY_BUFFER, trajectoryVBO);
  glBufferData(GL_ARRAY_BUFFER,
               CelestialBody::MAX_TRAJECTORY_POINTS * 3 * sizeof(float), NULL,
               GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
}

void Simulation::setupScene() {
  // central object fixed (e.g., sun)
  bodies.emplace_back(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f), 1000.0f,
                      5.0f, glm::vec3(1.0f, 1.0f, 0.0f), true);

  // random objects to orbit around the central object
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 2.0 * M_PI);

  // inner objects -> faster and closer orbits
  for (int i = 0; i < 100; i++) {
    float distance = 8.0f + i * 4.0f;
    float angle = dis(gen);
    float orbitalSpeed = sqrt(G * 1000.0f / distance) * 0.8f;

    glm::vec3 pos(distance * cos(angle), 0.0f, distance * sin(angle));
    glm::vec3 vel(-orbitalSpeed * sin(angle), 0.0f, orbitalSpeed * cos(angle));

    bodies.emplace_back(pos, vel, 1.0f + i * 0.5f, 0.3f + i * 0.1f,
                        glm::vec3(0.3f + i * 0.2f, 0.5f, 1.0f - i * 0.2f));
  }

  // outer objects -> slower and longer orbits
  for (int i = 0; i < 100; i++) {
    float distance = 25.0f + i * 8.0f;
    float angle = dis(gen);
    float orbitalSpeed = sqrt(G * 1000.0f / distance) * 0.7f;

    glm::vec3 pos(distance * cos(angle), 0.0f, distance * sin(angle));
    glm::vec3 vel(-orbitalSpeed * sin(angle), 0.0f, orbitalSpeed * cos(angle));

    bodies.emplace_back(pos, vel, 0.5f + i * 0.3f, 0.2f + i * 0.1f,
                        glm::vec3(1.0f - i * 0.2f, 0.3f + i * 0.2f, 0.5f));
  }

  // objects between inner and outer objects (small debris)
  for (int i = 0; i < 500; i++) {
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
  calculateBounds();
}

void Simulation::calculateBounds() {
  if (bodies.empty()) {
    spaceMin = glm::vec3(-1000.0f);
    spaceMax = glm::vec3(1000.0f);
    return;
  }

  spaceMin = glm::vec3(std::numeric_limits<float>::max());
  spaceMax = glm::vec3(std::numeric_limits<float>::lowest());

  for (const auto &body : bodies) {
    spaceMin = glm::min(spaceMin, body.position);
    spaceMax = glm::max(spaceMax, body.position);
  }

  glm::vec3 padding = (spaceMax - spaceMin) * 0.2f;
  spaceMax -= padding;
  spaceMax += padding;

  glm::vec3 size = spaceMax - spaceMin;
  float minSize = 100.f;
  if (glm::length(size) < minSize) {
    glm::vec3 center = (spaceMin + spaceMax) * 0.5f;
    spaceMin = center - glm::vec3(minSize * 0.5f);
    spaceMax = center + glm::vec3(minSize * 0.5f);
  }
}

void Simulation::buildOctree() {
  calculateBounds();
  glm::vec3 center = (spaceMin + spaceMax) * 0.5f;
  float size = glm::length(spaceMax - spaceMin);
  octreeRoot = std::make_unique<OctreeNode>(center, size);

  for (auto &body : bodies)
    octreeRoot->insertBody(&body);

  octreeRoot->updateMassProperties();
}

void Simulation::updateGravityBarnesHut() {
  buildOctree();

  for (auto &body : bodies) {
    if (!body.isFixed) {
      body.acceleration = glm::vec3(0.0f);
      octreeRoot->calculateForce(body, G, BARNES_HUT_THETA);
    }
  }
}

void Simulation::updateGravityDirect() {
  for (size_t i = 0; i < bodies.size(); i++) {
    if (!bodies[i].isFixed)
      bodies[i].acceleration = glm::vec3(0.0f);

    for (size_t j = 0; j < bodies.size(); j++) {
      if (i != j)
        bodies[i].applyGravity(bodies[j], G);
    }
  }
}

void Simulation::update(float deltaTime) {
  if (paused)
    return;

  float dt = deltaTime * timeScale;

  if (useBarnesHut)
    updateGravityBarnesHut();
  else
    updateGravityDirect();

  for (auto &body : bodies) {
    body.update(dt);
  }

  // update trajectories
  trajectoryUpdateCounter++;
  if (trajectoryUpdateCounter >= 1) {
    trajectoryUpdateCounter = 0;
    for (auto &body : bodies) {
      if (!body.isFixed)
        body.addTrajectoryPoint();
    }
  }
}

void Simulation::render(int width, int height) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  updateCamera(width, height);

  if (showTrajectories)
    renderTrajectories();

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
    float pointSize = (body.radius * POINT_SCALE_SIZE) / distance;
    pointSize = glm::clamp(pointSize, MIN_POINT_SIZE, MAX_POINT_SIZE);
    glPointSize(pointSize);

    float colorData[] = {0.0f,         0.0f,         0.0f,
                         body.color.r, body.color.g, body.color.b};
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(colorData), colorData);

    glDrawArrays(GL_POINTS, 0, 1);
  }

  glDisable(GL_BLEND);
}

void Simulation::renderTrajectories() {
  glUseProgram(trajectoryShaderProgram);
  glUniformMatrix4fv(glGetUniformLocation(trajectoryShaderProgram, "view"), 1,
                     GL_FALSE, glm::value_ptr(view));
  glUniformMatrix4fv(
      glGetUniformLocation(trajectoryShaderProgram, "projection"), 1, GL_FALSE,
      glm::value_ptr(projection));

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2.0f);

  glBindVertexArray(trajectoryVAO);

  for (const auto &body : bodies) {
    if (body.isFixed || body.trajectory.size() < 2)
      continue;

    std::vector<float> trajectoryData;
    for (const auto &point : body.trajectory) {
      trajectoryData.push_back(point.x);
      trajectoryData.push_back(point.y);
      trajectoryData.push_back(point.z);
    }
    if (trajectoryData.empty())
      continue;

    glBindBuffer(GL_ARRAY_BUFFER, trajectoryVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, trajectoryData.size() * sizeof(float),
                    trajectoryData.data());

    glm::vec3 trajectoryColor = body.color * 0.3f + glm::vec3(0.1f);
    glUniform3f(glGetUniformLocation(trajectoryShaderProgram, "color"),
                trajectoryColor.r, trajectoryColor.g, trajectoryColor.b);
    glUniform1f(glGetUniformLocation(trajectoryShaderProgram, "alpha"), 0.2f);

    glDrawArrays(GL_LINE_STRIP, 0, body.trajectory.size());
  }
  glDisable(GL_BLEND);
}

void Simulation::updateCamera(int width, int height) {
  cameraAngle += CAMERA_ROTATION_SPEED;

  glm::vec3 cameraPos =
      glm::vec3(cameraDistance * cos(cameraAngle),
                cameraDistance * CAMERA_ELEVATION_FACTOR, // slight elevation
                cameraDistance * sin(cameraAngle));

  view = glm::lookAt(cameraPos, glm::vec3(0.0f, 0.0f, 0.0f),
                     glm::vec3(0.0f, 1.0f, 0.0f));

  projection = glm::perspective(glm::radians(45.0f),
                                (float)width / (float)height, 0.1f, 1000.0f);
}

glm::vec3 Simulation::getCameraPosition() {
  return glm::vec3(cameraDistance * cos(cameraAngle),
                   cameraDistance * CAMERA_ELEVATION_FACTOR,
                   cameraDistance * sin(cameraAngle));
}

void Simulation::handleInput(GLFWwindow *window) {

  static bool spacePressed = false;
  static bool tPressed = false;
  static bool rPressed = false;
  static bool bPressed = false;

  // Toggle pause
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS && !spacePressed) {
    paused = !paused;
    spacePressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
    spacePressed = false;

  // Toggle trajectories
  if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && !tPressed) {
    showTrajectories = !showTrajectories;
    // std::cout << "Trajectories " << (showTrajectories ? 1 : 0) << std::endl;
    tPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_T) == GLFW_RELEASE)
    tPressed = false;

  // Toggle algorithm
  if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS && !bPressed) {
    useBarnesHut = !useBarnesHut;
    std::cout << "Using " << (useBarnesHut ? "Barnes-Hut" : "n-body")
              << " algoritm\n";
    bPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_RELEASE)
    bPressed = false;

  // WASD
  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    timeScale = glm::min(timeScale * 1.1f, 10.0f);
  }
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    timeScale = glm::max(timeScale * 0.9f, 0.1f);
  }

  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    cameraDistance = glm::max(cameraDistance - 1.0f, 10.0f);
  }
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    cameraDistance = glm::min(cameraDistance + 1.0f, 200.0f);
  }

  if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS && !rPressed) {
    bodies.clear();
    setupScene();
    rPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_R) == GLFW_RELEASE)
    rPressed = false;
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
