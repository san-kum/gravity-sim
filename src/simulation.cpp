#include "include/simulation.h"
#include "include/octreeNode.h"
#include <GLFW/glfw3.h>
#include <cassert>
#include <cmath>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <omp.h>
#include <random>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#define STB_EASY_FONT_IMPLEMENTATION
#include "include/stb_easy_font.h"

static GLuint textVAO = 0, textVBO = 0, textShader = 0;

static void initTextGL() {
  if (textShader)
    return;
  const char *vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        uniform vec2 uScreen;
        void main() {
            vec2 pos = aPos / uScreen * 2.0 - 1.0;
            pos.y = -pos.y;
            gl_Position = vec4(pos.x, pos.y, 0.0, 1.0);
        }
    )";
  GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
  glCompileShader(vertexShader);
  GLint success;
  glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char infoLog[512];
    glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
    glDeleteShader(vertexShader);
    return;
  }
  const char *fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        uniform vec3 uColor;
        void main() {
            FragColor = vec4(uColor, 1.0);
        }
    )";
  GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
  glCompileShader(fragmentShader);
  glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char infoLog[512];
    glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return;
  }
  textShader = glCreateProgram();
  glAttachShader(textShader, vertexShader);
  glAttachShader(textShader, fragmentShader);
  glLinkProgram(textShader);
  glGetProgramiv(textShader, GL_LINK_STATUS, &success);
  if (!success) {
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteProgram(textShader);
    textShader = 0;
    return;
  }
  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);
  glGenVertexArrays(1, &textVAO);
  glGenBuffers(1, &textVBO);
  glBindVertexArray(textVAO);
  glBindBuffer(GL_ARRAY_BUFFER, textVBO);
  glBufferData(GL_ARRAY_BUFFER, 9999, nullptr, GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 16, (void *)0);
  glEnableVertexAttribArray(0);
  glBindVertexArray(0);
}

// quad
static GLuint discVAO = 0, discVBO = 0, discShader = 0;
static void setupBillboardGeometry();
static void setupDiscShaders();

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
    layout (location = 1) in float aAlpha;

    uniform mat4 view;
    uniform mat4 projection;
    uniform vec3 color;

    out vec4 vertexColor;

    void main()
    {
        gl_Position = projection * view * vec4(aPos, 1.0);
        vertexColor = vec4(color, aAlpha);
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

const char *nbodyComputeShaderSource = R"(
#version 430
layout(local_size_x = 256) in;

layout(std430, binding = 0) buffer Pos {
    vec4 positions[];
};
layout(std430, binding = 1) buffer Vel {
    vec4 velocities[];
};
layout(std430, binding = 2) buffer Acc {
    vec4 accelerations[];
};
layout(std430, binding = 3) buffer Mass {
    float masses[];
};

uniform float G;
uniform int numBodies;

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= numBodies) return;
    vec3 acc = vec3(0.0);
    vec3 pos_i = positions[i].xyz;
    float mass_i = masses[i];
    for (uint j = 0; j < numBodies; ++j) {
        if (i == j) continue;
        vec3 dir = positions[j].xyz - pos_i;
        float distSqr = dot(dir, dir) + 0.01; 
        float invDist = inversesqrt(distSqr);
        float invDist3 = invDist * invDist * invDist;
        acc += G * masses[j] * dir * invDist3;
    }
    accelerations[i] = vec4(acc, 0.0);
}
)";

const char *Simulation::fabricVertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
uniform mat4 view;
uniform mat4 projection;
out float vHeight;
void main() {
    vHeight = aPos.y;
    gl_Position = projection * view * vec4(aPos.x, 0.0, aPos.z, 1.0) + vec4(0, aPos.y, 0, 0);
}
)";

const char *Simulation::fabricFragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;
uniform vec3 uColor;
uniform float uAlpha;
void main() {
    FragColor = vec4(uColor, uAlpha);
}
)";

Simulation::Simulation()
    : G(DEFAULT_GRAVITATIONAL_CONSTANT),
      cameraDistance(DEFAULT_CAMERA_DISTANCE), cameraAngle(0.0f),
      targetCameraDistance(DEFAULT_CAMERA_DISTANCE), targetCameraAngle(0.0f),
      paused(false), timeScale(DEFAULT_TIME_SCALE), showTrajectories(false),
      useBarnesHut(true), trajectoryUpdateCounter(0), spaceMin(-1000.0f),
      spaceMax(1000.f),
      algorithm(GravityAlgorithm::BarnesHut) { // Default to Barnes-Hut
  fabricGridRes = 48;
  setupShaders();
  setupGeometry();
  setupTrajectoryGeometry();
  setupScene();
  initFabricMesh();

  calculateBounds();
  glm::vec3 center = (spaceMin + spaceMax) * 0.5f;
  float size = glm::length(spaceMax - spaceMin);
  octreeRoot = std::make_unique<OctreeNode>(center, size);
  initComputeShaderAndBuffers();
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
  glGenBuffers(1, &trajectoryAlphaVBO);

  glBindVertexArray(trajectoryVAO);
  glBindBuffer(GL_ARRAY_BUFFER, trajectoryVBO);
  glBufferData(GL_ARRAY_BUFFER,
               CelestialBody::MAX_TRAJECTORY_POINTS * 3 * sizeof(float), NULL,
               GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, trajectoryAlphaVBO);
  glBufferData(GL_ARRAY_BUFFER,
               CelestialBody::MAX_TRAJECTORY_POINTS * sizeof(float), NULL,
               GL_DYNAMIC_DRAW);
  glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);

  glBindVertexArray(0);
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
  for (int i = 0; i < 50; i++) {
    float distance = 6.0f + i * 2.0f;
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
#pragma omp parallel for schedule(static) // CPU parallelization
  for (size_t i = 0; i < bodies.size(); i++) {
    if (!bodies[i].isFixed) {
      bodies[i].acceleration = glm::vec3(0.0f);
      octreeRoot->calculateForce(bodies[i], G, BARNES_HUT_THETA);
    }
  }
}

void Simulation::updateGravityDirect() {
#pragma omp parallel for schedule(static) // CPU parallelization
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

  float camLerp =
      1.0f -
      expf(-6.0f *
           deltaTime); // fast, smooth(cinematic transistions or whatever :p)
  cameraDistance =
      cameraDistance + (targetCameraDistance - cameraDistance) * camLerp;
  cameraAngle = cameraAngle + (targetCameraAngle - cameraAngle) * camLerp;

  // camera focus on click
  if (mouseLeftDown && hoveredBodyIndex >= 0) {
    focusedBodyIndex = hoveredBodyIndex;
  }

  switch (algorithm) {
  case GravityAlgorithm::BarnesHut:
    // FIX: GPU for Barnes Hut and FMM doesn't work
    if (computeDevice == ComputeDevice::GPU) {
      std::cout << "Barnes-Hut on GPU not implemented, falling back to CPU.\n";
      updateGravityBarnesHut();
    } else {
      updateGravityBarnesHut();
    }
    break;
  case GravityAlgorithm::Direct:
    if (computeDevice == ComputeDevice::GPU) {
      updateGravityGPU();
    } else {
      updateGravityDirect();
    }
    break;
  case GravityAlgorithm::FMM:
    if (computeDevice == ComputeDevice::GPU) {
      std::cout << "FMM on GPU not implemented, falling back to CPU.\n";
      updateGravityFMM();
    } else {
      updateGravityFMM();
    }
    break;
  default:
    assert(false && "Unknown gravity algorithm");
  }

#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < bodies.size(); i++) {
    bodies[i].update(dt);
  }

  trajectoryUpdateCounter++;
  if (trajectoryUpdateCounter >= 1) {
    trajectoryUpdateCounter = 0;
    for (auto &body : bodies) {
      if (!body.isFixed)
        body.addTrajectoryPoint();
    }
  }

  handleCollisions();
}

void Simulation::handleCollisions() {
  std::vector<bool> merged(bodies.size(), false);
  for (size_t i = 0; i < bodies.size(); ++i) {
    if (merged[i])
      continue;
    for (size_t j = i + 1; j < bodies.size(); ++j) {
      if (merged[j])
        continue;
      auto &a = bodies[i];
      auto &b = bodies[j];
      float minDist = a.radius + b.radius;
      float dist = glm::length(a.position - b.position);
      if (dist < minDist && dist > 0.0f) {
        bool aIsSun = (i == 0);
        bool bIsSun = (j == 0);
        if (!aIsSun && !bIsSun) {
          float totalMass = a.mass + b.mass;
          glm::vec3 newPos =
              (a.position * a.mass + b.position * b.mass) / totalMass;
          glm::vec3 newVel =
              (a.velocity * a.mass + b.velocity * b.mass) / totalMass;
          a.mass = totalMass;
          a.position = newPos;
          a.velocity = newVel;
          a.radius = std::cbrt(a.radius * a.radius * a.radius +
                               b.radius * b.radius * b.radius);
          merged[j] = true;
        } else {
          glm::vec3 n = glm::normalize(b.position - a.position);
          float va = glm::dot(a.velocity, n);
          float vb = glm::dot(b.velocity, n);
          float ma = a.mass, mb = b.mass;
          float vaNew = (va * (ma - mb) + 2 * mb * vb) / (ma + mb);
          float vbNew = (vb * (mb - ma) + 2 * ma * va) / (ma + mb);
          a.velocity += (vaNew - va) * n;
          b.velocity += (vbNew - vb) * n;
        }
      }
    }
  }
  for (int i = (int)bodies.size() - 1; i >= 0; --i) {
    if (merged[i])
      bodies.erase(bodies.begin() + i);
  }
}

void Simulation::render(int width, int height) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // --- animated cinematic background ---
  renderStarfield(width, height, (float)glfwGetTime());
  if (showFabric) {
    updateFabricMesh();
    renderFabricMesh();
  }
  updateCamera(width, height);

  if (showTrajectories)
    renderTrajectories();

  hoveredBodyIndex = -1;
  double minDist = 1e9;
  glm::mat4 vp = projection * view;
  for (size_t i = 0; i < bodies.size(); ++i) {
    const auto &body = bodies[i];
    glm::vec4 clip = vp * glm::vec4(body.position, 1.0f);
    if (clip.w <= 0.0f)
      continue;
    glm::vec3 ndc = glm::vec3(clip) / clip.w;
    int sx = int((ndc.x * 0.5f + 0.5f) * width);
    int sy = int((1.0f - (ndc.y * 0.5f + 0.5f)) * height);
    double dx = mouseX - sx;
    double dy = mouseY - sy;
    double dist = sqrt(dx * dx + dy * dy);
    float baseRadius = sqrt(body.radius);
    float pointSize;
    if (i == 0)
      pointSize = 100.0f * baseRadius / (cameraDistance * 0.05f + 1.0f);
    else if (i <= 100)
      pointSize = 30.0f * baseRadius / (cameraDistance * 0.08f + 1.0f);
    else
      pointSize = 10.0f * baseRadius / (cameraDistance * 0.12f + 1.0f);

    pointSize = glm::clamp(pointSize, 4.0f, 80.0f);
    if (dist < pointSize * 0.6 && dist < minDist) {
      hoveredBodyIndex = i;
      minDist = dist;
    }
  }

  float depthBlend = 0.0f;
  glUseProgram(shaderProgram);
  glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE,
                     glm::value_ptr(view));
  glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1,
                     GL_FALSE, glm::value_ptr(projection));
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBindVertexArray(VAO);

  if (!bodies.empty()) {
    const auto &sun = bodies[0];
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, sun.position);
    model = glm::scale(model, glm::vec3(sun.radius));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1,
                       GL_FALSE, glm::value_ptr(model));
    float distance = glm::length(sun.position - getCameraPosition());
    float zoomNorm = cameraDistance;
    float effectiveDist =
        (1.0f - depthBlend) * zoomNorm + depthBlend * distance;
    float baseRadius = sqrt(sun.radius);
    float sunSize = 100.0f * baseRadius / (effectiveDist * 0.05f + 1.0f);
    sunSize = glm::clamp(sunSize, 4.0f, 80.0f);
    float glowSize = sunSize * 2.7f;
    glPointSize(glowSize);
    glm::vec3 glowColor = glm::vec3(1.0f, 0.8f, 0.3f);
    float colorData[] = {0.0f,        0.0f,        0.0f,
                         glowColor.r, glowColor.g, glowColor.b};
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(colorData), colorData);
    glDrawArrays(GL_POINTS, 0, 1);
    glPointSize(sunSize);
  }

  for (size_t i = 0; i < bodies.size(); ++i) {
    const auto &body = bodies[i];
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, body.position);
    model = glm::scale(model, glm::vec3(body.radius));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1,
                       GL_FALSE, glm::value_ptr(model));
    float distance = glm::length(body.position - getCameraPosition());
    float zoomNorm = cameraDistance;
    float effectiveDist =
        (1.0f - depthBlend) * zoomNorm + depthBlend * distance;
    float baseRadius = sqrt(body.radius);
    float pointSize;
    if (i == 0)
      pointSize = 100.0f * baseRadius / (effectiveDist * 0.05f + 1.0f);
    else if (i <= 100)
      pointSize = 30.0f * baseRadius / (effectiveDist * 0.08f + 1.0f);
    else
      pointSize = 10.0f * baseRadius / (effectiveDist * 0.12f + 1.0f);

    pointSize = glm::clamp(pointSize, 4.0f, 80.0f);
    glm::vec3 color;
    if (i == 0)
      color = glm::vec3(1.0f, 0.7f, 0.3f); // sun: orange
    else if (i <= 100)
      color = glm::vec3(0.5f, 0.7f, 1.0f); // planets: blue
    else
      color = glm::vec3(0.95f, 0.95f, 0.95f); // asteroids: white

    // highlight if hovered
    if ((int)i == hoveredBodyIndex) {
      pointSize *= 1.35f;
      color = glm::min(color * 1.5f, glm::vec3(1.0f));
    }
    float colorData[] = {0.0f, 0.0f, 0.0f, color.r, color.g, color.b};
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(colorData), colorData);
    glPointSize(pointSize);
    glDrawArrays(GL_POINTS, 0, 1);
  }
  glDisable(GL_BLEND);

  // adjusting mass and radius in real-time(hovered/focused)
  int idx = hoveredBodyIndex >= 0 ? hoveredBodyIndex : focusedBodyIndex;
  if (idx > 0 && idx < (int)bodies.size()) {
    static bool plusPressed = false, minusPressed = false,
                lbracketPressed = false, rbracketPressed = false;
    GLFWwindow *window = glfwGetCurrentContext();
    if ((glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS ||
         glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS) &&
        !plusPressed) {
      bodies[idx].mass *= 1.05f;
      plusPressed = true;
    } else if (glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_RELEASE &&
               glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_RELEASE)
      plusPressed = false;

    if ((glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS ||
         glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS) &&
        !minusPressed) {
      bodies[idx].mass *= 0.95f;
      minusPressed = true;
    } else if (glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_RELEASE &&
               glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_RELEASE)
      minusPressed = false;

    if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS &&
        !lbracketPressed) {
      bodies[idx].radius *= 0.95f;
      lbracketPressed = true;
    } else if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_RELEASE)
      lbracketPressed = false;

    if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS &&
        !rbracketPressed) {
      bodies[idx].radius *= 1.05f;
      rbracketPressed = true;
    } else if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_RELEASE)
      rbracketPressed = false;
  }
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
  glLineWidth(1.0f);

  glBindVertexArray(trajectoryVAO);

  for (size_t i = 0; i < bodies.size(); ++i) {
    const auto &body = bodies[i];
    if (body.isFixed || body.trajectory.size() < 2)
      continue;

    size_t N = body.trajectory.size();
    int window = 10;
    std::vector<glm::vec3> smoothedPoints(N);
    glm::vec3 sum(0.0f);
    int half = window / 2;
    for (size_t j = 0; j < N; ++j) {
      if (j == 0) {
        sum = glm::vec3(0.0f);
        int count = 0;
        for (int k = int(j) - half; k <= int(j) + half; ++k) {
          if (k >= 0 && k < int(N)) {
            sum += body.trajectory[k];
            count++;
          }
        }
        smoothedPoints[j] = sum / float(count);
      } else {
        int prev_out = int(j) - half - 1;
        int next_in = int(j) + half;
        int count = 0;
        if (prev_out >= 0 && prev_out < int(N)) {
          sum -= body.trajectory[prev_out];
        }
        if (next_in >= 0 && next_in < int(N)) {
          sum += body.trajectory[next_in];
        }
        int left = std::max(0, int(j) - half);
        int right = std::min(int(N) - 1, int(j) + half);
        count = right - left + 1;
        smoothedPoints[j] = sum / float(count);
      }
    }
    std::vector<float> trajectoryData;
    std::vector<float> alphaData;
    for (size_t j = 0; j < N; ++j) {
      const auto &point = smoothedPoints[j];
      trajectoryData.push_back(point.x);
      trajectoryData.push_back(point.y);
      trajectoryData.push_back(point.z);
      float alpha = 0.005f + 0.495f * (float(j) / float(N - 1));
      alphaData.push_back(alpha);
    }
    if (trajectoryData.empty())
      continue;
    glBindBuffer(GL_ARRAY_BUFFER, trajectoryVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, trajectoryData.size() * sizeof(float),
                    trajectoryData.data());
    glBindBuffer(GL_ARRAY_BUFFER, trajectoryAlphaVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, alphaData.size() * sizeof(float),
                    alphaData.data());
    glm::vec3 color;
    if (i == 0)
      color = glm::vec3(1.0f, 0.7f, 0.3f); // sun: orange
    else if (i <= 100)
      color = glm::vec3(0.5f, 0.7f, 1.0f); // planets: blue
    else
      color = glm::vec3(0.95f, 0.95f, 0.95f); // asteroids: white
    glm::vec3 trajectoryColor = color * 0.5f + glm::vec3(0.5f);
    glUniform3f(glGetUniformLocation(trajectoryShaderProgram, "color"),
                trajectoryColor.r, trajectoryColor.g, trajectoryColor.b);
    glDrawArrays(GL_LINE_STRIP, 0, N);
  }
  glDisable(GL_BLEND);
}

void Simulation::updateCamera(int width, int height) {
  // smooth transition
  glm::vec3 focusTarget(0.0f);
  if (focusedBodyIndex >= 0 && focusedBodyIndex < (int)bodies.size())
    focusTarget = bodies[focusedBodyIndex].position;

  cameraAngle += CAMERA_ROTATION_SPEED;
  glm::vec3 cameraPos =
      glm::vec3(cameraDistance * cos(cameraAngle),
                cameraDistance * CAMERA_ELEVATION_FACTOR, // slight elevation
                cameraDistance * sin(cameraAngle));
  view = glm::lookAt(cameraPos + focusTarget, focusTarget,
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
  static bool mPressed = false;
  static bool nPressed = false;
  static bool gPressed = false;
  static bool iPressed = false;
  static bool fPressed = false;

  // Toggle pause
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS && !spacePressed) {
    paused = !paused;
    spacePressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
    spacePressed = false;

  // Toggle trajectories
  if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && !tPressed) {
    showTrajectories = !showTrajectories;
    tPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_T) == GLFW_RELEASE)
    tPressed = false;

  // Toggle Barnes-Hut algorithm
  if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS && !bPressed) {
    if (algorithm != GravityAlgorithm::BarnesHut) {
      algorithm = GravityAlgorithm::BarnesHut;
      std::cout << "Using Barnes-Hut algorithm\n";
    }
    bPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_RELEASE)
    bPressed = false;

  // Toggle FMM algorithm
  if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS && !mPressed) {
    if (algorithm != GravityAlgorithm::FMM) {
      algorithm = GravityAlgorithm::FMM;
      std::cout << "Using Fast Multipole Method (FMM) algorithm\n";
    }
    mPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_RELEASE)
    mPressed = false;

  // Toggle Direct algorithm
  if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS && !nPressed) {
    if (algorithm != GravityAlgorithm::Direct) {
      algorithm = GravityAlgorithm::Direct;
      std::cout << "Using Direct N-body algorithm\n";
    }
    nPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_N) == GLFW_RELEASE)
    nPressed = false;

  // Toggle compute device (CPU/GPU)
  if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS && !gPressed) {
    if (computeDevice == ComputeDevice::CPU && gpuSupported) {
      computeDevice = ComputeDevice::GPU;
      std::cout << "Switched to GPU for current algorithm\n";
    } else {
      computeDevice = ComputeDevice::CPU;
      std::cout << "Switched to CPU for current algorithm\n";
    }
    gPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_G) == GLFW_RELEASE) {
    gPressed = false;
  }

  // WASD
  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    timeScale = glm::min(timeScale * 1.1f, 10.0f);

  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    timeScale = glm::max(timeScale * 0.9f, 0.1f);

  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    targetCameraDistance = glm::max(targetCameraDistance - 1.0f, 10.0f);

  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    targetCameraDistance = glm::min(targetCameraDistance + 1.0f, 200.0f);

  if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS && !rPressed) {
    bodies.clear();
    setupScene();
    rPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_R) == GLFW_RELEASE)
    rPressed = false;

  // print info
  if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS && !iPressed) {
    printBodiesInfo();
    iPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_I) == GLFW_RELEASE) {
    iPressed = false;
  }

  if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS && !fPressed) {
    showFabric = !showFabric;
    fPressed = true;
  } else if (glfwGetKey(window, GLFW_KEY_F) == GLFW_RELEASE) {
    fPressed = false;
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

// Stub for FMM
template <typename T> void unused(const T &) {}
void Simulation::updateGravityFMM() {
  buildOctree();
  octreeRoot->upwardPass();
  glm::vec3 rootLocalExpansion(0.0f);
  octreeRoot->downwardPass(rootLocalExpansion);
#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < bodies.size(); i++) {
    if (!bodies[i].isFixed) {
      bodies[i].acceleration = glm::vec3(0.0f);
      octreeRoot->evaluateLocalExpansion(bodies[i], G);
    }
  }
}

void Simulation::updateGravityGPU() {
  std::vector<glm::vec4> posData, velData, accData;
  std::vector<float> massData;
  posData.reserve(bodies.size());
  velData.reserve(bodies.size());
  accData.resize(bodies.size(), glm::vec4(0.0f));
  massData.reserve(bodies.size());
  for (const auto &body : bodies) {
    posData.emplace_back(body.position, 0.0f);
    velData.emplace_back(body.velocity, 0.0f);
    massData.push_back(body.mass);
  }
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, posSSBO);
  glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
                  posData.size() * sizeof(glm::vec4), posData.data());
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, velSSBO);
  glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
                  velData.size() * sizeof(glm::vec4), velData.data());
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, massSSBO);
  glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, massData.size() * sizeof(float),
                  massData.data());
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, accSSBO);
  glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
                  accData.size() * sizeof(glm::vec4), accData.data());

  // dispatch compute shader
  glUseProgram(computeShaderProgram);
  glUniform1f(glGetUniformLocation(computeShaderProgram, "G"), G);
  glUniform1i(glGetUniformLocation(computeShaderProgram, "numBodies"),
              (int)bodies.size());
  GLuint numGroups = (GLuint)((bodies.size() + 255) / 256);
  glDispatchCompute(numGroups, 1, 1);
  glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

  // back accelerations
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, accSSBO);
  void *ptr = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
  if (ptr) {
    memcpy(accData.data(), ptr, accData.size() * sizeof(glm::vec4));
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
  }
  for (size_t i = 0; i < bodies.size(); ++i) {
    if (!bodies[i].isFixed)
      bodies[i].acceleration = glm::vec3(accData[i]);
    else
      bodies[i].acceleration = glm::vec3(0.0f);
  }
}

void Simulation::initFabricMesh() {
  // grid in XZ plane, y = 0, covering [-gridSize, gridSize]
  int N = fabricGridRes;
  float gridSize = 80.0f;
  fabricVertices.clear();
  fabricIndices.clear();
  for (int z = 0; z < N; ++z) {
    for (int x = 0; x < N; ++x) {
      float fx = ((float)x / (N - 1) - 0.5f) * 2.0f * gridSize;
      float fz = ((float)z / (N - 1) - 0.5f) * 2.0f * gridSize;
      fabricVertices.emplace_back(fx, 0.0f, fz);
    }
  }
  for (int z = 0; z < N - 1; ++z) {
    for (int x = 0; x < N - 1; ++x) {
      int i0 = z * N + x;
      int i1 = z * N + (x + 1);
      int i2 = (z + 1) * N + x;
      int i3 = (z + 1) * N + (x + 1);
      fabricIndices.push_back(i0);
      fabricIndices.push_back(i2);
      fabricIndices.push_back(i1);
      fabricIndices.push_back(i1);
      fabricIndices.push_back(i2);
      fabricIndices.push_back(i3);
    }
  }
  // openGL buffers
  if (fabricVAO == 0)
    glGenVertexArrays(1, &fabricVAO);
  if (fabricVBO == 0)
    glGenBuffers(1, &fabricVBO);
  if (fabricEBO == 0)
    glGenBuffers(1, &fabricEBO);
  glBindVertexArray(fabricVAO);
  glBindBuffer(GL_ARRAY_BUFFER, fabricVBO);
  glBufferData(GL_ARRAY_BUFFER, fabricVertices.size() * sizeof(glm::vec3),
               fabricVertices.data(), GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void *)0);
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fabricEBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               fabricIndices.size() * sizeof(unsigned int),
               fabricIndices.data(), GL_STATIC_DRAW);
  glBindVertexArray(0);
  // Shader
  if (fabricShader == 0) {
    GLuint vert = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert, 1, &fabricVertexShaderSource, nullptr);
    glCompileShader(vert);
    GLuint frag = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag, 1, &fabricFragmentShaderSource, nullptr);
    glCompileShader(frag);
    fabricShader = glCreateProgram();
    glAttachShader(fabricShader, vert);
    glAttachShader(fabricShader, frag);
    glLinkProgram(fabricShader);
    glDeleteShader(vert);
    glDeleteShader(frag);
  }
}

void Simulation::updateFabricMesh() {
  if (++fabricUpdateCounter % 3 != 0)
    return;
  float Gvis = G;
  float scale = 0.12f;
  int N = fabricGridRes;

#pragma omp parallel for schedule(static)
  for (int i = 0; i < (int)fabricVertices.size(); ++i) {
    glm::vec3 v = fabricVertices[i];
    float y = 0.0f;
    glm::vec2 pos2d(v.x, v.z);
    for (const auto &body : bodies) {
      glm::vec2 b2d(body.position.x, body.position.z);
      float dist = glm::length(pos2d - b2d);
      dist = glm::max(dist, 1.0f);
      y += -Gvis * body.mass / dist;
    }
    fabricVertices[i].y = y * scale;
  }

  std::vector<float> smoothedY(fabricVertices.size(), 0.0f);
  for (int iter = 0; iter < 3; ++iter) {
    for (int z = 0; z < N; ++z) {
      for (int x = 0; x < N; ++x) {
        int idx = z * N + x;
        float sum = fabricVertices[idx].y;
        int count = 1;
        if (z > 0) {
          sum += fabricVertices[(z - 1) * N + x].y;
          count++;
        }
        if (z < N - 1) {
          sum += fabricVertices[(z + 1) * N + x].y;
          count++;
        }
        if (x > 0) {
          sum += fabricVertices[z * N + (x - 1)].y;
          count++;
        }
        if (x < N - 1) {
          sum += fabricVertices[z * N + (x + 1)].y;
          count++;
        }
        smoothedY[idx] = sum / count;
      }
    }
    for (int i = 0; i < (int)fabricVertices.size(); ++i) {
      fabricVertices[i].y = smoothedY[i];
    }
  }
  glBindBuffer(GL_ARRAY_BUFFER, fabricVBO);
  glBufferSubData(GL_ARRAY_BUFFER, 0, fabricVertices.size() * sizeof(glm::vec3),
                  fabricVertices.data());
}

void Simulation::renderFabricMesh() {
  glUseProgram(fabricShader);
  glUniformMatrix4fv(glGetUniformLocation(fabricShader, "view"), 1, GL_FALSE,
                     glm::value_ptr(view));
  glUniformMatrix4fv(glGetUniformLocation(fabricShader, "projection"), 1,
                     GL_FALSE, glm::value_ptr(projection));
  glBindVertexArray(fabricVAO);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glLineWidth(1.0f);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);
  // Set line color to even lighter gray and lower alpha
  GLint colorLoc = glGetUniformLocation(fabricShader, "uColor");
  if (colorLoc >= 0)
    glUniform3f(colorLoc, 0.85f, 0.85f, 0.85f);
  GLint alphaLoc = glGetUniformLocation(fabricShader, "uAlpha");
  if (alphaLoc >= 0)
    glUniform1f(alphaLoc, 0.25f);
  glDrawElements(GL_TRIANGLES, fabricIndices.size(), GL_UNSIGNED_INT, 0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBindVertexArray(0);
  glDisable(GL_BLEND);
}

static GLuint starVAO = 0, starVBO = 0, starShader = 0;

static void setupStarfieldGL(int width, int height, glm::vec2 *starPositions,
                             int numStars, bool &initialized) {
  if (initialized && starVAO && starVBO && starShader)
    return;
  // random star positions in NDC
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> xdist(-1.0f, 1.0f);
  std::uniform_real_distribution<float> ydist(-1.0f, 1.0f);
  for (int i = 0; i < numStars; ++i)
    starPositions[i] = glm::vec2(xdist(rng), ydist(rng));

  if (!starVAO)
    glGenVertexArrays(1, &starVAO);
  if (!starVBO)
    glGenBuffers(1, &starVBO);
  glBindVertexArray(starVAO);
  glBindBuffer(GL_ARRAY_BUFFER, starVBO);
  glBufferData(GL_ARRAY_BUFFER, numStars * sizeof(glm::vec2), starPositions,
               GL_STATIC_DRAW);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glBindVertexArray(0);

  if (!starShader) {
    const char *vertSrc = R"(
            #version 330 core
            layout (location = 0) in vec2 aPos;
            uniform float uTime;
            out float vTwinkle;
            void main() {
                float drift = 0.003 * sin(uTime * 0.1 + gl_VertexID);
                gl_Position = vec4(aPos.x + drift, aPos.y, 0.0, 1.0);
                vTwinkle = 0.85 + 0.15 * sin(uTime * 0.7 + gl_VertexID);
                gl_PointSize = 1.3;
            }
        )";
    const char *fragSrc = R"(
            #version 330 core
            in float vTwinkle;
            out vec4 FragColor;
            void main() {
                FragColor = vec4(1.0, 1.0, 1.0, vTwinkle * 0.5);
            }
        )";
    GLuint vert = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert, 1, &vertSrc, nullptr);
    glCompileShader(vert);
    GLuint frag = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag, 1, &fragSrc, nullptr);
    glCompileShader(frag);
    starShader = glCreateProgram();
    glAttachShader(starShader, vert);
    glAttachShader(starShader, frag);
    glLinkProgram(starShader);
    glDeleteShader(vert);
    glDeleteShader(frag);
  }
  initialized = true;
}

void Simulation::renderStarfield(int width, int height, float time) {
  setupStarfieldGL(width, height, starPositions, NUM_STARS,
                   starfieldInitialized);
  glDisable(GL_DEPTH_TEST);
  glUseProgram(starShader);
  glUniform1f(glGetUniformLocation(starShader, "uTime"), time);
  glBindVertexArray(starVAO);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDrawArrays(GL_POINTS, 0, NUM_STARS);
  glDisable(GL_BLEND);
  glBindVertexArray(0);
  glUseProgram(0);
  glEnable(GL_DEPTH_TEST);
}

char Simulation::hudTextBuffer[9999];

void Simulation::renderHUD(int width, int height) const {
  // minimal legend HUD
  float x = 24.0f, y = height - 40.0f, dy = 36.0f;
  float scale = 2.2f;
  drawText2D("Legend:", x, y, 1, 1, 1, scale, width, height);
  drawText2D("Sun (large, orange)", x + 10, y - dy, 1.0f, 0.7f, 0.3f, scale,
             width, height);
  drawText2D("Planets (medium, blue)", x + 10, y - 2 * dy, 0.5f, 0.7f, 1.0f,
             scale, width, height);
  drawText2D("Asteroids (small, white)", x + 10, y - 3 * dy, 0.95f, 0.95f,
             0.95f, scale, width, height);

  // controls/keymap
  float y2 = y - 4.5f * dy;
  drawText2D("Controls:", x, y2, 1, 1, 1, scale, width, height);
  float d2 = 30.0f * scale;
  int line = 1;
  drawText2D("SPACE - Pause/Resume", x + 10, y2 - d2 * line, 1, 1, 1, scale,
             width, height);
  ++line;
  drawText2D("W/S   - Speed up/slow down time", x + 10, y2 - d2 * line, 1, 1, 1,
             scale, width, height);
  ++line;
  drawText2D("A/D   - Zoom in/out", x + 10, y2 - d2 * line, 1, 1, 1, scale,
             width, height);
  ++line;
  drawText2D("T     - Toggle trajectory", x + 10, y2 - d2 * line, 1, 1, 1,
             scale, width, height);
  ++line;
  drawText2D("B     - Use Barnes-Hut algorithm", x + 10, y2 - d2 * line, 1, 1,
             1, scale, width, height);
  ++line;
  drawText2D("M     - Use Fast Multipole Method (FMM)", x + 10, y2 - d2 * line,
             1, 1, 1, scale, width, height);
  ++line;
  drawText2D("N     - Use Direct N-body algorithm", x + 10, y2 - d2 * line, 1,
             1, 1, scale, width, height);
  ++line;
  drawText2D("I     - Print physical properties", x + 10, y2 - d2 * line, 1, 1,
             1, scale, width, height);
  ++line;
  drawText2D("F     - Toggle space-time fabric", x + 10, y2 - d2 * line, 1, 1,
             1, scale, width, height);
  ++line;
  drawText2D("G     - Toggle CPU/GPU (N-body only)", x + 10, y2 - d2 * line, 1,
             1, 1, scale, width, height);
  ++line;
  drawText2D("R     - Reset simulation", x + 10, y2 - d2 * line, 1, 1, 1, scale,
             width, height);
  ++line;
  drawText2D("Esc   - Exit", x + 10, y2 - d2 * line, 1, 1, 1, scale, width,
             height);

  // body info on hover
  if (hoveredBodyIndex >= 0 && hoveredBodyIndex < (int)bodies.size()) {
    const auto &body = bodies[hoveredBodyIndex];
    std::string type;
    if (hoveredBodyIndex == 0)
      type = "Sun";
    else if (hoveredBodyIndex <= 100)
      type = "Planet";
    else
      type = "Asteroid";
    char buf[256];
    float vy = y2 - d2 * (line + 2);
    snprintf(buf, sizeof(buf), "[Hover] %s | Mass: %.2f | Radius: %.2f",
             type.c_str(), body.mass, body.radius);
    drawText2D(buf, x, vy, 1, 1, 1, scale, width, height);
    snprintf(buf, sizeof(buf), "Pos: (%.2f, %.2f, %.2f)", body.position.x,
             body.position.y, body.position.z);
    drawText2D(buf, x, vy - d2, 1, 1, 1, scale, width, height);
    snprintf(buf, sizeof(buf), "Vel: (%.2f, %.2f, %.2f)", body.velocity.x,
             body.velocity.y, body.velocity.z);
    drawText2D(buf, x, vy - 2 * d2, 1, 1, 1, scale, width, height);
  }
}

void Simulation::printBodiesInfo() const {
  std::cout << "\nCelestial Bodies Info:\n";
  for (size_t i = 0; i < bodies.size(); ++i) {
    const CelestialBody &body = bodies[i];
    std::string type;
    if (i == 0)
      type = "Sun";
    else if (i <= 100)
      type = "Inner";
    else if (i <= 200)
      type = "Outer";
    else
      type = "Asteroid";
    std::cout << "[" << i << "] Type: " << type << ", Mass: " << body.mass
              << ", Radius: " << body.radius << ", Pos: (" << body.position.x
              << ", " << body.position.y << ", " << body.position.z << ")"
              << ", Vel: (" << body.velocity.x << ", " << body.velocity.y
              << ", " << body.velocity.z << ")"
              << ", Fixed: " << (body.isFixed ? "true" : "false")
              << ", Color: (" << body.color.r << ", " << body.color.g << ", "
              << body.color.b << ")" << std::endl;
  }
}

void Simulation::initComputeShaderAndBuffers() {
  computeShaderProgram = glCreateProgram();
  GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
  glShaderSource(shader, 1, &nbodyComputeShaderSource, nullptr);
  glCompileShader(shader);
  GLint success;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char infoLog[512];
    glGetShaderInfoLog(shader, 512, nullptr, infoLog);
    std::cerr << "Compute shader compilation failed: " << infoLog << std::endl;
    gpuSupported = false;
    return;
  }
  glAttachShader(computeShaderProgram, shader);
  glLinkProgram(computeShaderProgram);
  glGetProgramiv(computeShaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    char infoLog[512];
    glGetProgramInfoLog(computeShaderProgram, 512, nullptr, infoLog);
    std::cerr << "Compute shader linking failed: " << infoLog << std::endl;
    gpuSupported = false;
    return;
  }
  glDeleteShader(shader);

  // create SSBOs
  size_t bufSize = std::max<size_t>(1, bodies.size()) * sizeof(glm::vec4);
  glGenBuffers(1, &posSSBO);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, posSSBO);
  glBufferData(GL_SHADER_STORAGE_BUFFER, bufSize, nullptr, GL_DYNAMIC_DRAW);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, posSSBO);

  glGenBuffers(1, &velSSBO);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, velSSBO);
  glBufferData(GL_SHADER_STORAGE_BUFFER, bufSize, nullptr, GL_DYNAMIC_DRAW);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, velSSBO);

  glGenBuffers(1, &accSSBO);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, accSSBO);
  glBufferData(GL_SHADER_STORAGE_BUFFER, bufSize, nullptr, GL_DYNAMIC_DRAW);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, accSSBO);

  glGenBuffers(1, &massSSBO);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, massSSBO);
  glBufferData(GL_SHADER_STORAGE_BUFFER,
               std::max<size_t>(1, bodies.size()) * sizeof(float), nullptr,
               GL_DYNAMIC_DRAW);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, massSSBO);

  gpuSupported = true;
}

void Simulation::drawText2D(const char *text, float x, float y, float r,
                            float g, float b, float scale, int width,
                            int height) const {
  initTextGL();
  if (!textShader)
    return;
  int quads = stb_easy_font_print(x, y, (char *)text, NULL, hudTextBuffer,
                                  sizeof(hudTextBuffer));
  int numVerts = quads * 4;
  std::vector<GLuint> indices(quads * 6);
  for (int i = 0; i < quads; ++i) {
    int v = i * 4, idx = i * 6;
    indices[idx + 0] = v + 0;
    indices[idx + 1] = v + 1;
    indices[idx + 2] = v + 2;
    indices[idx + 3] = v + 0;
    indices[idx + 4] = v + 2;
    indices[idx + 5] = v + 3;
  }
  static GLuint textEBO = 0;
  if (!textEBO)
    glGenBuffers(1, &textEBO);
  glUseProgram(textShader);
  glUniform2f(glGetUniformLocation(textShader, "uScreen"), (float)width,
              (float)height);
  glUniform3f(glGetUniformLocation(textShader, "uColor"), r, g, b);
  glBindVertexArray(textVAO);
  glBindBuffer(GL_ARRAY_BUFFER, textVBO);
  glBufferData(GL_ARRAY_BUFFER, numVerts * 16, hudTextBuffer, GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, textEBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
               indices.data(), GL_DYNAMIC_DRAW);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glBindVertexArray(0);
  glUseProgram(0);
}
