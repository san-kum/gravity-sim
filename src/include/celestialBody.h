#pragma once

#include <deque>
#include <glm/glm.hpp>

class CelestialBody {
public:
  glm::vec3 position;
  glm::vec3 velocity;
  glm::vec3 acceleration;
  glm::vec3 color;
  float mass;
  float radius;
  bool isFixed;

  // Trajectory
  std::deque<glm::vec3> trajectory;
  static const size_t MAX_TRAJECTORY_POINTS = 500;

  CelestialBody(glm::vec3 pos, glm::vec3 vel, float m, float r, glm::vec3 col,
                bool fixed = false);

  void applyGravity(const CelestialBody &other, float G);
  void update(float deltaTime);
	void addTrajectoryPoint();
	void clearTrajectory();
};
