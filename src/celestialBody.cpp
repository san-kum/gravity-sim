#include "include/celestialBody.h"
#include <glm/gtc/matrix_transform.hpp>

CelestialBody::CelestialBody(glm::vec3 pos, glm::vec3 vel, float m, float r,
                             glm::vec3 col, bool fixed)
    : position(pos), velocity(vel), mass(m), radius(r), color(col),
      isFixed(fixed) {
  acceleration = glm::vec3(0.0f);
}

void CelestialBody::applyGravity(const CelestialBody &other, float G) {
  if (&other == this)
    return;

  glm::vec3 direction = other.position - position;
  float distance = glm::length(direction);

  if (distance < 0.1f)
    distance = 0.1f;

  direction = glm::normalize(direction);

  // gravitational force : F = G * m1 * m2 / r^2
  float forceMagnitude = G * mass * other.mass / (distance * distance);

  // F = ma
  acceleration += direction * (forceMagnitude / mass);
}

void CelestialBody::update(float deltaTime) {
  if (isFixed) {
    acceleration = glm::vec3(0.0f); // Fixed bodies don't move
    return;
  }

  /**
   *  Verlet integration
   *  v(t+dt) = v(t) + a(t) * dt
   *  x(t+dt) = x(t) + v(t) * dt + 0.5 * a(t) * dt^2
   * */

  velocity += acceleration * deltaTime;
  position +=
      velocity * deltaTime + 0.5f * acceleration * deltaTime * deltaTime;

  acceleration = glm::vec3(0.0f);
}

void CelestialBody::addTrajectoryPoint() {
  trajectory.push_back(position);

  if (trajectory.size() > MAX_TRAJECTORY_POINTS)
    trajectory.pop_front();
}

void CelestialBody::clearTrajectory(){
	trajectory.clear();
	trajectory.push_back(position);
}


