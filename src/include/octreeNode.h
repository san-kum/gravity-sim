#pragma once

#include "celestialBody.h"
#include <glm/glm.hpp>
#include <memory>

#define BARNES_HUT_THETA 0.5f
#define OCTREE_MAX_DEPTH 10
#define OCTREE_MIN_SIZE 0.1f

class OctreeNode {
public:
  glm::vec3 center;
  float size;

  float totalMass;
  glm::vec3 centerOfMass;

  std::unique_ptr<OctreeNode> children[8];
  CelestialBody *body;

  bool isLeaf;
  int depth;

  OctreeNode(const glm::vec3 &center, float size, int depth = 0);
  ~OctreeNode() = default;
  void insertBody(CelestialBody *celestialBody);
  void calculateForce(CelestialBody &target, float G,
                      float theta = BARNES_HUT_THETA) const;
  void updateMassProperties();

  void clear();
  int getOctant(const glm::vec3 &position) const;

  glm::vec3 getOctantCenter(int octant) const;
  bool contains(const glm::vec3 &position) const;

private:
  void subdivide();
  bool shouldUseApproximation(const glm::vec3 &targetPosition,
                              float theta) const;
};
