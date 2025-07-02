#include "include/octreeNode.h"
#include "include/celestialBody.h"
#include <glm/geometric.hpp>
#include <memory>

OctreeNode::OctreeNode(const glm::vec3 &center, float size, int depth)
    : center(center), size(size), totalMass(0.0f), centerOfMass(0.0f),
      body(nullptr), isLeaf(true), depth(depth) {
  for (int i = 0; i < 8; i++)
    children[i] = nullptr;
}

void OctreeNode::insertBody(CelestialBody *celestialBody) {
  if (!contains(celestialBody->position))
    return;

  if (totalMass == 0.0f) {
    body = celestialBody;
    totalMass = celestialBody->mass;
    centerOfMass = celestialBody->position;
    isLeaf = true;
    return;
  }
  if (isLeaf && body != nullptr) {
    if (depth >= OCTREE_MAX_DEPTH || size < OCTREE_MIN_SIZE) {
      float newTotalMass = totalMass + celestialBody->mass;
      centerOfMass = (centerOfMass * totalMass +
                      celestialBody->position * celestialBody->mass) /
                     newTotalMass;
      totalMass = newTotalMass;
      return;
    }

    CelestialBody *existingBody = body;
    body = nullptr;
    isLeaf = false;
    subdivide();

    int existingOctant = getOctant(existingBody->position);
    children[existingOctant]->insertBody(existingBody);

    int newOctant = getOctant(celestialBody->position);
    children[newOctant]->insertBody(celestialBody);
  } else if (!isLeaf) {
    int octant = getOctant(celestialBody->position);
    children[octant]->insertBody(celestialBody);
  }
  updateMassProperties();
}

void OctreeNode::calculateForce(CelestialBody &target, float G,
                                float theta) const {
  if (totalMass == 0.0f)
    return;

  if (isLeaf && body != nullptr) {
    if (body == &target)
      return;
    target.applyGravity(*body, G);
    return;
  }
  if (!isLeaf) {
    if (shouldUseApproximation(target.position, theta)) {
      glm::vec3 direction = centerOfMass - target.position;
      float distance = glm::length(direction);

      if (distance < 0.1f)
        distance = 0.1f;

      direction = glm::normalize(direction);
      float forceMagnitude =
          G * target.mass * totalMass / (distance * distance);
      target.acceleration += direction * (forceMagnitude / target.mass);
    } else {
      for (int i = 0; i < 8; i++) {
        if (children[i] != nullptr)
          children[i]->calculateForce(target, G, theta);
      }
    }
  }
}

void OctreeNode::updateMassProperties() {
  if (isLeaf && body != nullptr) {
    totalMass = body->mass;
    centerOfMass = body->position;
  } else if (!isLeaf) {
    totalMass = 0.0f;
    glm::vec3 weightedPosition(0.0f);

    for (int i = 0; i < 8; i++) {
      if (children[i] != nullptr && children[i]->totalMass > 0.0f) {
        totalMass += children[i]->totalMass;
        weightedPosition += children[i]->centerOfMass * children[i]->totalMass;
      }
    }
    if (totalMass > 0.0f)
      centerOfMass = weightedPosition / totalMass;
    else
      centerOfMass = center;
  }
}

void OctreeNode::clear() {
  totalMass = 0.0f;
  centerOfMass = glm::vec3(0.0f);
  body = nullptr;
  isLeaf = true;

  for (int i = 0; i < 8; i++)
    children[i] = nullptr;
}

int OctreeNode::getOctant(const glm::vec3 &position) const {
  int octant = 0;
  if (position.x >= center.x)
    octant |= 1;
  if (position.y >= center.y)
    octant |= 2;
  if (position.z >= center.z)
    octant |= 4;

  return octant;
}

glm::vec3 OctreeNode::getOctantCenter(int octant) const {
  float halfSize = size * 0.5f;
  float quarterSize = size * 0.25f;

  glm::vec3 octantCenter = center;

  octantCenter.x += (octant & 1) ? quarterSize : -quarterSize;
  octantCenter.y += (octant & 2) ? quarterSize : -quarterSize;
  octantCenter.z += (octant & 4) ? quarterSize : -quarterSize;

  return octantCenter;
}

bool OctreeNode::contains(const glm::vec3 &position) const {
  float halfSize = size * 0.5f;
  return (
      position.x >= center.x - halfSize && position.x < center.x + halfSize &&
      position.y >= center.y - halfSize && position.y < center.y + halfSize &&
      position.z >= center.z - halfSize && position.z < center.z + halfSize);
}

void OctreeNode::subdivide() {
  float childSize = size * 0.5f;

  for (int i = 0; i < 8; i++) {
    glm::vec3 childCenter = getOctantCenter(i);
    children[i] =
        std::make_unique<OctreeNode>(childCenter, childSize, depth + 1);
  }
}

bool OctreeNode::shouldUseApproximation(const glm::vec3& targetPosition, float theta) const {
	float distance = glm::length(centerOfMass - targetPosition);

	if(distance < 0.1f)
		return false;

	return (size / distance) < theta;
}




