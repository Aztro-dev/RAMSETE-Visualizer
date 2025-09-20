#pragma once
#include "path.hpp"
#include "raylib.h"

#define RAY_WIDTH 4.0
class Beam {
  Ray ray;
  RayCollision collision;

public:
  Beam(Pose position) {
    Vector3 pos = Vector3{static_cast<float>(position.x), static_cast<float>(position.y), 0.0};
    Vector3 direction = Vector3{
        std::cos(static_cast<float>(position.heading)),
        std::sin(static_cast<float>(position.heading)),
        0.0};

    float magnitude = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    direction.x /= magnitude;
    direction.y /= magnitude;
    direction.z /= magnitude;

    ray = Ray{pos, direction};
  }

  // Does the raycast to see how far the walls are
  void perform_hit() {
    // TODO: Finish this function
    // ...
  }

  double get_hit_angle() {
    if (!collision.hit) {
      return 0.0;
    }

    Vector3 direction = ray.direction;
    Vector3 normal = collision.normal;

    // A dot B = AB cos(theta)
    double dot_product = direction.x * normal.x + direction.y * normal.y + direction.z * normal.z;
    // Clamp to -1.0, 1.0 because arccos is silly
    dot_product = std::max(-1.0, std::min(1.0, dot_product));
    // theta = arccos(A dot B / AB)
    // AB is 1 because A and B are normalized already
    double angle = std::acos(std::abs(dot_product));
    return angle;
  }

  double get_hit_distance() {
    return collision.distance;
  }

  void draw_beam() {
    Vector2 start = Vector2{ray.position.x, ray.position.y};
    Vector2 end = Vector2{collision.point.x, collision.point.y};
    DrawLineEx(start, end, RAY_WIDTH, YELLOW);
  }
};
