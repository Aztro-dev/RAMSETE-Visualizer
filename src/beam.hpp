#pragma once
#include "path.hpp"
#include "raylib.h"

std::vector<BoundingBox> walls(4);

#define BEAM_WIDTH 2.0
class Beam {
  Ray ray;
  RayCollision collision;
  double relative_angle; // Store the beam's angle relative to robot

public:
  Beam() {}
  Beam(Pose position) {
    relative_angle = position.heading;

    Vector3 pos = Vector3{
        static_cast<float>(position.x),
        static_cast<float>(position.y),
        0.0};

    Vector3 direction = Vector3{
        std::cos(static_cast<float>(position.heading)),
        std::sin(static_cast<float>(position.heading)),
        0.0};

    float magnitude = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    direction.x /= magnitude;
    direction.y /= magnitude;
    direction.z /= magnitude;

    ray = Ray{pos, direction};

    if (walls.size() != 3) {
      walls.push_back(BoundingBox{
          Vector3{-72.0 * IN_TO_M, -72.0 * IN_TO_M, 0.0},
          Vector3{72.0 * IN_TO_M, -70.0 * IN_TO_M, 0.0}});
      walls.push_back(BoundingBox{
          Vector3{70.0 * IN_TO_M, -72.0 * IN_TO_M, 0.0},
          Vector3{72.0 * IN_TO_M, 72.0 * IN_TO_M, 0.0}});
      walls.push_back(BoundingBox{
          Vector3{-72.0 * IN_TO_M, 70.0 * IN_TO_M, 0.0},
          Vector3{72.0 * IN_TO_M, 72.0 * IN_TO_M, 0.0}});
      walls.push_back(BoundingBox{
          Vector3{-72.0 * IN_TO_M, -72.0 * IN_TO_M, 0.0},
          Vector3{-70.0 * IN_TO_M, 72.0 * IN_TO_M, 0.0}});
    }
  }

  double get_relative_angle() {
    return relative_angle;
  }

  void update_pose(Pose robot_pose) {
    ray.position.x = robot_pose.x;
    ray.position.y = robot_pose.y;

    double absolute_heading = robot_pose.heading + relative_angle;

    Vector3 direction = Vector3{
        std::cos(static_cast<float>(absolute_heading)),
        std::sin(static_cast<float>(absolute_heading)),
        0.0};

    float magnitude = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    direction.x /= magnitude;
    direction.y /= magnitude;
    direction.z /= magnitude;
    ray.direction = direction;
  }

  // Does the raycast to see how far the walls are
  void perform_hit() {
    for (size_t i = 0; i < walls.size(); i++) {
      collision = GetRayCollisionBox(ray, walls[i]);
      if (collision.hit) {
        break;
      }
    }
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
    Vector2 start = Vector2{ray.position.x * M_TO_PX, ray.position.y * M_TO_PX};
    Vector2 end = Vector2{collision.point.x * M_TO_PX, collision.point.y * M_TO_PX};

    start.y = -start.y;
    end.y = -end.y;

    start.x += 72.0 * IN_TO_PX;
    start.y += 72.0 * IN_TO_PX;
    end.x += 72.0 * IN_TO_PX;
    end.y += 72.0 * IN_TO_PX;
    DrawLineEx(start, end, BEAM_WIDTH, YELLOW);
  }

  void draw_walls() {
    for (size_t i = 0; i < walls.size(); i++) {
      Vector2 position = Vector2{walls[i].min.x * M_TO_PX, walls[i].min.y * M_TO_PX};
      Vector2 size = Vector2{walls[i].max.x * M_TO_PX, walls[i].max.y * M_TO_PX};
      size.x -= position.x;
      size.y -= position.y;
      position.x += 72.0 * IN_TO_PX;
      position.y += 72.0 * IN_TO_PX;
      DrawRectangleV(position, size, RED);
    }
  }
};
