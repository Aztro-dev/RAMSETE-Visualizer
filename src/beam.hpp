#pragma once
#include "path.hpp"
#include "raylib.h"

BoundingBox walls[NUM_WALLS] = {
    // Field walls
    BoundingBox{Vector3{-72.0 * IN_TO_M, -72.0 * IN_TO_M, 0.0}, Vector3{72.0 * IN_TO_M, -70.0 * IN_TO_M, 0.0}},
    BoundingBox{Vector3{70.0 * IN_TO_M, -72.0 * IN_TO_M, 0.0}, Vector3{72.0 * IN_TO_M, 72.0 * IN_TO_M, 0.0}},
    BoundingBox{Vector3{-72.0 * IN_TO_M, 70.0 * IN_TO_M, 0.0}, Vector3{72.0 * IN_TO_M, 72.0 * IN_TO_M, 0.0}},
    BoundingBox{Vector3{-72.0 * IN_TO_M, -72.0 * IN_TO_M, 0.0}, Vector3{-70.0 * IN_TO_M, 72.0 * IN_TO_M, 0.0}},
    // Center goals
    BoundingBox{Vector3{-5.0 * IN_TO_M, -5.0 * IN_TO_M, 0.0}, Vector3{5.0 * IN_TO_M, 5.0 * IN_TO_M, 0.0}},
    // Long goals
    BoundingBox{Vector3{-24.0 * IN_TO_M, -50.0 * IN_TO_M, 0.0}, Vector3{-22.0 * IN_TO_M, -44.0 * IN_TO_M, 0.0}},
    BoundingBox{Vector3{-24.0 * IN_TO_M, 44.0 * IN_TO_M, 0.0}, Vector3{-22.0 * IN_TO_M, 50.0 * IN_TO_M, 0.0}},
    BoundingBox{Vector3{22.0 * IN_TO_M, -50.0 * IN_TO_M, 0.0}, Vector3{24.0 * IN_TO_M, -44.0 * IN_TO_M, 0.0}},
    BoundingBox{Vector3{22.0 * IN_TO_M, 44.0 * IN_TO_M, 0.0}, Vector3{24.0 * IN_TO_M, 50.0 * IN_TO_M, 0.0}}
    //
};

#define BEAM_WIDTH 2.0
class Beam {
  Ray ray;
  RayCollision collision;
  Pose local_pose;

public:
  Beam() {}
  // position is the pose of the LiDAR relative to teh robot
  // start_pose is the pose of the robot to begin with
  Beam(Pose position, Pose start_pose) {
    local_pose = position; // Position relative to the robot

    Pose rotated;
    rotated.x = local_pose.x * std::cos(start_pose.heading) - local_pose.y * std::sin(start_pose.heading);
    rotated.y = local_pose.x * std::sin(start_pose.heading) + local_pose.y * std::cos(start_pose.heading);

    Vector3 pos = Vector3{
        static_cast<float>(start_pose.x + rotated.x),
        static_cast<float>(start_pose.y + rotated.y),
        0.0};

    double absolute_heading = start_pose.heading + local_pose.heading;

    Vector3 direction = Vector3{
        std::cos(static_cast<float>(absolute_heading)),
        std::sin(static_cast<float>(absolute_heading)),
        0.0};

    float magnitude = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    direction.x /= magnitude;
    direction.y /= magnitude;
    direction.z /= magnitude;

    ray = Ray{pos, direction};
  }

  Pose get_local_pose() {
    return local_pose;
  }

  void update_pose(Pose robot_pose) {
    Pose rotated;
    rotated.x = local_pose.x * std::cos(robot_pose.heading) - local_pose.y * std::sin(robot_pose.heading);
    rotated.y = local_pose.x * std::sin(robot_pose.heading) + local_pose.y * std::cos(robot_pose.heading);
    ray.position.x = robot_pose.x + rotated.x;
    ray.position.y = robot_pose.y + rotated.y;

    double absolute_heading = robot_pose.heading + local_pose.heading;

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
    double min_distance = INFINITY;
    RayCollision min_collision = {0};
    bool found_hit = false;
    for (size_t i = 0; i < NUM_WALLS; i++) {
      collision = GetRayCollisionBox(ray, walls[i]);
      if (collision.hit && collision.distance <= MAX_BEAM_DISTANCE && collision.distance < min_distance) {
        min_distance = collision.distance;
        min_collision = collision;
        found_hit = true;
      }
    }

    if (found_hit) {
      collision = min_collision;
      collision.hit = true;
    } else {
      collision.distance = INFINITY;
      collision.hit = false;
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
    if (!collision.hit) {
      return;
    }

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
    for (size_t i = 0; i < NUM_WALLS; i++) {
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
