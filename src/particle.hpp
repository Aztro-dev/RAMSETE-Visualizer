#pragma once
#include "path.hpp"
#include "raylib.h"

#define GAUSSIAN_STDEV 0.100
#define GAUSSIAN_FACTOR 1.0
inline double gaussian(double input) {
  return GAUSSIAN_FACTOR * std::exp(-0.5 * std::pow(input / GAUSSIAN_STDEV, 2)) / (GAUSSIAN_STDEV * std::sqrt(2 * PI));
}

#define XY_NOISE 0.01    // in m
#define THETA_NOISE 0.03 // in rad
struct Particle {
  Pose pose;     // Position of the particle, but we don't need v
  double weight; // Weight of the particle

public:
  Particle() {
    pose = Pose();
    weight = 0.0;
  }

  Particle(Pose new_pose) {
    pose = new_pose;
    weight = 0.0;
  }

  void update_delta_noise(Pose delta) {
    pose.x += delta.x;
    pose.y += delta.y;
    pose.heading += delta.heading;

    pose.x += XY_NOISE * GetRandomValue(-1e9, 1e9) / 1e9;
    pose.y += XY_NOISE * GetRandomValue(-1e9, 1e9) / 1e9;
    pose.heading += THETA_NOISE * GetRandomValue(-1e9, 1e9) / 1e9;
  }

  double simulate_distance_reading(double relative_beam_angle) {
    double absolute_beam_angle = pose.heading + relative_beam_angle;
    Vector3 particle_pos = {static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f};
    Vector3 beam_direction = {
        std::cos(static_cast<float>(absolute_beam_angle)),
        std::sin(static_cast<float>(absolute_beam_angle)),
        0.0f};
    Ray particle_ray = {particle_pos, beam_direction};

    double min_distance = INFINITY;
    for (size_t i = 0; i < NUM_WALLS; i++) {
      RayCollision temp_collision = GetRayCollisionBox(particle_ray, walls[i]);
      // Make this match your Beam::perform_hit() logic exactly
      if (temp_collision.hit && temp_collision.distance <= MAX_BEAM_DISTANCE && temp_collision.distance < min_distance) {
        min_distance = temp_collision.distance;
      }
    }

    // If no collision found within range, return MAX_BEAM_DISTANCE (not INFINITY)
    return (min_distance == INFINITY) ? MAX_BEAM_DISTANCE : min_distance;
  }

  void update_weight(Beam beams[NUM_BEAMS]) {
    weight = 1.0; // neutral weight
    for (size_t i = 0; i < NUM_BEAMS; i++) {
      double actual_distance = beams[i].get_hit_distance();
      double expected_distance = simulate_distance_reading(beams[i].get_relative_angle());

      if (actual_distance == INFINITY && expected_distance >= MAX_BEAM_DISTANCE) {
        // Neither see an obstacle
        continue;
      } else if (actual_distance == INFINITY || expected_distance >= MAX_BEAM_DISTANCE) {
        // only 1 sees an obstacle
        double error = MAX_BEAM_DISTANCE * 0.5; // penalty of -50%
        weight *= gaussian(error);
      } else {
        // Both see obstacles
        double error = std::abs(actual_distance - expected_distance);
        weight *= gaussian(error);
      }
    }
  }

  Particle expected_point(Beam beam) {
    double global_theta = pose.heading + beam.get_hit_angle();
    return Pose{
        pose.x + beam.get_hit_distance() * std::cos(global_theta),
        pose.y + beam.get_hit_distance() * std::sin(global_theta),
        0, // No velocity
        global_theta};
  }

  // Semi good way to approximate distance of the point to the closest wall
  double distance_to_wall() {
    // Halfway across the field is 72 inches
    double min = std::min(std::abs(pose.x - 72 * IN_TO_M) / std::cos(pose.heading), std::abs(pose.x + 72 * IN_TO_M) / std::cos(pose.heading));
    min = std::min(min, std::abs(pose.y + 72 * IN_TO_M) / std::sin(pose.heading));
    min = std::min(min, std::abs(pose.y - 72 * IN_TO_M) / std::sin(pose.heading));
    return min;
  }
};
