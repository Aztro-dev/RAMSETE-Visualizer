#pragma once
#include "beam.hpp"
#include "path.hpp"
#include "raylib.h"

#define GAUSSIAN_STDEV 1
#define GAUSSIAN_FACTOR 1
inline double gaussian(double input) {
  return GAUSSIAN_FACTOR * std::exp(-0.5 * std::pow(input / GAUSSIAN_STDEV, 2)) / (GAUSSIAN_STDEV * std::sqrt(2 * PI));
}

#define RAYS 4        // 4 distance sensors
#define PARTICLES 500 // 500 particles means quite good Localization, but high performance hit

#define XY_NOISE 0.025   // in m
#define THETA_NOISE 0.03 // in rad
struct Particle {
  Pose pose;     // Position of the particle, but we don't need v
  double weight; // Weight of the particle

public:
  Particle(Pose new_pose) {
    pose = new_pose;
    weight = 0.0;
  }

  void update_delta_noise(Pose delta) {
    pose.x += XY_NOISE * GetRandomValue(-1e9, 1e9) / 1e9;
    pose.y += XY_NOISE * GetRandomValue(-1e9, 1e9) / 1e9;
    pose.heading += THETA_NOISE * GetRandomValue(-1e9, 1e9) / 1e9;
  }

  void update_weight(std::vector<Beam> beams) {
    weight = 0.0;
    for (int i = 0; i < beams.size(); i++) {
      weight += gaussian(expected_point(beams[i]).distance_to_wall());
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

// Uses Monte-Carlo Localization to approximate the position of the robot
class Odometry {
  std::vector<Particle> particles;
  std::vector<Beam> beams;
  Pose pose;
  Pose prev_pose;

public:
  Odometry(Pose start_pose) {
    pose = start_pose;
    prev_pose = pose;

    particles = std::vector<Particle>(PARTICLES);
    for (int i = 0; i < PARTICLES; i++) {
      // All particles at the start pose
      particles.push_back(Particle(start_pose));
    }

    beams = std::vector<Beam>(RAYS);
    // Equally space the rays out in a circle
    double angle = 2 * PI / RAYS;
    for (int i = 0; i < RAYS; i++) {
      Pose beam_pose = start_pose;
      double beam_angle = angle * i;
      beam_pose.x += TRACK_WIDTH_M * std::cos(beam_angle);
      beam_pose.y += TRACK_HEIGHT_M * std::sin(beam_angle);
      beam_pose.heading += beam_angle;

      beams.push_back(Beam(beam_pose));
    }
  }

  void step() {
    Pose delta = pose - prev_pose;

    for (int i = 0; i < beams.size(); i++) {
      beams[i].perform_hit();
    }

    // Used in offset calculation
    double total_weight = 0.0;
    // 1.0 * is to cast to double without doing static_cast
    double random_offset = 1.0 * GetRandomValue(-1e9, 1e9) / 1e9;
    // Used to add a threshold to acceptable values
    std::vector<double> weight_sums(particles.size());
    // Used to add a threshold to acceptable values
    std::vector<double> offsets(particles.size());
    // Update particles
    for (int i = 0; i < particles.size(); i++) {
      particles[i].update_delta_noise(delta);
      particles[i].update_weight(beams);
      total_weight += particles[i].weight;
      weight_sums[i] = total_weight;
      offsets[i] = random_offset + static_cast<double>(i) / particles.size();
    }

    for (int i = 0; i < offsets.size(); i++) {
      offsets[i] *= total_weight;
    }

    // Select next gen of particles based on weight
    // More weighted particles have a higher chance to get into the next gen
    std::vector<Particle> new_particles(PARTICLES);
    for (int i = 0; i < offsets.size(); i++) {
      for (int ii = 0; ii < particles.size(); i++) {
        if (weight_sums[i] >= offsets[i]) {
          new_particles.push_back(particles[i]);
          break; // Go to next offset
        }
      }
    }

    // Update pose now
    particles = new_particles;
    prev_pose = pose;
    pose = Pose(0.0, 0.0, 0.0, 0.0);
    for (int i = 0; i < particles.size(); i++) {
      pose.x += particles[i].pose.x;
      pose.y += particles[i].pose.y;
      pose.heading += particles[i].pose.heading;
    }
    pose.x /= particles.size();
    pose.y /= particles.size();
    pose.heading /= particles.size();
  }
};
