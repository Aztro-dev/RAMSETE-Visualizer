#pragma once
#include "beam.hpp"
#include "path.hpp"
#include "raylib.h"

#define PARTICLE_RADIUS 1.0

#define GAUSSIAN_STDEV 0.1
#define GAUSSIAN_FACTOR 1.0
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
    // Calculate the beam direction from this particle's pose
    double absolute_beam_angle = pose.heading + relative_beam_angle;

    // Create a ray from this particle's position
    Vector3 particle_pos = {static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f};
    Vector3 beam_direction = {
        std::cos(static_cast<float>(absolute_beam_angle)),
        std::sin(static_cast<float>(absolute_beam_angle)),
        0.0f};

    Ray particle_ray = {particle_pos, beam_direction};

    // Cast the ray against walls to find distance
    double min_distance = INFINITY;
    for (size_t i = 0; i < walls.size(); i++) {
      RayCollision collision = GetRayCollisionBox(particle_ray, walls[i]);
      if (collision.hit && collision.distance < min_distance) {
        min_distance = collision.distance;
      }
    }

    return min_distance;
  }

  void update_weight(std::vector<Beam> &beams) {
    weight = 1.0; // Start with neutral weight

    for (size_t i = 0; i < beams.size(); i++) {
      // Get actual sensor reading
      double actual_distance = beams[i].get_hit_distance();

      // Simulate what this particle would "see"
      double expected_distance = simulate_distance_reading(beams[i].get_relative_angle());

      // Compare actual vs expected (smaller difference = higher weight)
      double error = std::abs(actual_distance - expected_distance);

      // Apply Gaussian weight based on error
      weight *= gaussian(error);
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

    // particles = std::vector<Particle>(PARTICLES);
    for (size_t i = 0; i < PARTICLES; i++) {
      // All particles at the start pose
      particles.push_back(Particle(start_pose));
    }

    // beams = std::vector<Beam>(RAYS);
    // Equally space the rays out in a circle
    double angle = 2 * PI / RAYS;
    for (size_t i = 0; i < RAYS; i++) {
      Pose beam_pose = start_pose;
      double beam_angle = angle * i;
      beam_pose.x += TRACK_WIDTH_M * std::cos(beam_angle);
      beam_pose.y += TRACK_HEIGHT_M * std::sin(beam_angle);
      beam_pose.heading += beam_angle;

      beams.push_back(Beam(beam_pose));
    }
  }

  Pose get_pose() {
    return pose;
  }

  void step() {
    position_mutex.lock();
    Pose current_robot_pose = robot_pose;
    position_mutex.unlock();

    // Calculate the change in robot pose since last update
    Pose delta;
    delta.x = current_robot_pose.x - prev_pose.x;
    delta.y = current_robot_pose.y - prev_pose.y;
    delta.heading = current_robot_pose.heading - prev_pose.heading;
    delta.v = current_robot_pose.v;

    // Wrap the heading difference
    delta.heading = std::atan2(std::sin(delta.heading), std::cos(delta.heading));

    for (size_t i = 0; i < beams.size(); i++) {
      beams[i].update_pose(robot_pose);
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
    for (size_t i = 0; i < particles.size(); i++) {
      particles[i].update_delta_noise(delta);
      particles[i].update_weight(beams);
      total_weight += particles[i].weight;
      weight_sums[i] = total_weight;
      offsets[i] = random_offset + static_cast<double>(i) / particles.size();
    }

    for (size_t i = 0; i < offsets.size(); i++) {
      offsets[i] *= total_weight;
    }

    // Select next gen of particles based on weight
    // More weighted particles have a higher chance to get into the next gen
    std::vector<Particle> new_particles;
    new_particles.reserve(PARTICLES);

    // Systematic resampling
    double cumulative_sum = 0.0;
    size_t particle_index = 0;

    for (size_t i = 0; i < PARTICLES; i++) {
      double target = (i + random_offset) * total_weight / PARTICLES;

      while (cumulative_sum < target && particle_index < particles.size()) {
        cumulative_sum += particles[particle_index].weight;
        particle_index++;
      }

      if (particle_index > 0) {
        new_particles.push_back(particles[particle_index - 1]);
      }
    }

    // Update pose now
    particles = new_particles;
    prev_pose = current_robot_pose;
    pose = Pose(0.0, 0.0, 0.0, 0.0);
    for (size_t i = 0; i < particles.size(); i++) {
      pose.x += particles[i].pose.x;
      pose.y += particles[i].pose.y;
      pose.heading += particles[i].pose.heading;
    }
    pose.x /= particles.size();
    pose.y /= particles.size();
    pose.heading /= particles.size();
  }

  void draw() {
    for (size_t i = 0; i < particles.size(); i++) {
      Vector2 pos = Vector2{particles[i].pose.x * M_TO_PX, particles[i].pose.y * M_TO_PX};
      pos.x += 72.0 * IN_TO_PX;
      pos.y = -pos.y;
      pos.y += 72.0 * IN_TO_PX;
      DrawCircleV(pos, PARTICLE_RADIUS, GREEN);
    }

    for (size_t i = 0; i < beams.size(); i++) {
      beams[i].draw_beam();
    }

    // beams[0].draw_walls();
  }
};

static Odometry odometry(trajectory[0].pose);

void odometry_loop() {
  while (true) {
    odometry.step();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}
