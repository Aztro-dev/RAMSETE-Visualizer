#pragma once
#include "beam.hpp"
#include "particle.hpp"
#include "path.hpp"
#include "raylib.h"

#define PARTICLE_RADIUS 1.0

// Uses Monte-Carlo Localization to approximate the position of the robot
// MCL is based off of https://aadishv.github.io/mcl/
class Odometry {
  Particle particles[NUM_PARTICLES];
  Beam beams[NUM_BEAMS];
  Pose pose;
  Pose prev_pose;

public:
  Odometry(Pose start_pose) {
    pose = start_pose;
    prev_pose = pose;

    for (size_t i = 0; i < NUM_PARTICLES; i++) {
      // All particles at the start pose
      particles[i] = Particle(start_pose);
    }

    // Equally space the rays out in a circle
    double angle = 2 * PI / NUM_BEAMS;
    for (size_t i = 0; i < NUM_BEAMS; i++) {
      Pose beam_pose;
      double beam_angle = angle * i;
      beam_pose.x = (TRACK_WIDTH_M / 2.0) * std::cos(beam_angle);
      beam_pose.y = (TRACK_HEIGHT_M / 2.0) * std::sin(beam_angle);
      beam_pose.heading = (i % NUM_BEAMS) * angle;

      beams[i] = Beam(beam_pose, start_pose);
    }
  }

  Pose get_pose() {
    return pose;
  }

  void step() {
    position_mutex.lock();
    Pose commanded_pose = trajectory[i].pose;
    position_mutex.unlock();

    Pose delta;
    delta.x = commanded_pose.x - prev_pose.x;
    delta.y = commanded_pose.y - prev_pose.y;
    delta.heading = commanded_pose.heading - prev_pose.heading;
    delta.heading = std::atan2(std::sin(delta.heading), std::cos(delta.heading));

    for (size_t i = 0; i < NUM_BEAMS; i++) {
      beams[i].update_pose(commanded_pose);
      beams[i].perform_hit();
    }

    for (size_t i = 0; i < NUM_PARTICLES; i++) {
      particles[i].update_delta_noise(delta);
      particles[i].update_weight(beams);
    }

    double offsets[NUM_PARTICLES];
    double random_offset = 1.0 * GetRandomValue(-1e9, 1e9) / 1e9 / NUM_PARTICLES;
    for (size_t i = 0; i < NUM_PARTICLES; i++) {
      offsets[i] = random_offset + static_cast<double>(i) / NUM_PARTICLES;
    }

    double weight_sums[NUM_PARTICLES];
    weight_sums[0] = particles[0].weight;
    for (size_t i = 1; i < NUM_PARTICLES; i++) {
      weight_sums[i] = particles[i].weight + weight_sums[i - 1];
    }

    double total_weight = weight_sums[NUM_PARTICLES - 1];
    for (size_t i = 0; i < NUM_PARTICLES; i++) {
      offsets[i] *= total_weight;
    }

    Particle new_particles[NUM_PARTICLES];
    for (size_t i = 0; i < NUM_PARTICLES; i++) {
      for (size_t ii = 0; ii < NUM_PARTICLES; ii++) {
        if (weight_sums[ii] >= offsets[i]) {
          new_particles[i] = particles[ii];
          break;
        }
      }
    }

    pose.x = 0.0;
    pose.y = 0.0;
    pose.heading = 0.0;

    // circular averaging:
    double sin_sum = 0.0, cos_sum = 0.0;
    for (size_t i = 0; i < NUM_PARTICLES; i++) {
      particles[i] = new_particles[i];
      pose.x += particles[i].pose.x;
      pose.y += particles[i].pose.y;
      sin_sum += std::sin(particles[i].pose.heading);
      cos_sum += std::cos(particles[i].pose.heading);
    }
    pose.x /= NUM_PARTICLES;
    pose.y /= NUM_PARTICLES;
    pose.heading = std::atan2(sin_sum / NUM_PARTICLES, cos_sum / NUM_PARTICLES);

    prev_pose = commanded_pose;
  }

  void draw() {
    for (size_t i = 0; i < NUM_PARTICLES; i++) {
      Vector2 pos = Vector2{particles[i].pose.x * M_TO_PX, particles[i].pose.y * M_TO_PX};
      pos.x += 72.0 * IN_TO_PX;
      pos.y = -pos.y;
      pos.y += 72.0 * IN_TO_PX;
      DrawCircleV(pos, PARTICLE_RADIUS, GREEN);
    }

    for (size_t i = 0; i < NUM_BEAMS; i++) {
      beams[i].draw_beam();
    }

#if DEBUG_MODE
    beams[0].draw_walls();
#endif
  }
};

static Odometry odometry(trajectory[0].pose);

void odometry_loop() {
  while (true) {
    odometry.step();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}
