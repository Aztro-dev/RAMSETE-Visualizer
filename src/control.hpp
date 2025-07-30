#pragma once
#include "motor.hpp"
#include "ramsete.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

double pid_turn(double error);

#define TIMESTEP 0.010 // corresponds to robot's 10ms update rate
#define STARTING_NODE 0

std::atomic<bool> should_end({false});

std::vector<int> reverse_indices = {3, 6, 12};
std::vector<int> rotating_indices = {5, 7, 9, 13};

std::mutex position_mutex;
Pose robot_pose;

std::mutex target_mutex;
TrajectoryPose target;

void control_robot(std::string path) {
  std::vector<TrajectoryPose> trajectory = loadJerryIOCSVPath("paths/qualifier-AWP.txt", reverse_indices);
  RamseteController ramsete(B, ZETA);

  if (trajectory.empty()) {
    std::cerr << "No path points loaded.\n";
    return;
  }

  size_t i = 0;

  bool reverse_switch = false;
  bool on_node = false;

  int current_node = STARTING_NODE;
  size_t current_node_index = -1;
  bool rotating_in_place = false;

  double time = 0.0;

  // Find the starting position based on current_node
  if (current_node != 0) {
    int node_count = 0;
    for (size_t j = 0; j < trajectory.size(); j++) {
      if (trajectory[j].is_node) {
        node_count++;
        if (node_count == current_node) {
          i = j;
          current_node_index = j;
          break;
        }
      }
    }
  } else {
    current_node_index = 0;
  }

  // If we couldn't find the requested node, fall back to start
  if (current_node_index == (size_t)-1) {
    std::cerr << "Warning: Could not find node " << current_node << ", starting from beginning.\n";
    current_node = 0;
    i = 0;
  }

  time = trajectory[i].time;

  position_mutex.lock();
  robot_pose = trajectory[i].pose;
  position_mutex.unlock();

  target_mutex.lock();
  target = trajectory[0];
  target_mutex.unlock();

  double now = GetTime();
  double frame_time = 0.0;
  double last_time = GetTime();
  double accumulator = 0.0;

  double prev_drive_left = 0.0;
  double prev_drive_right = 0.0;

  while (!should_end && time <= trajectory[trajectory.size() - 1].time) {
    if (IsKeyDown(KEY_SPACE)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    now = GetTime();
    frame_time = now - last_time;
    last_time = now;

    accumulator += frame_time;

    while (accumulator >= TIMESTEP) {
      time += TIMESTEP;
      accumulator -= TIMESTEP;
    }

    while (i + 1 < trajectory.size() && trajectory[i + 1].time <= time) {
      i++;
    }

    target_mutex.lock();
    target = trajectory[i];
    target_mutex.unlock();

    Pose error = target.pose - robot_pose;

    error.heading = std::atan2(std::sin(target.pose.heading - robot_pose.heading),
                               std::cos(target.pose.heading - robot_pose.heading));

    // Find out if we are at the next node
    if (target.is_node && current_node_index != i) {
      current_node_index = i;
      current_node++;
    }
    // If we are at the next node, check to see if we should turn in place or not
    rotating_in_place = false;
    //       // std::vector<int> rotating_indices = {3, 5, 7, 12, 14, 16, 18, 20, 22, 26, 28, 29};
    for (size_t j = 0; j < rotating_indices.size(); j++) {
      if (current_node == rotating_indices[j]) {
        rotating_in_place = true;
        break;
      }
    }

    for (size_t j = 0; j < reverse_indices.size(); j++) {
      if (current_node == reverse_indices[j]) {
        reverse_switch = true;
      } else if (current_node == reverse_indices[j] + 1) {
        reverse_switch = false;
      }
    }

    on_node = current_node_index == i;

    switch (current_node) {
    case 4: {
      if (on_node) {
        pid_turn(error.heading);
      }
    }
    }

    // Calculate desired angular velocity for RAMSETE feedforward
    double w_desired = 0.0;
    if (i + 1 < trajectory.size() && !rotating_in_place && !reverse_switch) {
      // Estimate curvature from trajectory
      double dt_traj = trajectory[i + 1].time - trajectory[i].time;
      if (dt_traj > 1e-6) {
        double dheading = trajectory[i + 1].pose.heading - trajectory[i].pose.heading;
        // Wrap angle difference
        dheading = std::atan2(std::sin(dheading), std::cos(dheading));
        w_desired = dheading / dt_traj;

        // For reverse motion, negate the feedforward
        if (reverse_switch) {
          w_desired = -w_desired;
        }
      }
    }

    auto [drive_left, drive_right] = ramsete.calculate(robot_pose, target.pose, w_desired);

    if (reverse_switch) {
      drive_left = -drive_left;
      drive_right = -drive_right;
    }

#ifdef MOTOR_SIM
    double desired_change_left = drive_left - prev_drive_left;
    double desired_change_right = drive_right - prev_drive_right;

    double max_change_left = max_rpm_change(prev_drive_left, TIMESTEP);
    double max_change_right = max_rpm_change(prev_drive_right, TIMESTEP);

    drive_left = prev_drive_left + std::clamp(desired_change_left, -max_change_left, max_change_left);
    drive_right = prev_drive_right + std::clamp(desired_change_right, -max_change_right, max_change_right);
#else
    drive_left = std::clamp(drive_left, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
    drive_right = std::clamp(drive_right, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
#endif

    double left_velocity = (drive_left / 60.0) * 2.0 * M_PI * WHEEL_RADIUS_M;
    double right_velocity = (drive_right / 60.0) * 2.0 * M_PI * WHEEL_RADIUS_M;

    double v_wheels = (left_velocity + right_velocity) / 2.0;
    double w_wheels = (right_velocity - left_velocity) / TRACK_WIDTH_M;

#ifdef MOTOR_SIM
    prev_drive_left = drive_left;
    prev_drive_right = drive_right;
#endif

    position_mutex.lock();
    printf("update robot_pose\n");
    robot_pose.x += v_wheels * std::cos(robot_pose.heading) * TIMESTEP;
    robot_pose.y += v_wheels * std::sin(robot_pose.heading) * TIMESTEP;
    robot_pose.heading += w_wheels * TIMESTEP;
    robot_pose.v = v_wheels;
    position_mutex.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(TIMESTEP * 1000.0)));
  }
}

#define TURN_KP 200.0 // Increased gain for faster turning
double pid_turn(double error) {
  // Simple proportional control: output RPM proportional to heading error
  double rpm = error * TURN_KP;
  rpm = -rpm;
  // Clamp to reasonable limits
  return std::clamp(rpm, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
}
