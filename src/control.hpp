#pragma once
#include "motor.hpp"
#include "ramsete.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

void pid_turn();

#define TIMESTEP 0.010 // corresponds to robot's 10ms update rate
#define STARTING_NODE 1

std::atomic<bool> should_end({false});
std::atomic<bool> path_done({false});

std::vector<int> reverse_indices = {3, 6, 11};

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

  bool on_node = false;

  int current_node = STARTING_NODE;
  size_t current_node_index = -1;

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

#ifdef MOTOR_SIM
  double prev_drive_left = 0.0;
  double prev_drive_right = 0.0;
#endif

  while (!should_end && time <= trajectory[trajectory.size() - 1].time) {
    if (IsKeyDown(KEY_SPACE)) {
      last_time = GetTime();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    now = GetTime();
    frame_time = now - last_time;

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

    // Find out if we are at the next node
    if (target.is_node && current_node_index != i) {
      current_node_index = i;
      current_node++;

      on_node = true;

      printf("On node %d\n", current_node);
    } else {
      on_node = false;
    }

    switch (current_node) {
    case 3: {
      if (on_node) {
        printf("outtaking...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }
    case 4: {
      if (on_node) {
        pid_turn();
      }
    }
    case 5: {
      if (on_node) {
        pid_turn();
      }
    }

    case 7: {
      if (on_node) {
        pid_turn();
      }
    }

    case 9: {
      if (on_node) {
        pid_turn();
      }
    }
    case 12: {
      if (on_node) {
        pid_turn();
      }
    }
    }

    // Because movements/delays might take time in the previous step, we have to reset the previous time
    last_time = GetTime();

    // Calculate desired angular velocity for RAMSETE feedforward
    double w_desired = 0.0;
    if (i + 1 < trajectory.size()) {
      // Estimate curvature from trajectory
      double dt_traj = trajectory[i + 1].time - trajectory[i].time;
      if (dt_traj > 1e-6) {
        double dheading = trajectory[i + 1].pose.heading - trajectory[i].pose.heading;
        // Wrap angle difference
        dheading = std::atan2(std::sin(dheading), std::cos(dheading));
        w_desired = dheading / dt_traj;

        // For reverse motion, negate the feedforward
      }
    }

    auto [drive_left, drive_right] = ramsete.calculate(robot_pose, target.pose, w_desired);

#ifdef MOTOR_SIM
    double desired_change_left = drive_left - prev_drive_left;
    double desired_change_right = drive_right - prev_drive_right;

    double max_change_left = max_rpm_change(prev_drive_left, TIMESTEP);
    double max_change_right = max_rpm_change(prev_drive_right, TIMESTEP);

    drive_left = prev_drive_left + std::clamp(desired_change_left, -max_change_left, max_change_left);
    drive_right = prev_drive_right + std::clamp(desired_change_right, -max_change_right, max_change_right);

    prev_drive_left = drive_left;
    prev_drive_right = drive_right;
#else
    drive_left = std::clamp(drive_left, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
    drive_right = std::clamp(drive_right, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
#endif

    double left_velocity = (drive_left / 60.0) * 2.0 * M_PI * WHEEL_RADIUS_M;
    double right_velocity = (drive_right / 60.0) * 2.0 * M_PI * WHEEL_RADIUS_M;

    double v_wheels = (left_velocity + right_velocity) / 2.0;

    double velocity_difference = right_velocity - left_velocity;
    double w_wheels = velocity_difference / TRACK_WIDTH_M;

    position_mutex.lock();
    robot_pose.x += v_wheels * std::cos(robot_pose.heading) * TIMESTEP;
    robot_pose.y += v_wheels * std::sin(robot_pose.heading) * TIMESTEP;
    robot_pose.heading += w_wheels * TIMESTEP;
    robot_pose.v = v_wheels;
    position_mutex.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(TIMESTEP * 1000.0)));
  }

  path_done = true;
}

#define TURN_KP 150.0 // Increased gain for faster turning
void pid_turn() {
#ifdef MOTOR_SIM
  double prev_drive_left = 0.0;
  double prev_drive_right = 0.0;
#endif

  double error = std::atan2(std::sin(target.pose.heading - robot_pose.heading),
                            std::cos(target.pose.heading - robot_pose.heading));

  while (std::fabs(error) > 1.0 * DEG_TO_RAD) {
    if (should_end) {
      break;
    }

    error = std::atan2(std::sin(target.pose.heading - robot_pose.heading),
                       std::cos(target.pose.heading - robot_pose.heading));

    double turn_speed = error * TURN_KP;

    turn_speed = std::clamp(turn_speed, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);

    double drive_left = -turn_speed;
    double drive_right = turn_speed;

#ifdef MOTOR_SIM
    double desired_change_left = drive_left - prev_drive_left;
    double desired_change_right = drive_right - prev_drive_right;

    double max_change_left = max_rpm_change(prev_drive_left, TIMESTEP);
    double max_change_right = max_rpm_change(prev_drive_right, TIMESTEP);

    drive_left = prev_drive_left + std::clamp(desired_change_left, -max_change_left, max_change_left);
    drive_right = prev_drive_right + std::clamp(desired_change_right, -max_change_right, max_change_right);

    prev_drive_left = drive_left;
    prev_drive_right = drive_right;
#else
    drive_left = std::clamp(drive_left, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
    drive_right = std::clamp(drive_right, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
#endif

    double left_velocity = drive_left * 2.0 * M_PI * WHEEL_RADIUS_M / 60.0;
    double right_velocity = drive_right * 2.0 * M_PI * WHEEL_RADIUS_M / 60.0;

    double w_wheels = (right_velocity - left_velocity) / TRACK_WIDTH_M;

    position_mutex.lock();
    robot_pose.heading += w_wheels * TIMESTEP;
    position_mutex.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(TIMESTEP * 1000.0)));
  }

  position_mutex.lock();
  robot_pose.heading = target.pose.heading;
  position_mutex.unlock();
}
