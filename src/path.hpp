#pragma once
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// pose structure for the robot
struct Pose {
  double x;       // target x position in meters
  double y;       // target y position in meters
  double v;       // target linear velocity in m/s
  double heading; // target heading in radians

  Pose() {
    x = 0.0;
    y = 0.0;
    v = 0.0;
    heading = 0.0;
  }

  Pose(double x, double y, double v, double heading)
      : x(x), y(y), v(v), heading(heading) {}

  Pose operator+(const Pose &a) const {
    return Pose(x + a.x, y + a.y, v + a.v, heading + a.heading);
  }

  Pose operator-(const Pose &a) const {
    return Pose(x - a.x, y - a.y, v - a.v, heading - a.heading);
  }
};

// pose for trajectory
struct TrajectoryPose {
  Pose pose;
  double time;
  bool is_node;
};

std::vector<TrajectoryPose> loadJerryIOCSVPath(const std::string &pathFile, std::vector<int> reverse_list) {
  std::ifstream file(pathFile);
  std::string line;
  std::vector<TrajectoryPose> raw_path;
  bool inPathSection = false;

  while (std::getline(file, line)) {
    if (line.rfind("#PATH-POINTS-START", 0) == 0) {
      inPathSection = true;
      if (raw_path.size() > 0) {
        // There are two nodes at the end of the path, this deletes the second one
        raw_path.pop_back();
        // skip a line
        std::getline(file, line);
      }
      continue;
    }
    if (!inPathSection || line.empty() || line[0] == '#')
      continue;

    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ','))
      tokens.push_back(token);

    if (tokens.size() < 3)
      continue;

    double x = std::stod(tokens[0]) / 100.0; // cm to m
    double y = std::stod(tokens[1]) / 100.0;
    double rpm = std::stod(tokens[2]);
    double v = (rpm / 60.0) * 2 * M_PI * WHEEL_RADIUS_M;

    double heading = 0.0;

    bool is_node = tokens.size() == 4;

    TrajectoryPose traj_pose = {Pose(x, y, v, heading), 0.0, is_node};
    raw_path.emplace_back(traj_pose);
  }

  double total_time = 0.0;

  double current_node = 0;

  for (size_t i = 0; i < raw_path.size() - 1; ++i) {
    if (raw_path[i].is_node) {
      current_node++;
    }

    double dx = raw_path[i + 1].pose.x - raw_path[i].pose.x;
    double dy = raw_path[i + 1].pose.y - raw_path[i].pose.y;
    raw_path[i].pose.heading = std::atan2(dy, dx);

    double distance = std::hypot(dx, dy);
    double average_velocity = (raw_path[i + 1].pose.v + raw_path[i].pose.v) / 2.0;

    bool should_reverse = std::find(reverse_list.begin(), reverse_list.end(), current_node) != reverse_list.end();
    if (should_reverse) {
      raw_path[i].pose.heading = std::remainder(raw_path[i].pose.heading + M_PI, 2 * M_PI);
      raw_path[i].pose.v = -raw_path[i].pose.v;
    }

    double dt = (average_velocity > 0.01) ? distance / average_velocity : 0.01;
    if (i == 0) {
      dt = 0.0;
    }
    dt *= 1.0;
    total_time += dt;
    raw_path[i].time = total_time;
  }
  raw_path[raw_path.size() - 1].time = total_time;

  return raw_path;
}
