#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

constexpr double WHEEL_RADIUS_M = (2.75 / 2.0) * 2.54 / 100.0; // 2.75 inch diameter / 2 -> meters
constexpr double TRACK_WIDTH_M = 12.5 * 2.54 / 100.0;          // 12.5 inches -> meters
constexpr double B = 2.7;                                      // Standard RAMSETE aggressiveness
constexpr double ZETA = 0.85;                                  // Standard RAMSETE damping
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI);
constexpr double RPM_TO_RAD_S = (2.0 * M_PI) / 60.0;

// Theta is in radians
double sinc(double theta) {
  if (theta < 1e-3) {
    return 1.0;
  } else {
    return std::sin(theta) / theta;
  }
}

// pose structure for the robot
struct Pose {
  double x;       // target x position in meters
  double y;       // target y position in meters
  double v;       // target linear velocity in m/s
  double heading; // target heading in radians

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

std::pair<double, double> output_to_speeds(double velocity, double angular_velocity) {
  double left_mps = velocity - angular_velocity * (TRACK_WIDTH_M / 2.0);
  double right_mps = velocity + angular_velocity * (TRACK_WIDTH_M / 2.0);

  double left_rpm = (left_mps / (2.0 * M_PI * WHEEL_RADIUS_M)) * 60.0;
  double right_rpm = (right_mps / (2.0 * M_PI * WHEEL_RADIUS_M)) * 60.0;

  return {left_rpm, right_rpm};
}

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
    raw_path.push_back(traj_pose);
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

  return raw_path;
}

// ramsete controller
class RamseteController {
public:
  RamseteController(double b, double z) : beta(b), zeta(z) {}

  std::pair<double, double> calculate(const Pose &current, const Pose &desired, double w_desired = 0.0) {
    double dx = desired.x - current.x;
    double dy = desired.y - current.y;

    double angle_error = std::atan2(std::sin(desired.heading - current.heading),
                                    std::cos(desired.heading - current.heading));

    double cosTheta = std::cos(current.heading);
    double sinTheta = std::sin(current.heading);

    double x_error_robot = cosTheta * dx + sinTheta * dy;
    double y_error_robot = -sinTheta * dx + cosTheta * dy;

    double k = 2.0 * zeta * std::sqrt(w_desired * w_desired + beta * desired.v * desired.v);

    double v = desired.v * std::cos(angle_error) + k * x_error_robot;

    double angular_velocity = w_desired + k * angle_error + beta * desired.v * y_error_robot * sinc(angle_error);

    return output_to_speeds(v, angular_velocity);
  }

private:
  double beta;
  double zeta;
};
