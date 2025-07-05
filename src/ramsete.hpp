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
constexpr double B = 30.0;                                     // Ramsete aggressiveness factor
constexpr double ZETA = 1.3;                                   // Ramsete damping factor
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI);
constexpr double RPM_TO_RAD_S = (2.0 * M_PI) / 60.0;

// Unified Pose structure
struct Pose {
  double x;       // target x position in meters
  double y;       // target y position in meters
  double v;       // target linear velocity in m/s
  double heading; // target heading in radians
  double time;    // target time in seconds

  Pose(double x, double y, double v, double heading, double time)
      : x(x), y(y), v(v), heading(heading), time(time) {}

  Pose operator+(const Pose &a) const {
    return Pose(x + a.x, y + a.y, v + a.v, heading + a.heading, time + a.time);
  }

  Pose operator-(const Pose &a) const {
    return Pose(x - a.x, y - a.y, v - a.v, heading - a.heading, time - a.time);
  }
};

std::pair<double, double> output_to_speeds(double velocity, double angular_velocity) {
  double left_mps = velocity - angular_velocity * (TRACK_WIDTH_M / 2.0);
  double right_mps = velocity + angular_velocity * (TRACK_WIDTH_M / 2.0);

  double left_rpm = (left_mps / (WHEEL_RADIUS_M * M_PI)) * 60.0;
  double right_rpm = (right_mps / (WHEEL_RADIUS_M * M_PI)) * 60.0;

  return {left_rpm, right_rpm};
}

std::vector<Pose> loadJerryIOCSVPath(const std::string &pathFile) {
  std::ifstream file(pathFile);
  std::string line;
  std::vector<Pose> rawPath;
  bool inPathSection = false;

  while (std::getline(file, line)) {
    if (line.rfind("#PATH-POINTS-START", 0) == 0) {
      inPathSection = true;
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

    rawPath.emplace_back(x, y, v, heading, 0.0);
  }

  double total_time = 0.0;

  for (size_t i = 0; i < rawPath.size(); ++i) {
    double dx = rawPath[i + 1].x - rawPath[i].x;
    double dy = rawPath[i + 1].y - rawPath[i].y;
    rawPath[i].heading = std::atan2(dy, dx);

    double distance = std::hypot(dx, dy);
    double average_velocity = (rawPath[i + 1].v + rawPath[i].v) / 2.0;
    double dt = (average_velocity > 0.01) ? distance / average_velocity : 0.02;
    if (i == 0) {
      dt = 0.0;
    }
    dt *= 2.0;
    total_time += dt;
    rawPath[i].time = total_time;
  }

  return rawPath;
}

// Simple RAMSETE controller
class RamseteController {
public:
  RamseteController(double b, double z) : beta(b), zeta(z) {}

  std::pair<double, double> calculate(const Pose &current, const Pose &desired, double w_desired = 0.0) {
    double dx = desired.x - current.x;
    double dy = desired.y - current.y;

    double angleError = std::atan2(std::sin(desired.heading - current.heading),
                                   std::cos(desired.heading - current.heading));

    double cosTheta = std::cos(current.heading);
    double sinTheta = std::sin(current.heading);

    double x_error_robot = cosTheta * dx + sinTheta * dy;
    double y_error_robot = -sinTheta * dx + cosTheta * dy;

    double k = 2.0 * zeta * std::sqrt(w_desired * w_desired + beta * desired.v * desired.v);

    double v = desired.v * std::cos(angleError) + k * x_error_robot;

    double angular_velocity = w_desired + beta * desired.v * y_error_robot + k * angleError;

    return output_to_speeds(v, angular_velocity);
  }

private:
  double beta;
  double zeta;
};
