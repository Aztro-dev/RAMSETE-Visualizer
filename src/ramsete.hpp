#pragma once
#include "path.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

#define B 2.7     // Standard RAMSETE aggressiveness
#define ZETA 0.85 // Standard RAMSETE damping

// Theta is in radians
double sinc(double theta) {
  if (theta < 1e-3) {
    return 1.0;
  } else {
    return std::sin(theta) / theta;
  }
}

std::pair<double, double> output_to_speeds(double velocity, double angular_velocity) {
  double left_mps = velocity - angular_velocity * (TRACK_WIDTH_M / 2.0);
  double right_mps = velocity + angular_velocity * (TRACK_WIDTH_M / 2.0);

  double left_rpm = (left_mps / (2.0 * M_PI * WHEEL_RADIUS_M)) * 60.0;
  double right_rpm = (right_mps / (2.0 * M_PI * WHEEL_RADIUS_M)) * 60.0;

  return {left_rpm, right_rpm};
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
