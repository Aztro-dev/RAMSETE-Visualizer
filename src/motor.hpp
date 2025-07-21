#include "ramsete.hpp"
#include <algorithm>
#include <cmath>

// Motor constants
#define MAX_RPM 600              // Max speed of the motor, rpm
#define STALL_TORQUE (2.1 / 6.0) // N⋅m for 600 RPM cartridge (1/6th of 100 RPM)
#define FREE_SPEED_TORQUE 0.0
#define TORQUE_DROPOFF_SPEED (0.6 * MAX_RPM) // 60% of max speed

// Robor Constants
#define GEAR_RATIO (1.0 / 1.0)
#define ROBOT_MASS 6.0 / 2.2049 // lbs -> kg
#define NUM_MOTORS_PER_SIDE 3

// Calculate the torque available from a motor at a given RPM
double get_motor_torque(double motor_rpm) {
  double abs_rpm = std::abs(motor_rpm);

  if (abs_rpm >= MAX_RPM) {
    return 0.0; // Motor can't exceed max RPM
  }

  if (abs_rpm <= TORQUE_DROPOFF_SPEED) {
    // Before dropoff point: constant torque
    return STALL_TORQUE;
  } else {
    // After dropoff point: linear decrease from stall torque to zero
    double remaining_speed_ratio = (MAX_RPM - abs_rpm) / (MAX_RPM - TORQUE_DROPOFF_SPEED);
    return STALL_TORQUE * remaining_speed_ratio;
  }
}

// Calculate maximum RPM change for one side of the drivetrain
double max_rpm_change(double current_rpm, double dt) {
  // Get current motor RPM (accounting for gear ratio)
  double motor_rpm = current_rpm / GEAR_RATIO;

  // Calculate available torque from all motors on this side
  double total_motor_torque = NUM_MOTORS_PER_SIDE * get_motor_torque(motor_rpm);

  // Convert motor torque to wheel torque (accounting for gear ratio)
  double wheel_torque = total_motor_torque * GEAR_RATIO;

  // Calculate force at wheel contact point
  double drive_force = wheel_torque / WHEEL_RADIUS_M;

  // Calculate maximum acceleration (F = ma, so a = F/m)
  // For one side of drivetrain, use half the robot mass
  double max_acceleration = drive_force / (ROBOT_MASS / 2.0);

  // Convert linear acceleration to angular acceleration of wheel
  double angular_acceleration = max_acceleration / WHEEL_RADIUS_M;

  // Convert to RPM/s
  double max_rpm_per_second = angular_acceleration * (60.0 / (2.0 * M_PI));

  // Calculate maximum change over the time step
  double max_change = max_rpm_per_second * dt;

  // Ensure we don't exceed motor limits
  double possible_max_rpm = current_rpm + max_change;
  if (possible_max_rpm > MAX_RPM * GEAR_RATIO) {
    max_change = MAX_RPM * GEAR_RATIO - current_rpm;
  }
  if (possible_max_rpm < -MAX_RPM * GEAR_RATIO) {
    max_change = -MAX_RPM * GEAR_RATIO - current_rpm;
  }

  return std::abs(max_change);
}
