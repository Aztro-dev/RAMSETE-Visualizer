// While we would normally *want* to be as realistic as possible,
// The motor simulation is too unrealistic and relies on too many
// variables to make realistic.
#include "motor.hpp"
#include "ramsete.hpp"
#include "raylib.h"
#include <cstdint>

#define MOTOR_SIM

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define TRAIL_THICKNESS 2

#define FPS 100
#define TIMESTEP 0.010 // corresponds to robot's 10ms update rate
#define IN_TO_PX 800.0 / 144.0
#define M_TO_PX IN_TO_PX * 100.0 / 2.54
#define ROBOT_WIDTH 12.5 * IN_TO_PX
#define ROBOT_LENGTH 15 * IN_TO_PX

#define MAX_SPEED_OUTPUT 600.0
#define HEADING_THRESHOLD 5.0 * DEG_TO_RAD
void draw_path(std::vector<Pose> trail, std::vector<TrajectoryPose> trajectory, TrajectoryPose target, Rectangle robot_rect);
Color velocity_to_color(double v_wheels, double min_speed, double max_speed);
Color lerp_color(Color a, Color b, double t);
double pid_turn(double error);

double min_speed = 0.0;
double max_speed = 1.0;

int main() {
  std::vector<int> reverse_indices = {3, 6, 12};
  std::vector<int> rotating_indices = {4, 5, 7, 9, 13};
  auto trajectory = loadJerryIOCSVPath("paths/qualifier-awp.txt", reverse_indices);
  RamseteController ramsete(B, ZETA);

  if (trajectory.empty()) {
    std::cerr << "No path points loaded.\n";
    return 1;
  }

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "RAMSETE visualizer");

  Texture2D robot_texture = LoadTexture("resources/robot.png");
  Rectangle robot_texture_source_rect = {0.0f, 0.0f, (float)robot_texture.width, (float)robot_texture.height};
  Texture2D field_texture = LoadTexture("resources/field_match.png");
  Rectangle field_texture_source_rect = {0.0f, 0.0f, (float)field_texture.width, (float)field_texture.height};

  Vector2 robot_origin = {ROBOT_WIDTH / 2, ROBOT_LENGTH / 2};
  size_t i = 0;

  SetTargetFPS(FPS);

  std::vector<Pose> trail;

  double final_time = INFINITY;
  bool end = false;

  bool reverse_switch = false;

  int current_node = 0;
  size_t current_node_index = -1;
  bool rotating_in_place = false;

  double time = 0.0;

#ifdef MOTOR_SIM
  double prev_drive_left = 0.0;
  double prev_drive_right = 0.0;
#endif

  double accumulator = 0.0;
  double now;
  double frame_time;
  double last_time = GetTime();

  TrajectoryPose target = trajectory[0];
  Pose error = target.pose;
  double target_heading;
  double w_desired;
  std::pair<double, double> drive;
  double drive_left;
  double drive_right;
#ifdef MOTOR_SIM
  double max_change_right;
  double max_change_left;
  double desired_change_right;
  double desired_change_left;
#endif
  double left_velocity;
  double right_velocity;
  double v_wheels;
  double w_wheels;

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

    // If we couldn't find the requested node, fall back to start
    if (current_node_index == (size_t)-1) {
      std::cerr << "Warning: Could not find node " << current_node << ", starting from beginning.\n";
      current_node = 0;
      i = 0;
    }
  }

  time = trajectory[i].time;

  Pose robot_pose = trajectory[i].pose;

  Rectangle robot_rect = {WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX,
                          WINDOW_HEIGHT / 2 - robot_pose.y * M_TO_PX,
                          ROBOT_WIDTH,
                          ROBOT_LENGTH};

  while (!WindowShouldClose()) {
    now = GetTime();
    frame_time = now - last_time;
    last_time = now;

    if (IsKeyDown(KEY_SPACE)) {
      goto draw;
    }

    accumulator += frame_time;

    while (accumulator >= TIMESTEP) {
      time += TIMESTEP;
      accumulator -= TIMESTEP;
    }
    if (!rotating_in_place) {
      while (i + 1 < trajectory.size() && trajectory[i + 1].time <= time) {
        i++;
      }
    }

    target = trajectory[i];

    // Find out if we are at the next node
    if (target.is_node && current_node_index != i) {
      current_node_index = i;
      current_node++;

      // If we are at the next node, check to see if we should turn in place or not
      rotating_in_place = false;
      // std::vector<int> rotating_indices = {3, 5, 7, 12, 14, 16, 18, 20, 22, 26, 28, 29};
      for (size_t i = 0; i < rotating_indices.size(); i++) {
        if (current_node == rotating_indices[i]) {
          rotating_in_place = true;
          break;
        }
      }

      // These indices we are supposed to reverse in
      // std::vector<int> reverse_indices = {4, 6, 11, 13, 17, 19, 21, 25, 27};
      for (size_t i = 0; i < reverse_indices.size(); i++) {
        if (current_node == reverse_indices[i]) {
          reverse_switch = true;
        } else if (current_node == reverse_indices[i] + 1) {
          reverse_switch = false;
        }
      }

      if (rotating_in_place) {
        printf("Node %d reached — rotating in place (heading change: %.1f°)\n", current_node,
               fabs(fmod(trajectory[i + 1].pose.heading - trajectory[i].pose.heading + M_PI, 2 * M_PI) - M_PI) * RAD_TO_DEG);
      } else {
        printf("Node %d reached — curved path, rotating while moving\n", current_node);
      }
    }

    target_heading = target.pose.heading;
    if (rotating_in_place && i + 1 < trajectory.size()) {
      target_heading = trajectory[i + 1].pose.heading;
    }

    // Calculate desired angular velocity for RAMSETE feedforward
    w_desired = 0.0;
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

    error = target.pose - robot_pose;
    error.heading = std::atan2(std::sin(target_heading - robot_pose.heading),
                               std::cos(target_heading - robot_pose.heading));

    drive = ramsete.calculate(robot_pose, target.pose, w_desired);
    drive_left = drive.first;
    drive_right = drive.second;

    if (reverse_switch) {
      drive_left = -drive_left;
      drive_right = -drive_right;
    }

    error = target.pose - robot_pose;
    error.heading = std::atan2(std::sin(target_heading - robot_pose.heading),
                               std::cos(target_heading - robot_pose.heading));

    drive = ramsete.calculate(robot_pose, target.pose, w_desired);
    drive_left = drive.first;
    drive_right = drive.second;

    // printf("RAMSETE Output - Left RPM: %.1f, Right RPM: %.1f\n", drive_left, drive_right);

    if (rotating_in_place) {
      drive_left = pid_turn(error.heading);
      drive_right = -drive_left;

      // Keep the current time the same
      time = target.time;

      rotating_in_place = std::abs(error.heading) > HEADING_THRESHOLD;
      if (!rotating_in_place) {
        printf("Rotation complete, resuming trajectory following\n");
      }
    }

#ifdef MOTOR_SIM
    desired_change_left = drive_left - prev_drive_left;
    desired_change_right = drive_right - prev_drive_right;

    max_change_left = max_rpm_change(prev_drive_left, TIMESTEP);
    max_change_right = max_rpm_change(prev_drive_right, TIMESTEP);

    drive_left = prev_drive_left + std::clamp(desired_change_left, -max_change_left, max_change_left);
    drive_right = prev_drive_right + std::clamp(desired_change_right, -max_change_right, max_change_right);
#else
    drive_left = std::clamp(drive_left, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
    drive_right = std::clamp(drive_right, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
#endif

    left_velocity = drive_left * (2 * M_PI * WHEEL_RADIUS_M) / 60.0;
    right_velocity = drive_right * (2 * M_PI * WHEEL_RADIUS_M) / 60.0;

    v_wheels = (left_velocity + right_velocity) / 2.0;
    w_wheels = (right_velocity - left_velocity) / TRACK_WIDTH_M;

    if (!rotating_in_place) {
      if (i + 1 < trajectory.size()) {
        double px = WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX;
        double py = WINDOW_HEIGHT / 2 - robot_pose.y * M_TO_PX;
        trail.push_back({px, py, v_wheels, robot_pose.heading});
      } else if (!end) {
        final_time = time;
        end = true;
      }
    }

    // updates robot pose based on velocities
    robot_pose.x += v_wheels * std::cos(robot_pose.heading) * TIMESTEP;
    robot_pose.y += v_wheels * std::sin(robot_pose.heading) * TIMESTEP;
    robot_pose.heading += w_wheels * TIMESTEP;

    robot_rect.x = WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX;
    robot_rect.y = WINDOW_HEIGHT / 2 - robot_pose.y * M_TO_PX;

#ifdef MOTOR_SIM
    prev_drive_left = drive_left;
    prev_drive_right = drive_right;
#endif

  draw:
    BeginDrawing();
    ClearBackground(WHITE);

    DrawTexturePro(field_texture, field_texture_source_rect, {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT}, {0, 0}, 0.0, WHITE);
    DrawTexturePro(robot_texture, robot_texture_source_rect, robot_rect, robot_origin, 270 - robot_pose.heading * RAD_TO_DEG, BLACK);

    if (rotating_in_place) {
      DrawText("Rotating to heading...", 10, 10, 25, BLACK);
    } else {
      DrawText(TextFormat("Time: %.1f", std::min(time, final_time)), 10, 10, 25, BLACK);
    }
    draw_path(trail, trajectory, target, robot_rect);

    EndDrawing();
  } // end of while (!WindowShouldClose())

  // Cleanup resources after main loop
  UnloadTexture(robot_texture);
  UnloadTexture(field_texture);
  CloseWindow();

  return 0;
}

// Function definitions moved outside main()

void draw_path(std::vector<Pose> trail, std::vector<TrajectoryPose> trajectory, TrajectoryPose target, Rectangle robot_rect) {
  for (size_t i = 0; i < trail.size() - 1; i++) {
    Pose start = trail[i];
    Pose end = trail[i + 1];
    Color start_color = velocity_to_color(start.v, min_speed, max_speed);
    Color end_color = velocity_to_color(end.v, min_speed, max_speed);
    Color trail_color = lerp_color(start_color, end_color, static_cast<double>(i) / (trail.size() - 1));
    DrawLineEx({start.x, start.y}, {end.x, end.y}, TRAIL_THICKNESS * 2, trail_color);
  }

  for (size_t i = 0; i < trajectory.size() - 1; i++) {
    Pose start = trajectory[i].pose;
    Pose end = trajectory[i + 1].pose;
    Vector2 start_vec = {WINDOW_WIDTH / 2 + start.x * M_TO_PX, WINDOW_HEIGHT / 2 - start.y * M_TO_PX};
    Vector2 end_vec = {WINDOW_WIDTH / 2 + end.x * M_TO_PX, WINDOW_HEIGHT / 2 - end.y * M_TO_PX};
    DrawLineEx(start_vec, end_vec, TRAIL_THICKNESS, WHITE);
  }

  int current_node = 1;
  for (size_t i = 0; i < trajectory.size() - 1; i++) {
    if (!trajectory[i].is_node) {
      continue;
    }

    Pose start = trajectory[i].pose;
    Vector2 start_vec = {WINDOW_WIDTH / 2 + start.x * M_TO_PX, WINDOW_HEIGHT / 2 - start.y * M_TO_PX};
    DrawCircleV(start_vec, 10, BLUE);
    const char *text = TextFormat("%d", current_node++);
    Vector2 text_size = MeasureTextEx(GetFontDefault(), text, 10.0, 0.0);
    DrawText(text, start_vec.x - text_size.x / 2, start_vec.y - text_size.y / 2, 10, BLACK);
  }

  Vector2 target_px = {
      WINDOW_WIDTH / 2 + target.pose.x * M_TO_PX,
      WINDOW_HEIGHT / 2 - target.pose.y * M_TO_PX};
  DrawCircleV(target_px, 5, RED);

  DrawCircleV({robot_rect.x, robot_rect.y}, 5, YELLOW);
}

Color velocity_to_color(double speed, double min_speed, double max_speed) {
  double norm = 0.0;
  if (max_speed != min_speed) {
    norm = (speed - min_speed) / (max_speed - min_speed);
  }
  norm = std::clamp(norm, 0.0, 1.0);

  double ratio;
  uint8_t r;
  uint8_t g;
  uint8_t b;

  if (norm >= 0.5) {
    ratio = (norm - 0.5) / 0.5;
    ratio = std::clamp(ratio, 0.0, 1.0);
    r = (int)(0 + ratio * 255);
    g = 255;
    b = 0;
  } else {
    ratio = norm / 0.5;
    ratio = std::clamp(ratio, 0.0, 1.0);
    r = 255;
    g = int(255 - ratio * 255);
    b = 0;
  }

  return {r, g, b, 255};
}

Color lerp_color(Color a, Color b, double t) {
  return (Color){
      .r = (uint8_t)(a.r + t * (b.r - a.r)),
      .g = (uint8_t)(a.g + t * (b.g - a.g)),
      .b = (uint8_t)(a.b + t * (b.b - a.b)),
      .a = (uint8_t)(a.a + t * (b.a - a.a))};
}

#define TURN_KP 200.0 // Increased gain for faster turning
double pid_turn(double error) {
  // Simple proportional control: output RPM proportional to heading error
  double rpm = error * TURN_KP;
  rpm = -rpm;
  // Clamp to reasonable limits
  return std::clamp(rpm, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
}
