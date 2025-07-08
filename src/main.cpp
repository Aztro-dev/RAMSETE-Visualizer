#include "ramsete.hpp"
#include "raylib.h"
#include <cstdint>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define TRAIL_THICKNESS 2

#define FPS 200

#define IN_TO_PX 800.0 / 144.0
#define M_TO_PX IN_TO_PX * 100.0 / 2.54
#define ROBOT_WIDTH 12.5 * IN_TO_PX
#define ROBOT_LENGTH 15 * IN_TO_PX

#define MAX_SPEED_OUTPUT 600.0

Color velocity_to_color(double v_wheels, double min_speed, double max_speed);
Color lerp_color(Color a, Color b, double t);

int main() {
  auto trajectory = loadJerryIOCSVPath("paths/path.jerryio-tourney.txt");
  RamseteController ramsete(B, ZETA);

  double min_speed = 0.0;
  double max_speed = 1.0;

  if (trajectory.empty()) {
    std::cerr << "No path points loaded.\n";
    return 1;
  }

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "RAMSETE visualizer");

  Texture2D robot_texture = LoadTexture("resources/robot.png");
  Rectangle robot_texture_source_rect = {0.0f, 0.0f, (float)robot_texture.width, (float)robot_texture.height};
  Texture2D field_texture = LoadTexture("resources/field.png");
  Rectangle field_texture_source_rect = {0.0f, 0.0f, (float)field_texture.width, (float)field_texture.height};

  Pose robot_pose = trajectory.front().pose;

  Rectangle robot_rect = {WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX,
                          WINDOW_HEIGHT / 2 + robot_pose.y * M_TO_PX,
                          ROBOT_WIDTH,
                          ROBOT_LENGTH};
  Vector2 robot_origin = {ROBOT_WIDTH / 2, ROBOT_LENGTH / 2};
  int i = 0;

  SetTargetFPS(FPS);

  std::vector<Pose> trail;

  double aggregated_error;
  double final_time = INFINITY;
  bool end = false;

  int current_node = 0;
  int current_node_index = -1;

  double time = 0.0;

  while (!WindowShouldClose()) {
    TrajectoryPose target = trajectory[i];
    time += GetFrameTime();

    while (i + 1 < trajectory.size() && trajectory[i + 1].time <= time) {
      i++;
    }

    if (target.is_node && current_node_index != i) {
      switch (current_node) {
      case 2: {
        WaitTime(1.5);
        time -= 1.5;
      }
      }
      printf("Node found: %d\n", current_node++);
      current_node_index = i;
    }
    printf("%f\n", robot_pose.heading * RAD_TO_DEG);

    double dt = time - target.time;

    double heading_error = target.pose.heading - trajectory[std::max(0, i - 1)].pose.heading;
    double delta_heading = std::atan2(std::sin(heading_error), std::cos(heading_error));
    double w_desired = delta_heading / dt;

    auto [drive_left, drive_right] = ramsete.calculate(robot_pose, target.pose);
    drive_left = std::clamp(drive_left, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
    drive_right = std::clamp(drive_right, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);

    double left_velocity = drive_left * (2 * M_PI * WHEEL_RADIUS_M) / 60.0;
    double right_velocity = drive_right * (2 * M_PI * WHEEL_RADIUS_M) / 60.0;

    double v_wheels = (left_velocity + right_velocity) / 2.0;
    double w_wheels = (right_velocity - left_velocity) / TRACK_WIDTH_M;

    robot_pose.x += v_wheels * std::cos(robot_pose.heading) * dt;
    robot_pose.y += v_wheels * std::sin(robot_pose.heading) * dt;
    robot_pose.heading += w_wheels * dt;

    Pose error = target.pose - robot_pose;
    double error_dist = std::hypot(error.x, error.y);

    aggregated_error += error_dist;

    robot_rect.x = WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX;
    robot_rect.y = WINDOW_WIDTH / 2 - robot_pose.y * M_TO_PX;

    if (i + 1 < trajectory.size()) {
      trail.push_back({robot_rect.x, robot_rect.y, v_wheels, robot_pose.heading});
    } else if (!end) {
      printf("Average error: %.3fm\n", aggregated_error / trajectory.size());
      final_time = time;
      end = true;
    }

    BeginDrawing();
    ClearBackground(WHITE);

    DrawTexturePro(field_texture, field_texture_source_rect, {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT}, {0, 0}, 0.0, WHITE);
    DrawTexturePro(robot_texture, robot_texture_source_rect, robot_rect, robot_origin, 270 - robot_pose.heading * RAD_TO_DEG, BLACK);

    DrawText(TextFormat("Err: %.2f", 100.0 * error_dist), 10, 10, 25, BLACK);
    DrawText(TextFormat("Time: %.1f", std::min(time, final_time)), 10, 40, 25, BLACK);

    for (int j = 0; j < trail.size() - 1; j++) {
      Pose start = trail[j];
      Pose end = trail[j + 1];
      Color start_color = velocity_to_color(start.v, min_speed, max_speed);
      Color end_color = velocity_to_color(end.v, min_speed, max_speed);
      Color trail_color = lerp_color(start_color, end_color, j / (trail.size() - 1));
      DrawLineEx({start.x, start.y}, {end.x, end.y}, TRAIL_THICKNESS * 2, trail_color);
    }

    for (int j = 0; j < trajectory.size() - 1; j++) {
      Pose start = trajectory[j].pose;
      Pose end = trajectory[j + 1].pose;
      Vector2 start_vec = {WINDOW_WIDTH / 2 + start.x * M_TO_PX, WINDOW_HEIGHT / 2 - start.y * M_TO_PX};
      Vector2 end_vec = {WINDOW_WIDTH / 2 + end.x * M_TO_PX, WINDOW_HEIGHT / 2 - end.y * M_TO_PX};
      DrawLineEx(start_vec, end_vec, TRAIL_THICKNESS, WHITE);
    }
    Vector2 target_px = {
        WINDOW_WIDTH / 2 + target.pose.x * M_TO_PX,
        WINDOW_WIDTH / 2 - target.pose.y * M_TO_PX};
    DrawCircleV(target_px, 5, RED);

    DrawCircleV({robot_rect.x, robot_rect.y}, 5, YELLOW);

    EndDrawing();
  }
  UnloadTexture(robot_texture);
  UnloadTexture(field_texture);

  CloseWindow();

  return 0;
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
    ratio = norm / 0.5;
    r = (int)(0 + ratio * 255);
    g = 255;
    b = 0;
  } else {
    ratio = (norm - 0.5) / 0.5;
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
