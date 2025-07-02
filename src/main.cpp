#include "ramsete.hpp"
#include "raylib.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define FPS 100
#define IN_TO_PX 800.0 / 144.0
#define M_TO_PX IN_TO_PX * 100.0 / 2.54
#define ROBOT_WIDTH 12.5 * IN_TO_PX
#define ROBOT_LENGTH 15 * IN_TO_PX
#define TRAIL_THICKNESS 2

int main() {
  auto trajectory = loadJerryIOCSVPath("paths/path.jerryio-smooth.txt");
  RamseteController ramsete(B, ZETA);

  if (trajectory.empty()) {
    std::cerr << "No path points loaded.\n";
    return 1;
  }

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "RAMSETE visualizer");

  Texture2D robot_texture = LoadTexture("resources/robot.png");
  Rectangle robot_texture_source_rect = {0.0f, 0.0f, (float)robot_texture.width, (float)robot_texture.height};
  Texture2D field_texture = LoadTexture("resources/field.png");
  Rectangle field_texture_source_rect = {0.0f, 0.0f, (float)field_texture.width, (float)field_texture.height};

  Pose robot_pose = trajectory.front();

  Rectangle robot_rect = {WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX,
                          WINDOW_HEIGHT / 2 + robot_pose.y * M_TO_PX,
                          ROBOT_WIDTH,
                          ROBOT_LENGTH};
  Vector2 robot_origin = {ROBOT_WIDTH / 2, ROBOT_LENGTH / 2};
  double dt = 1.0 / FPS;
  int i = 0;

  SetTargetFPS(FPS);

  std::vector<Vector2> trail;

  while (!WindowShouldClose()) {
    Pose target = trajectory[i];

    double delta_heading = std::atan2(std::sin(target.heading - trajectory[i - 1].heading),
                                      std::cos(target.heading - trajectory[i - 1].heading));
    double w_desired = delta_heading / dt;

    auto [drive_left, drive_right] = ramsete.calculate(robot_pose, target);
    drive_left = std::clamp(drive_left, -450.0, 450.0);
    drive_right = std::clamp(drive_right, -450.0, 450.0);
    printf("%.2f, %.2f\n", drive_left, drive_right);

    double left_velocity = drive_left * (2 * M_PI * WHEEL_RADIUS_M) / 60.0;
    double right_velocity = drive_right * (2 * M_PI * WHEEL_RADIUS_M) / 60.0;

    double v_wheels = (left_velocity + right_velocity) / 2.0;
    double w_wheels = (right_velocity - left_velocity) / TRACK_WIDTH_M;

    robot_pose.x += v_wheels * std::cos(robot_pose.heading) * dt;
    robot_pose.y += v_wheels * std::sin(robot_pose.heading) * dt;
    robot_pose.heading += w_wheels * dt;

    robot_rect.x = WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX;
    robot_rect.y = WINDOW_WIDTH / 2 - robot_pose.y * M_TO_PX;

    if (i + 1 < trajectory.size()) {
      i++;
      trail.push_back({robot_rect.x, robot_rect.y});
    }

    BeginDrawing();
    ClearBackground(WHITE);

    DrawTexturePro(field_texture, field_texture_source_rect, {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT}, {0, 0}, 0.0, WHITE);
    DrawTexturePro(robot_texture, robot_texture_source_rect, robot_rect, robot_origin, 270 - robot_pose.heading * RAD_TO_DEG, BLACK);

    DrawText(TextFormat("%.2f", 100.0 * std::hypot(target.x - robot_pose.x, target.y - robot_pose.y)), 10, 10, 25, BLACK);

    for (int j = 0; j < trail.size() - 1; j++) {
      Vector2 start = trail[j];
      Vector2 end = trail[j + 1];
      DrawLineEx(start, end, TRAIL_THICKNESS, YELLOW);
    }

    for (int j = 0; j < trail.size() - 1; j++) {
      Pose start = trajectory[j];
      Pose end = trajectory[j + 1];
      Vector2 start_vec = {WINDOW_WIDTH / 2 + start.x * M_TO_PX, WINDOW_HEIGHT / 2 - start.y * M_TO_PX};
      Vector2 end_vec = {WINDOW_WIDTH / 2 + end.x * M_TO_PX, WINDOW_HEIGHT / 2 - end.y * M_TO_PX};
      DrawLineEx(start_vec, end_vec, TRAIL_THICKNESS, RED);
    }
    Vector2 target_px = {
        WINDOW_WIDTH / 2 + target.x * M_TO_PX,
        WINDOW_WIDTH / 2 - target.y * M_TO_PX};
    DrawCircleV(target_px, 5, RED);

    DrawCircleV({robot_rect.x, robot_rect.y}, 5, YELLOW);

    EndDrawing();
  }
  UnloadTexture(robot_texture);
  UnloadTexture(field_texture);

  CloseWindow();

  return 0;
}
