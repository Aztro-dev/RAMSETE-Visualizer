#include "color.cpp"
#include "constants.cpp"
#include "control.hpp"
#include "path.hpp"
#include "ramsete.hpp"
#include "raylib.h"
#include <cstdint>
#include <thread>
#include <unistd.h>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define TRAIL_THICKNESS 2

#define FPS 300

#define HEADING_THRESHOLD 5.0 * DEG_TO_RAD

void draw_path(std::vector<Pose> trail, std::vector<TrajectoryPose> trajectory, TrajectoryPose target, Rectangle robot_rect);

int main() {
  std::thread control_thread(control_robot, "paths/qualifier-AWP.txt");

  SetTraceLogLevel(LOG_ERROR); // Only show error and fatal messages

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "RAMSETE visualizer");

  Texture2D robot_texture = LoadTexture("resources/robot.png");
  Rectangle robot_texture_source_rect = {0.0f, 0.0f, (float)robot_texture.width, (float)robot_texture.height};
  Texture2D field_texture = LoadTexture("resources/field_match.png");
  Rectangle field_texture_source_rect = {0.0f, 0.0f, (float)field_texture.width, (float)field_texture.height};

  Vector2 robot_origin = {ROBOT_WIDTH / 2, ROBOT_LENGTH / 2};

  SetTargetFPS(FPS);

  std::vector<Pose> trail;

  Rectangle robot_rect = {WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX,
                          WINDOW_HEIGHT / 2 - robot_pose.y * M_TO_PX,
                          ROBOT_WIDTH,
                          ROBOT_LENGTH};

  std::vector<TrajectoryPose> trajectory = loadJerryIOCSVPath("paths/qualifier-AWP.txt", reverse_indices);

  while (!WindowShouldClose()) {
    position_mutex.lock();
    Pose robot_copy_pose = robot_pose;
    position_mutex.unlock();

    target_mutex.lock();
    TrajectoryPose target_copy = target;
    target_mutex.unlock();

    double px = WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX;
    double py = WINDOW_HEIGHT / 2 - robot_pose.y * M_TO_PX;
    trail.push_back({px, py, robot_copy_pose.v, robot_copy_pose.heading});

    robot_rect.x = WINDOW_WIDTH / 2 + robot_copy_pose.x * M_TO_PX;
    robot_rect.y = WINDOW_HEIGHT / 2 - robot_copy_pose.y * M_TO_PX;

    BeginDrawing();
    ClearBackground(WHITE);

    DrawTexturePro(field_texture, field_texture_source_rect, {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT}, {0, 0}, 0.0, WHITE);
    position_mutex.lock();
    DrawTexturePro(robot_texture, robot_texture_source_rect, robot_rect, robot_origin, 270 - robot_copy_pose.heading * RAD_TO_DEG, BLACK);
    position_mutex.unlock();

    DrawText(TextFormat("Time: %.1f", GetTime()), 10, 10, 25, BLACK);
    draw_path(trail, trajectory, target_copy, robot_rect);

    EndDrawing();
  }

  // Cleanup resources after main loop
  UnloadTexture(robot_texture);
  UnloadTexture(field_texture);
  CloseWindow();

  should_end = true;
  control_thread.join();

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
