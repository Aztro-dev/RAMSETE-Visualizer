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
#define HEADING_THRESHOLD 5.0 * DEG_TO_RAD
#define CURVE_THRESHOLD 1.5 * DEG_TO_RAD
#define ROTATE_SPEED 3.0

void draw_path(std::vector<Pose> trail, std::vector<TrajectoryPose> trajectory, TrajectoryPose target, Rectangle robot_rect);
Color velocity_to_color(double v_wheels, double min_speed, double max_speed);
Color lerp_color(Color a, Color b, double t);

double min_speed = 0.0;
double max_speed = 1.0;

int main() {
  auto trajectory = loadJerryIOCSVPath("paths/skills.txt");
  RamseteController ramsete(B, ZETA);

  if (trajectory.empty()) {
    std::cerr << "No path points loaded.\n";
    return 1;
  }

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "RAMSETE visualizer");

  Texture2D robot_texture = LoadTexture("resources/robot.png");
  Rectangle robot_texture_source_rect = {0.0f, 0.0f, (float)robot_texture.width, (float)robot_texture.height};
  Texture2D field_texture = LoadTexture("resources/field_skills.png");
  Rectangle field_texture_source_rect = {0.0f, 0.0f, (float)field_texture.width, (float)field_texture.height};

  Pose robot_pose = trajectory.front().pose;

  Rectangle robot_rect = {WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX,
                          WINDOW_HEIGHT / 2 - robot_pose.y * M_TO_PX,
                          ROBOT_WIDTH,
                          ROBOT_LENGTH};
  Vector2 robot_origin = {ROBOT_WIDTH / 2, ROBOT_LENGTH / 2};
  int i = 0;

  SetTargetFPS(FPS);

  std::vector<Pose> trail;

  double final_time = INFINITY;
  bool end = false;

  int current_node = 0;
  int current_node_index = -1;
  bool rotating_in_place = false;

  double time = 0.0;

  double prev_drive_left = 0.0;
  double prev_drive_right = 0.0;

  while (!WindowShouldClose()) {
    time += GetFrameTime();
    if (!rotating_in_place) {
      while (i + 1 < trajectory.size() && trajectory[i + 1].time <= time) {
        i++;
      }
    }

    TrajectoryPose target = trajectory[i];

    // Find out if we are at the next node
    if (target.is_node && current_node_index != i) {
      current_node_index = i;
      current_node++;

      // If we are at the next node, check to see if we should turn in place or not
      rotating_in_place = false;
      std::vector<int> rotating_indices = {3, 6, 7, 10, 11, 13, 14, 18, 19};
      for (int i = 0; i < rotating_indices.size(); i++) {
        if (current_node == rotating_indices[i]) {
          rotating_in_place = true;
          break;
        }
      }

      if (rotating_in_place) {
        printf("Node %d reached — rotating in place (heading change: %.1f°)\n", current_node,
               fabs(fmod(trajectory[i + 1].pose.heading - trajectory[i].pose.heading + M_PI, 2 * M_PI) - M_PI) * RAD_TO_DEG);
      } else {
        printf("Node %d reached — curved path, rotating while moving\n", current_node);
      }
    }

    double dt;
    if (rotating_in_place) {
      dt = GetFrameTime(); // Use frame time for rotation
    } else {
      dt = time - target.time; // Use trajectory time for RAMSETE
      if (dt < 1e-6)
        dt = 1e-6;
    }

    double target_heading = target.pose.heading;
    if (rotating_in_place && i + 1 < trajectory.size()) {
      target_heading = trajectory[i + 1].pose.heading;
    }

    Pose error = target.pose - robot_pose;
    error.heading = std::atan2(std::sin(target_heading - robot_pose.heading),
                               std::cos(target_heading - robot_pose.heading));

    auto [drive_left, drive_right] = ramsete.calculate(robot_pose, target.pose);

    if (rotating_in_place) {
      double w_rotate = std::clamp(error.heading, -ROTATE_SPEED, ROTATE_SPEED);

      drive_left = (-w_rotate * TRACK_WIDTH_M / 2.0) * 60.0 / (2 * M_PI * WHEEL_RADIUS_M);
      drive_right = -drive_left;

      // Keep the current time the same
      time = target.time;

      rotating_in_place = std::abs(error.heading) > HEADING_THRESHOLD;
      if (!rotating_in_place) {
        printf("Rotation complete, resuming trajectory following\n");
      }
    }

    drive_left = std::clamp(drive_left, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);
    drive_right = std::clamp(drive_right, -MAX_SPEED_OUTPUT, MAX_SPEED_OUTPUT);

    double left_velocity = drive_left * (2 * M_PI * WHEEL_RADIUS_M) / 60.0;
    double right_velocity = drive_right * (2 * M_PI * WHEEL_RADIUS_M) / 60.0;

    double v_wheels = (left_velocity + right_velocity) / 2.0;
    double w_wheels = (right_velocity - left_velocity) / TRACK_WIDTH_M;

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

    robot_pose.x += v_wheels * std::cos(robot_pose.heading) * dt;
    robot_pose.y += v_wheels * std::sin(robot_pose.heading) * dt;
    robot_pose.heading += w_wheels * dt;

    robot_rect.x = WINDOW_WIDTH / 2 + robot_pose.x * M_TO_PX;
    robot_rect.y = WINDOW_HEIGHT / 2 - robot_pose.y * M_TO_PX;

    BeginDrawing();
    ClearBackground(WHITE);

    DrawTexturePro(field_texture, field_texture_source_rect, {0, 0, WINDOW_WIDTH, WINDOW_HEIGHT}, {0, 0}, 0.0, WHITE);
    DrawTexturePro(robot_texture, robot_texture_source_rect, robot_rect, robot_origin, 270 - robot_pose.heading * RAD_TO_DEG, BLACK);

    if (rotating_in_place) {
      DrawText("Rotating to heading...", 10, 10, 25, BLACK);
    } else {
      DrawText(TextFormat("Time: %.1f", std::min(time, final_time)), 10, 10, 25, BLACK);
    }

    DrawText("Left:       , Right:       ", 10, 40, 25, BLACK);
    int left_measure = MeasureText("Left: ", 25);
    int right_measure = MeasureText("Left:       , Right: ", 25);
    if ((int)(GetTime() * 1000.0) % 100 <= 5) {
      DrawText(TextFormat("%5.1f", drive_left), 10 + left_measure, 40, 25, BLACK);
      DrawText(TextFormat("%5.1f", drive_right), 10 + right_measure, 40, 25, BLACK);
      prev_drive_left = drive_left;
      prev_drive_right = drive_right;
    } else {
      DrawText(TextFormat("%5.1f", prev_drive_left), 10 + left_measure, 40, 25, BLACK);
      DrawText(TextFormat("%5.1f", prev_drive_right), 10 + right_measure, 40, 25, BLACK);
    }

    draw_path(trail, trajectory, target, robot_rect);

    EndDrawing();
  }
  UnloadTexture(robot_texture);
  UnloadTexture(field_texture);

  CloseWindow();

  return 0;
}

void draw_path(std::vector<Pose> trail, std::vector<TrajectoryPose> trajectory, TrajectoryPose target, Rectangle robot_rect) {
  for (int j = 0; j < trail.size() - 1; j++) {
    Pose start = trail[j];
    Pose end = trail[j + 1];
    Color start_color = velocity_to_color(start.v, min_speed, max_speed);
    Color end_color = velocity_to_color(end.v, min_speed, max_speed);
    Color trail_color = lerp_color(start_color, end_color, static_cast<double>(j) / (trail.size() - 1));
    DrawLineEx({start.x, start.y}, {end.x, end.y}, TRAIL_THICKNESS * 2, trail_color);
  }

  for (int j = 0; j < trajectory.size() - 1; j++) {
    Pose start = trajectory[j].pose;
    Pose end = trajectory[j + 1].pose;
    Vector2 start_vec = {WINDOW_WIDTH / 2 + start.x * M_TO_PX, WINDOW_HEIGHT / 2 - start.y * M_TO_PX};
    Vector2 end_vec = {WINDOW_WIDTH / 2 + end.x * M_TO_PX, WINDOW_HEIGHT / 2 - end.y * M_TO_PX};
    DrawLineEx(start_vec, end_vec, TRAIL_THICKNESS, WHITE);
  }

  int current_node = 1;
  for (int j = 0; j < trajectory.size() - 1; j++) {
    if (!trajectory[j].is_node) {
      continue;
    }

    Pose start = trajectory[j].pose;
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
