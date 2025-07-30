#include "raylib.h"
#include <algorithm>
#include <cstdint>

double min_speed = 0.0;
double max_speed = 1.0;

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
