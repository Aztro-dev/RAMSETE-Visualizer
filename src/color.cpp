#include "constants.cpp"
#include "raylib.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

Color velocity_to_color(double speed) {
  double t = std::fabs(speed) / (MAX_SPEED_OUTPUT * 2.0 * M_PI * WHEEL_RADIUS_M / 60.0);

  // Clamp to [0.0, 1.0] range (in case speed exceeds MAX_SPEED)
  if (t > 1.0f)
    t = 1.0f;

  if (t <= 0.5f) {
    // Interpolate from Red to Yellow (0.0 to 0.5)
    float factor = t * 2.0f;
    return ColorLerp(RED, YELLOW, factor);
  } else {
    // Interpolate from Yellow to Green (0.5 to 1.0)
    float factor = (t - 0.5f) * 2.0f;
    return ColorLerp(YELLOW, GREEN, factor);
  }
}
