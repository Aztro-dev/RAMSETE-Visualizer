#ifndef ROBOT_CONSTANTS
#define ROBOT_CONSTANTS

#define IN_TO_PX (800.0 / 144.0)
#define M_TO_PX (IN_TO_PX * 100.0 / 2.54)
#define SEC_TO_uSEC 1000000
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
#define RAD_S_TO_RPM (60.0 / (2.0 * M_PI))
#define RPM_TO_RAD_S ((2.0 * M_PI) / 60.0)

#define WHEEL_RADIUS_M (2.75 / 2.0 * 2.54 / 100.0) // 2.75 inch diameter / 2 -> meters
#define TRACK_WIDTH_M (12.5 * 2.54 / 100.0)        // 12.5 inches -> meters
#define ROBOT_WIDTH (12.5 * IN_TO_PX)
#define ROBOT_LENGTH (14.0 * IN_TO_PX)

#define MAX_SPEED_OUTPUT 600.0

#define ROBOT_MASS 12.0 / 2.2049 // lbs -> kg
#endif
