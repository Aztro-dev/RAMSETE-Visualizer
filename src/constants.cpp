#ifndef ROBOT_CONSTANTS
#define ROBOT_CONSTANTS

#define IN_TO_M (2.54 / 100.0)
#define IN_TO_PX (800.0 / 144.0)
#define M_TO_PX (IN_TO_PX * 100.0 / 2.54)
#define SEC_TO_uSEC 1000000
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
#define RAD_S_TO_RPM (60.0 / (2.0 * M_PI))
#define RPM_TO_RAD_S ((2.0 * M_PI) / 60.0)

#define WHEEL_RADIUS_M (2.75 / 2.0 * IN_TO_M) // 2.75 inch diameter / 2 -> meters
#define TRACK_WIDTH_M (12.5 * IN_TO_M)        // 12.5 inches -> meters
#define TRACK_HEIGHT_M (15.0 * IN_TO_M)       // 15.0 inches -> meters
#define ROBOT_WIDTH (12.5 * IN_TO_PX)
#define ROBOT_LENGTH (14.0 * IN_TO_PX)

#define MAX_SPEED_OUTPUT 600.0

#define ROBOT_MASS 12.0 / 2.2049 // lbs -> kg

#define MAX_BEAM_DISTANCE 2.0 // Distance sensors are only accurate up to 2m

#define NUM_PARTICLES 500 // 500 particles means quite good Localization, but high performance hit
#define NUM_BEAMS 4       // 4 distance sensors
#define NUM_WALLS 9       // Number of obstacles (field walls, goals, etc)

#endif
