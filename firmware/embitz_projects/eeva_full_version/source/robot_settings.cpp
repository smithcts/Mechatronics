
// Includes
#include "physical_constants.h"
#include "robot_settings.h"

// Current firmware version
const int32_t FIRMWARE_VERSION = 6;

// Wheel radius in meters.
const float WHEEL_RADIUS = 0.03f;

// Length of wheel base in meters
const float WHEEL_BASE = 0.089f;

// Motor gear ratio (output/input)
const float GEAR_RATIO = 29.86f;

// Number of encoder counts per motor shaft revolution (before gearbox).
const float DIST_PER_COUNT = 12.0f;

// Distance (in meters) wheel travels per encoder count.
const float ENCODER_2_DIST = WHEEL_RADIUS * 2.0f * PI / GEAR_RATIO / DIST_PER_COUNT;

// Angle that motor shaft turns (in radians) per encoder tick.
const float ENCODER_2_THETA = 2.0f * PI / GEAR_RATIO / DIST_PER_COUNT;

// Changes motor direction depending on wiring order. (left, right)
const float MOTOR_SIGNS[2] = { 1.0f, -1.0f };

// Scales encoder values for distance measurements.
// Also changes sign to account for wiring order.
const float ENCODER_SCALES[2] = { ENCODER_2_DIST, -ENCODER_2_DIST };

// Full-state feedback gains. (tilt, tilt rate, wheel position, wheel velocity)
const float K_DEFAULT[4] = { 1.656f,  0.10f,   0.05f,  0.04f };

// Scales and offsets for accelerometer.
const float ACCEL_SCALES[3] = { 1.0f, 1.0f, 1.0f };
const float ACCEL_OFFSETS[3] = { 0.0f, 0.0f, 0.0f };

// Scaling and offset for battery voltage
const float BATTERY_SCALE = 4.286f;
const float BATTERY_OFFSET = 0.343f;

// The lowest battery voltage that is considered 'full'.
const float FULL_BATTERY_VOLTAGE = 7.4;

// The highest battery voltage that is considered 'low' but not critical.
const float LOW_BATTERY_VOLTAGE = 6.8;

// The battery voltage that is considered too low and will cause the motors to not spin.
const float CRITICAL_BATTERY_VOLTAGE = 6.3;
