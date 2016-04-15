// Define types of all globs. Instances are defined in globs.cpp
// To use globs include globs.h.
// To define new objects you must modify this file and globs.h

#ifndef GLOB_TYPES_H_INCLUDED
#define GLOB_TYPES_H_INCLUDED

// Includes
#include <cstdint>
#include "glob_constants.h"

//******************************************************************************
// Desired linear/angular velocity of robot.
typedef struct
{
     float linear_velocity;  // [m/s]
     float angular_velocity; // [rad/s] yaw rate

} glo_motion_commands_t;

//******************************************************************************
// Raw gyro and accel data that has NOT been calibrated and corrected for robot coordinate frame.
typedef struct
{
     float gyros[3];  // [rad/sec]
     float accels[3]; // [m/sec/sec]

} glo_raw_imu_t;

//******************************************************************************
// Gyro and accel data that has been calibrated and corrected for robot coordinate frame.
typedef struct
{
     float gyros[3];  // [rad/sec]
     float accels[3]; // [m/sec/sec]

} glo_imu_t;

//******************************************************************************
// Voltages reported by ADC.
typedef struct
{
    float voltages[9];     // [volts]
    float battery_voltage; // [volts]

} glo_analog_t;

//******************************************************************************
// Orientation estimate of robot as roll pitch yaw.
typedef struct
{
     float rpy[3]; // [radians]

} glo_roll_pitch_yaw_t;

//******************************************************************************
// Orientation estimate of robot as a quaternion.
typedef struct
{
     float q[4]; // [dimensionless]

} glo_quaternion_t;

//******************************************************************************
// Robot zero angle (for tilt) when balanced.
typedef struct
{
    float theta; // [rad]

} glo_theta_zero_t;

//******************************************************************************
// Robot state information calculated from odometry (i.e. using encoders).
typedef struct
{
    float left_distance;    // [m] distance left wheel has traveled
    float right_distance;   // [m] distance right wheel has traveled
    float avg_distance;     // [m] average of left and right wheel distances
    float yaw;              // [rad] heading angle
    float left_speed;       // [m/s] left wheel speed
    float right_speed;      // [m/s] right wheel speed
    float avg_speed;        // [m/s] average of left and right wheel speeds

} glo_odometry_t;

//******************************************************************************
// PID parameters used to sync/edit gains from GUI.
typedef struct
{
    float kp;               // Proportional gain
    float ki;               // Integral gain
    float kd;               // Derivative gain
    float integral_lolimit; // Lower saturation limit on integral state
    float integral_hilimit; // Upper saturation limit on integral state
    float lolimit;          // Lower saturation limit on controller output
    float hilimit;          // Upper saturation limit on controller output

} glo_pid_params_t;

//******************************************************************************
// Robot mode and operating state information.
typedef struct
{
    glo_main_modes_t main_mode;  // Balancing, line following, etc
    uint8_t sub_mode;            // Depends on main mode. Could be different experiments.
    glo_operating_state_t state; // Stopped, initializing, normal, etc

} glo_modes_t;

//******************************************************************************
// Outgoing telemetry assert message for user feedback. Automatically sent when assert fails.
typedef struct
{
    uint32_t action; // corresponds to enum in util_assert.h (e.g. continue, restart, stop, etc)
    char text[TELEMETRY_TEXT_SIZE];
    uint32_t valid;  // Non-zero if text is valid.

} glo_assert_message_t;

//******************************************************************************
// Outgoing telemetry debug log/status message.  Separate message to prevent overwriting
// assert message in telemetry queue. Use debug_printf() to send message.
typedef struct
{
    char text[TELEMETRY_TEXT_SIZE];
    uint32_t valid;  // Non-zero if text is valid.

} glo_debug_message_t;

//******************************************************************************
// Data that is transmitted when a capture command is received.
// Variable names are kept generic since what's being sent back changes frequently.
typedef struct
{
    float time; // seconds
    float d1;
    float d2;
    float d3;
    float d4;
    float d5;
    float d6;
    float d7;
    float d8;

} glo_capture_data_t;

//******************************************************************************
// Command that is received from user for driving robot.
typedef struct
{
    driving_command_id_t movement_type; // Drive forward, turn left, etc
    float linear_velocity;              // [meters / sec]
    float angular_velocity;             // [rad / sec]

} glo_driving_command_t;

//******************************************************************************
// Command that is received from user for starting/stopping data logging.
// This is also sent back to user when data logging completes.
typedef struct
{
    uint8_t is_start;         // False (0) if should stop sending data.
    uint8_t paused;           // Collection won't start until this is set to false.
    uint16_t frequency;       // Rate that data is recorded [Hz]
    uint32_t desired_samples; // How many samples to collect before stopping.
    uint32_t total_samples;   // Used to notify UI samples are done being sent and how many there should be.

} glo_capture_command_t;

//******************************************************************************
// For sending low rate periodic status data for user feedback.
typedef struct
{
    float battery; // [volts]
    float roll;    // [rad]
    float pitch;   // [rad]
    float yaw;     // [rad] this comes from odometry, not filter

    // See glo_modes_t (not used directly to keep globs independent)
    uint8_t main_mode;
    uint8_t sub_mode;
    uint8_t state;
    uint8_t error_codes;

    float left_linear_position;   // [meters]
    float right_linear_position;  // [meters]
    float left_angular_position;  // [radians]
    float right_angular_position; // [radians]
    float left_linear_velocity;   // [meters / sec]
    float right_linear_velocity;  // [meters / sec]
    float left_angular_velocity;  // [radians / sec]
    float right_angular_velocity; // [radians / sec]

    float left_pwm;      // from -1 to 1
    float right_pwm;     // from -1 to 1

    int32_t firmware_version; // Firmware version stored in robot settings.

    uint8_t processor_id[12]; // Unique processor ID read from ROM.

} glo_status_data_t;

//******************************************************************************
// PWM value applied to each motor.
typedef struct
{
    // Duty cycle from -1 (backwards) to 1 (forwards) applied to each motor.
    float left_duty;
    float right_duty;

} glo_motor_pwm_t;

//******************************************************************************
// Experiment wave input
typedef struct
{
    glo_wave_type_t type;   // type of wave (e.g sine)
    glo_wave_state_t state; // current state of wave (e.g. started / stopped)
    uint8_t pad[2];         // pad for alignment
    float value;            // current value of wave
    float magnitude;        // size of wave
    float frequency;        // Hz
    float duration;         // seconds
    float offset;           // constant wave offset
    float time;             // current time (in sec) of wave period.
    float total_time;       // how long (in sec) the wave has been running.
    uint8_t run_continuous; // if non-zero then is equivalent to duration being infinity
    uint8_t pad2[3];        // second set of pad bytes

    // Trapezoid parameters
    float vmax;                // maximum velocity
    float amax;                // maximum acceleration
    float dx;                  // change in X
    float t1, t2, t3;          // time splits for 3 parabolic splines
    float c1[3], c2[3], c3[3]; // coefficients for 3 parabolic splines

} glo_wave_t;

//******************************************************************************
// Request a glob to be sent back.
typedef struct
{
    uint8_t requested_id; // Glob ID that's being requested.

} glo_request_t;

//******************************************************************************
// Data recorded about a task when analyzing the task timing.
// Only published once the timing analysis is complete.
typedef struct
{
    char task_name[32];
    //int32_t task_id;

    uint32_t timer_frequency;

    float recording_duration;
    uint32_t execute_counts;
    uint32_t times_skipped;

    uint32_t delay_ticks_max, delay_ticks_min, delay_ticks_avg;
    uint32_t run_ticks_max, run_ticks_min, run_ticks_avg;
    uint32_t interval_ticks_max, interval_ticks_min, interval_ticks_avg;

} glo_task_timing_t;

#endif // GLOB_TYPES_H_INCLUDED
