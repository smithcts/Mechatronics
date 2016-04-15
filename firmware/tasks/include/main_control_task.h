#ifndef MAIN_CONTROL_TASK_H_INCLUDED
#define MAIN_CONTROL_TASK_H_INCLUDED

// Includes
#include "analog_in.h"
#include "derivative_filter.h"
#include "digital_out.h"
#include "encoder.h"
#include "glob_types.h"
#include "periodic_task.h"
#include "pid_controller.h"
#include "tb6612fng.h"

// Functionality depends on current robot mode.
class MainControlTask : public Scheduler::PeriodicTask
{
public: // methods

    // Constructor
    MainControlTask(float frequency);

    // Clear and publish theta zero.
    void reset_zero_tilt_angle(void);

    // Validate and publish new capture command.
    void handle(glo_capture_command_t & command);

    // Publish wave settings
    void handle(glo_wave_t & wave);

    // Update full state feedback parameters for balancing.
    void handle_balance_tilt_gains(glo_pid_params_t & tilt_params);
    void handle_balance_position_gains(glo_pid_params_t & position_params);

    // Get current full state feedback gains used for balancing.
    void get_balance_tilt_gains(glo_pid_params_t & tilt_params);
    void get_balance_position_gains(glo_pid_params_t & position_params);

    // Reset commands and filters back to default state.
    void reset(void);

public: // fields

    // PID controllers. Public to keep in sync with global PID parameters.
    PidController yaw_pid;
    PidController left_speed_pid;
    PidController right_speed_pid;
    PidController left_position_pid;
    PidController right_position_pid;
    PidController line_track_pid;
    PidController track_maze_line_pid;

private: // methods

    // Perform actions that are independent of mode (reading in globs, odometry, etc)
    // and then runs method corresponding to current mode.
    virtual void run(void);

    // Setup task fields that can't be initialized in constructor.
    virtual void initialize(void);

    // Read in data from global objects (globs).
    void readNewData(void);

    // Publish data to all globs that this task owns.
    void publishNewData(void);

    // Validate the duty cycles stored in motor_pwm_ and write the values to the H-Bridge.
    // If there is a critical error, such as low battery, then the values will be set to 0.
    void updateMotorPWM(void);

    // Check if need to be capturing data and if so then saves data off
    // until it's time to send back.
    void runDataCapture(void);

    // Run full state feedback loop for balancing + yaw control.
    void balanceMode(void);

    // Run speed controllers for each wheel + yaw control for turning.
    void horizontalMode(void);

    // Use QTR array and PID loop to track a black line while traveling at a nominal velocity.
    void lineFollowingMode(void);

    // Use QTR array and PID loop to track a black line while traveling at a nominal velocity.
    void raceMode(void);

    // Customized mode for user to implement.
    void customMode(void);

    // Update experiment input and then run selected experiment.
    void experimentMode(void);
    void experiment1Mode(float experiment_input);
    void experiment2Mode(float experiment_input);
    void experiment3Mode(float experiment_input);

private: // fields

    // Used for odometry calculations.
    Encoder left_encoder_;
    Encoder right_encoder_;

    // Used to read battery voltage and QTR array.
    AnalogIn analog_inputs_;

    // Interface for setting voltage applied to each motor.
    TB6612FNG hbridge_;

    // Digital filters for estimating time derivatives.
    DerivativeFilter left_deriv_;
    DerivativeFilter right_deriv_;
    DerivativeFilter pos_cmd_deriv_;
    DerivativeFilter theta_cmd_deriv_;
    DerivativeFilter beta_deriv_;

    // Stored as fields since these commands are integrated from velocities.
    float distance_command_; // meters
    float yaw_command_;      // radians

    // Balancing full-state feedback gains. (tilt, tilt rate, wheel position, wheel velocity)
    float K_[4];

    // Flag so we know if data capture has already started.
    bool capturing_data_;

    // Counts up when capturing data to get time of reading.
    uint32_t capture_counter_;

    // How many times task has been ran since data collection was started.
    uint32_t capture_run_counts_;

    // Maximum number of sample data to record before stopping data capture.
    uint16_t max_samples_;

    // Globs from other tasks.
    glo_modes_t modes_;
    glo_motion_commands_t motion_commands_;
    glo_imu_t imu_;
    glo_roll_pitch_yaw_t roll_pitch_yaw_;
    glo_theta_zero_t theta_zero_;
    glo_status_data_t status_data_;

    // Globs that this task owns.
    glo_motor_pwm_t motor_pwm_;
    glo_odometry_t odometry_;
    glo_analog_t analog_;
    glo_wave_t wave_;
    glo_capture_command_t capture_command_;
    glo_capture_data_t capture_data_;

};

// Task instance - defined in main.cpp
extern MainControlTask main_control_task;

#endif
