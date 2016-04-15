// Includes
#include <cmath>
#include "telemetry_receive_task.h"
#include "globs.h"
#include "main_control_task.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "telemetry_send_task.h"
#include "debug_printf.h"
#include "util_assert.h"

//******************************************************************************
void MainControlTask::balanceMode(void)
{
    // Integrate velocity commands to find position commands.
    distance_command_ += motion_commands_.linear_velocity * delta_t_;
    yaw_command_ += motion_commands_.angular_velocity * delta_t_;

    // Always try to balance at 0 tilt.
    float theta_cmd = 0;

    // Measure states. theta = tilt (i.e. pitch) and beta = wheel angle
    // Need to add tilt to encoder measurement to get absolute wheel angle
    float theta = roll_pitch_yaw_.rpy[1];
    float thetad = imu_.gyros[1];
    float beta_relative = odometry_.avg_distance / WHEEL_RADIUS;
    float beta = beta_relative + theta*(1 - 1.0f / GEAR_RATIO);
    float betad = beta_deriv_.calculate(beta_relative) + thetad*(1 - 1.0f / GEAR_RATIO);

    float beta_command = distance_command_ / WHEEL_RADIUS;

    // Multiply states by feedback gains to calculate duty cycle command.
    float duty_cycle = theta*K_[0] + thetad*K_[1] + (beta-beta_command)*K_[2] + betad*K_[3];

    float theta_error = theta_cmd - roll_pitch_yaw_.rpy[1];

    // Run yaw controller to calculate desired difference in duty cycle between motors.
    float yaw_error = yaw_command_ - odometry_.yaw;
    float delta_duty_cycle = yaw_pid.calculate(yaw_error, -imu_.gyros[2], delta_t_);

    bool fallen_down = (theta_error > 0.8f) || (theta_error < -0.8f);
    if (fallen_down)
    {
        duty_cycle = 0.0f;
        delta_duty_cycle = 0.0f;
        yaw_pid.resetIntegral();

        // Keep commands in sync with state to keep from building up large error.
        distance_command_ = odometry_.avg_distance;
        yaw_command_ = odometry_.yaw;
        motion_commands_.linear_velocity = 0.0f;
        motion_commands_.angular_velocity = 0.0f;
        receive_task.handle(motion_commands_); // request reset to be published.
    }

    motor_pwm_.left_duty = duty_cycle - delta_duty_cycle;
    motor_pwm_.right_duty = duty_cycle + delta_duty_cycle;

}
