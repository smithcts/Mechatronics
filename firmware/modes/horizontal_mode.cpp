// Includes
#include <cmath>
#include "telemetry_receive_task.h"
#include "globs.h"
#include "main_control_task.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "telemetry_send_task.h"
#include "util_assert.h"

//******************************************************************************
void MainControlTask::horizontalMode(void)
{
    yaw_command_ += motion_commands_.angular_velocity * delta_t_;

    float left_speed_error = motion_commands_.linear_velocity - odometry_.left_speed;
    float left_duty_cycle_command = left_speed_pid.calculate(left_speed_error, delta_t_);

    float right_speed_error = motion_commands_.linear_velocity - odometry_.right_speed;
    float right_duty_cycle_command = right_speed_pid.calculate(right_speed_error, delta_t_);

    // Run yaw controller to calculate desired difference in duty cycle between motors.
    float yaw_error = (yaw_command_ - odometry_.yaw);
    float delta_duty = yaw_pid.calculate(yaw_error, -imu_.gyros[2], delta_t_);

    if (modes_.state != STATE_NORMAL)
    {
        left_duty_cycle_command = 0.0f;
        right_duty_cycle_command = 0.0f;
        delta_duty = 0.0f;
        yaw_pid.resetIntegral();

        // Keep commands in sync with state to keep from building up large error.
        yaw_command_ = odometry_.yaw;
        motion_commands_.linear_velocity = 0.0f;
        motion_commands_.angular_velocity = 0.0f;
        receive_task.handle(motion_commands_); // request reset to be published.
    }

    motor_pwm_.left_duty = left_duty_cycle_command - delta_duty;
    motor_pwm_.right_duty = right_duty_cycle_command + delta_duty;

}
