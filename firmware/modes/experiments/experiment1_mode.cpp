// Includes
#include <cmath>
#include "globs.h"
#include "main_control_task.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "util_assert.h"

//******************************************************************************
void MainControlTask::experiment1Mode(float experiment_input)
{
    if (modes_.state != STATE_NORMAL)
    {
        return;
    }

    // Speed (velocity) control in meters/second.
    float speed_command = experiment_input;

    float left_speed_error = speed_command - odometry_.left_speed;
    motor_pwm_.left_duty = left_speed_pid.calculate(left_speed_error, delta_t_);

    float right_speed_error = speed_command - odometry_.right_speed;
    motor_pwm_.right_duty = right_speed_pid.calculate(right_speed_error, delta_t_);

}
