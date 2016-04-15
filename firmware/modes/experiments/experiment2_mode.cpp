// Includes
#include <cmath>
#include "globs.h"
#include "main_control_task.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "util_assert.h"

//******************************************************************************
void MainControlTask::experiment2Mode(float experiment_input)
{
    if (modes_.state != STATE_NORMAL)
    {
        return;
    }

    // Wheel angular position control (in degrees).
    float position_command = experiment_input;

    float left_position_error = position_command - (odometry_.left_distance / WHEEL_RADIUS * RAD2DEG);
    motor_pwm_.left_duty = left_position_pid.calculate(left_position_error, delta_t_);

    float right_position_error = position_command - (odometry_.right_distance / WHEEL_RADIUS * RAD2DEG);
    motor_pwm_.right_duty = right_position_pid.calculate(right_position_error, delta_t_);

}
