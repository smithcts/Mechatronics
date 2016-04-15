// Includes
#include "status_update_task.h"
#include "globs.h"
#include "math_util.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "telemetry_send_task.h"
#include "util_assert.h"

//******************************************************************************
StatusUpdateTask::StatusUpdateTask(float frequency) :
      PeriodicTask("Status", TASK_ID_STATUS_UPDATE, frequency),
      error_codes_(0)
{
}

//******************************************************************************
void StatusUpdateTask::readNewData(void)
{
    // Read in data needed to send status data to user.
    glo_status_data.read(&status_data_);
    glo_roll_pitch_yaw.read(&roll_pitch_yaw_);
    glo_odometry.read(&odometry_);
    glo_modes.read(&modes_);
    glo_analog.read(&analog_);
    glo_motor_pwm.read(&motor_pwm_);
}

//******************************************************************************
void StatusUpdateTask::publishNewData(void)
{
    glo_status_data.publish(&status_data_);
    send_task.send(glo_status_data.get_id());
}

//******************************************************************************
void StatusUpdateTask::run(void)
{
    readNewData();

    // Use odometry for yaw since complementary filter can't calculate it well.
    // Need to wrap value since main control task doesn't do this.
    float yaw = wrap_angle(odometry_.yaw);

    status_data_.battery = analog_.battery_voltage;
    status_data_.roll = roll_pitch_yaw_.rpy[0];
    status_data_.pitch = roll_pitch_yaw_.rpy[1];
    status_data_.yaw = yaw;
    status_data_.main_mode = modes_.main_mode;
    status_data_.sub_mode = modes_.sub_mode;
    status_data_.state = modes_.state;
    status_data_.error_codes = error_codes_;
    status_data_.left_linear_position = odometry_.left_distance;
    status_data_.right_linear_position = odometry_.right_distance;
    status_data_.left_angular_position = odometry_.left_distance / WHEEL_RADIUS;
    status_data_.right_angular_position = odometry_.right_distance / WHEEL_RADIUS;
    status_data_.left_linear_velocity = odometry_.left_speed;
    status_data_.right_linear_velocity = odometry_.right_speed;
    status_data_.left_angular_velocity = odometry_.left_speed / WHEEL_RADIUS;
    status_data_.right_angular_velocity = odometry_.right_speed / WHEEL_RADIUS;
    status_data_.left_pwm = motor_pwm_.left_duty;
    status_data_.right_pwm = motor_pwm_.right_duty;
    status_data_.firmware_version = FIRMWARE_VERSION;
    memcpy(status_data_.processor_id, (void *)0x1FFF7A10, 12);

    publishNewData();
}

//******************************************************************************
void StatusUpdateTask::setErrorCodes(glo_error_codes_t error_codes)
{
    error_codes_ |= error_codes;
}

//******************************************************************************
void StatusUpdateTask::clearErrorCodes(glo_error_codes_t error_codes)
{
    error_codes_ &= ~error_codes;
}


