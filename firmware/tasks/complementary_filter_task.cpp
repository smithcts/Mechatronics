// Includes
#include "complementary_filter_task.h"
#include "coordinate_conversions.h"
#include "globs.h"
#include "modes_task.h"
#include "robot_settings.h"
#include "util_assert.h"

//******************************************************************************
ComplementaryFilterTask::ComplementaryFilterTask( float frequency) :
        PeriodicTask("Comp. Filter", TASK_ID_FILTER, frequency)
{
    current_step_ = READ_ACCEL;
}

//******************************************************************************
void ComplementaryFilterTask::initialize(void)
{
    uint32_t mpu_fail_count = 0;
    while (mpu_.initialize() != 0)
    {
        // Sensor failed to initialize properly.  Wait then try again.
        sys_timer.busyWait(0.5);
        if (++mpu_fail_count == 3)
        {
            assert_always_msg(ASSERT_CONTINUE, "MPU6000 failed to initialize.");
        }
    }
}

//******************************************************************************
void ComplementaryFilterTask::publishNewData(void)
{
    glo_imu.publish(&imu_);
    glo_raw_imu.publish(&raw_imu_);
    glo_quaternion.publish(&quaternion_);
    glo_roll_pitch_yaw.publish(&roll_pitch_yaw_);
}

//******************************************************************************
void ComplementaryFilterTask::run(void)
{
    switch (current_step_)
    {
        case READ_ACCEL:
            mpu_.readAccel(raw_imu_.accels);
            current_step_ = READ_GYRO;
            break;
        case READ_GYRO:
            mpu_.readGyro(raw_imu_.gyros);
            current_step_ = ESTIMATE_STATE;
            break;
        case ESTIMATE_STATE:
            // Actually run the filter now that we have new measurements.
            estimateState();
            publishNewData();
            current_step_ = READ_ACCEL; // start back at first step
    }
}

//******************************************************************************
void ComplementaryFilterTask::estimateState(void)
{
    // switch the axes (dependent on configuration) and apply accel calibration
    if (modes_task.inVerticalConfiguration())
    {
        imu_.accels[0] = -(raw_imu_.accels[2]*ACCEL_SCALES[2] + ACCEL_OFFSETS[2]);
        imu_.accels[1] = (raw_imu_.accels[1]*ACCEL_SCALES[1] + ACCEL_OFFSETS[1]);
        imu_.accels[2] = (raw_imu_.accels[0]*ACCEL_SCALES[0] + ACCEL_OFFSETS[0]);
        imu_.gyros[0] = -raw_imu_.gyros[2];
        imu_.gyros[1] = raw_imu_.gyros[1];
        imu_.gyros[2] = raw_imu_.gyros[0];
    }
    else // in horizontal configuration
    {
        imu_.accels[0] = (raw_imu_.accels[0]*ACCEL_SCALES[0] + ACCEL_OFFSETS[0]);
        imu_.accels[1] = (raw_imu_.accels[1]*ACCEL_SCALES[1] + ACCEL_OFFSETS[1]);
        imu_.accels[2] = (raw_imu_.accels[2]*ACCEL_SCALES[2] + ACCEL_OFFSETS[2]);
        imu_.gyros[0] = raw_imu_.gyros[0];
        imu_.gyros[1] = raw_imu_.gyros[1];
        imu_.gyros[2] = raw_imu_.gyros[2];
    }

    // run the complementary filter to calculate roll pitch yaw
    complementary_filter_.update(delta_t_, imu_.gyros, imu_.accels);
    complementary_filter_.getAttitude(quaternion_.q);
    quaternion_2_rpy(quaternion_.q, roll_pitch_yaw_.rpy);
}
