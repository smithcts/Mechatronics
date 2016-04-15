#ifndef COMPLEMENTARY_FILTER_TASK_H_INCLUDED
#define COMPLEMENTARY_FILTER_TASK_H_INCLUDED

// Includes
#include "complementary_filter.h"
#include "digital_out.h"
#include "glob_types.h"
#include "mpu6000.h"
#include "periodic_task.h"

// Estimate robot orientation using accel and gyro measurements.
class ComplementaryFilterTask : public Scheduler::PeriodicTask
{
private: // types

    // Since this task is in charge of reading the accel/gyro from IMU (which can be slow)
    // the task is split up into steps to keep it from running for too long each time.
    enum
    {
        READ_ACCEL,
        READ_GYRO,
        ESTIMATE_STATE
    };

public: // methods

    // Constructor
    ComplementaryFilterTask(float frequency);

private: // methods

    // Initialize sensors used by this task.
    virtual void initialize(void);

    // This will run the current 'step' of the  task, then update to move to the next step
    // for the next time run() is called.
    virtual void run(void);

    // Run filter to produce new roll-pitch-yaw measurement.
    void estimateState(void);

    // Publish data to all globs that this task owns.
    void publishNewData(void);

private: // fields

    // The IMU used for getting gyro and accel measurements.
    MPU6000 mpu_;

    // Filter used to provide state measurements.
    ComplementaryFilter complementary_filter_;

    // Globs that this task owns.
    glo_raw_imu_t raw_imu_;
    glo_imu_t imu_;
    glo_quaternion_t quaternion_;
    glo_roll_pitch_yaw_t roll_pitch_yaw_;

};

// Task instance - defined in main.cpp
extern ComplementaryFilterTask comp_filter_task;

#endif
