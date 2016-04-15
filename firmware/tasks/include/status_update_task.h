#ifndef STATUS_UPDATE_TASK_H_INCLUDED
#define STATUS_UPDATE_TASK_H_INCLUDED

// Includes
#include "digital_out.h"
#include "glob_types.h"
#include "periodic_task.h"

// Periodically send low-frequency status data back to user.
class StatusUpdateTask : public Scheduler::PeriodicTask
{
public: // methods

    // Constructor
    StatusUpdateTask(float frequency);

    // Set / clear error codes which will get included in next published status.
    void setErrorCodes(glo_error_codes_t error_codes);
    void clearErrorCodes(glo_error_codes_t error_codes);

private: // methods

    // Not implemented.
    virtual void initialize(void) {}

    // Send status data over telemetry.
    virtual void run(void);

    // Read in data from global objects (globs).
    void readNewData(void);

    // Publish data to all globs that this task owns.
    void publishNewData(void);

private: // fields

    // Globs from other tasks.
    glo_roll_pitch_yaw_t roll_pitch_yaw_;
    glo_odometry_t odometry_;
    glo_analog_t analog_;
    glo_motor_pwm_t motor_pwm_;

    // Globs that this task owns.
    glo_status_data_t status_data_;
    glo_modes_t modes_;

    // If a bit is set then the corresponding error code is active.
    glo_error_codes_t error_codes_;

};

// Task instance - defined in main.cpp
extern StatusUpdateTask status_update_task;

#endif
