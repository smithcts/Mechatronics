#ifndef MODES_TASK_H_INCLUDED
#define MODES_TASK_H_INCLUDED

// Includes
#include "glob_types.h"
#include "periodic_task.h"
#include "user_pb.h"

// In charge of managing transition between robot modes (balance, line following, etc) and
// also between robot states (initializing, stopped, etc)
class ModesTask : public Scheduler::PeriodicTask
{
  public: // methods

    // Constructor
    ModesTask(float frequency);

    // Return true if robot is in a mode where it should be vertical.
    bool inVerticalConfiguration(void);

    // Copy new modes to be handled synchronously.
    void handle(glo_modes_t new_modes);

    // Set flag that a new command was received.
    void handle(glo_robot_command_t command);

  private: // methods

    // Initialize to first mode (i.e. waiting for push button)
    virtual void initialize(void);

    // Check state of user push-buttons and change mode/state accordingly.
    virtual void run(void);

    // Reset stateful information about robot (e.g. wheel angle)
    void reset(void);

    // Methods that are continuously ran when the robot is in a certain state.
    void stoppedState(void);
    void initializingState(void);
    void normalState(void);

    // Update and publish the modes glob.
    void changeState(glo_operating_state_t new_state);

  private: // fields

    // Used to switch between main robot modes (e.g. balance, line following, etc)
    UserPushButton modes_push_button_;

    // Used to change robot state (e.g. start, stop, etc)
    UserPushButton state_push_button_;

    // Globs that this task owns.
    glo_modes_t modes_;
    glo_theta_zero_t theta_zero_;

  private: // for vertical configuration initialization state machine.

    // Current vertical configuration initialization state.
    typedef uint8_t vc_init_state_type;
    enum
    {
        VC_INIT_STATE_RESET,
        VC_INIT_STATE_WAITING
    };
    vc_init_state_type vc_init_state_;

    // Time in seconds that the state machine should stop waiting.
    double vc_init_wait_stop_time_;

};

// Task instance - defined in main.cpp
extern ModesTask modes_task;

#endif
