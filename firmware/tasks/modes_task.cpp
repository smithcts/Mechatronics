// Includes
#include <cmath>
#include "debug_printf.h"
#include "globs.h"
#include "main_control_task.h"
#include "modes_task.h"
#include "util_assert.h"

//******************************************************************************
ModesTask::ModesTask(float frequency) :
        PeriodicTask("Modes", TASK_ID_MODES, frequency),
        modes_push_button_(USER_PB_TOP),
        state_push_button_(USER_PB_BOTTOM),
        vc_init_state_(VC_INIT_STATE_RESET),
        vc_init_wait_stop_time_(0)
{
}

//******************************************************************************
void ModesTask::initialize(void)
{
    modes_.main_mode = MAIN_MODE_CUSTOM;        // Changed this so custom mode is first
    modes_.sub_mode = 0;
    changeState(STATE_STOPPED);
    glo_modes.publish(&modes_);
}

//******************************************************************************
void ModesTask::run(void)
{
    if (modes_push_button_.activated())
    {
        // Go to the next mode. This will automatically stop robot.
        glo_modes_t next_modes;
        next_modes.main_mode = (modes_.main_mode + 1) % NUM_MAIN_MODES;
        next_modes.sub_mode = 0;
        handle(next_modes);
    }

    switch (modes_.state)
    {
        case STATE_STOPPED:
            stoppedState();
            break;
        case STATE_INITIALIZING:
            initializingState();
            break;
        case STATE_NORMAL:
            normalState();
            break;
    }
}

//******************************************************************************
void ModesTask::reset(void)
{
    main_control_task.reset();
}

//******************************************************************************
bool ModesTask::inVerticalConfiguration(void)
{
    return (modes_.main_mode == MAIN_MODE_BALANCE) ||
           (modes_.main_mode == MAIN_MODE_CUSTOM);
}

//******************************************************************************
void ModesTask::handle(glo_modes_t new_modes)
{
    // First verify the robot mode is valid.
    if (new_modes.main_mode >= NUM_MAIN_MODES)
    {
        assert_always_msg(ASSERT_CONTINUE, "No mode with ID %d", (int)new_modes.main_mode);
        new_modes.main_mode = MAIN_MODE_BALANCE;
    }

    // Then verify the sub mode is valid if necessary.
    if ((new_modes.main_mode == MAIN_MODE_EXPERIMENT) &&
        (new_modes.sub_mode >= NUM_EXPERIMENTS))
    {
        assert_always_msg(ASSERT_CONTINUE, "No experiment with ID %d", (int)new_modes.sub_mode);
        new_modes.sub_mode = EXPERIMENT_NONE_SELECTED;
    }

    bool main_mode_changed = modes_.main_mode != new_modes.main_mode;
    bool sub_mode_changed = modes_.sub_mode != new_modes.sub_mode;
    if (main_mode_changed || sub_mode_changed)
    {
        changeState(STATE_STOPPED);
        reset();
    }

    // Don't let state change from outside task.
    new_modes.state = modes_.state;

    modes_ = new_modes;
    glo_modes.publish(&modes_);

}

//******************************************************************************
void ModesTask::handle(glo_robot_command_t command)
{
    switch (command)
    {
        case ROBOT_COMMAND_START:
            changeState(STATE_INITIALIZING);
            break;
        case ROBOT_COMMAND_STOP:
            changeState(STATE_STOPPED);
            break;
        case ROBOT_COMMAND_RESET:
            reset();
            break;
        case ROBOT_COMMAND_TIME_TASKS:
            scheduler.timeTasks();
            break;
    }
}

//******************************************************************************
void ModesTask::stoppedState(void)
{
    if (state_push_button_.activated())
    {
        changeState(STATE_INITIALIZING);
    }
}

//******************************************************************************
void ModesTask::initializingState(void)
{
    if (inVerticalConfiguration())
    {
        switch (vc_init_state_)
        {
            case VC_INIT_STATE_RESET:
                // Wait for a brief period for user to move hand away from robot.
                main_control_task.reset_zero_tilt_angle();
                vc_init_wait_stop_time_ = sys_timer.seconds() + .3;
                vc_init_state_ = VC_INIT_STATE_WAITING;
                break;
            case VC_INIT_STATE_WAITING:
                if (sys_timer.seconds() > vc_init_wait_stop_time_)
                {
                    changeState(STATE_NORMAL);
                    vc_init_state_ = VC_INIT_STATE_RESET;
                }
                break;
        }
    }
    else // not in vertical configuration so can straight to next state.
    {
        changeState(STATE_NORMAL);
    }
}

//******************************************************************************
void ModesTask::normalState(void)
{
    if (state_push_button_.activated())
    {
        changeState(STATE_STOPPED);
    }
}

//******************************************************************************
void ModesTask::changeState(glo_operating_state_t new_state)
{
    modes_.state = new_state;
    glo_modes.publish(&modes_);
}
