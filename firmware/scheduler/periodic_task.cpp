// Includes
#include <cmath>
#include "periodic_task.h"
#include "system_timer.h"
#include "util_assert.h"

namespace Scheduler {

//*****************************************************************************
PeriodicTask::PeriodicTask(char const * task_name, task_id_t task_id, float frequency) :
    Task(task_name, task_id),
    frequency_(frequency),
    delay_ticks_(1),
    next_run_ticks_(0)
{
    assert_msg(frequency_ > 0, ASSERT_STOP, "Invalid frequency for periodic task.");
    assert_msg(frequency_ <= sys_timer.frequency(), ASSERT_STOP, "Periodic task can't run faster than main timer.");

    delta_t_ = 1.0f / frequency_;

    delay_ticks_ = (uint32_t)(sys_timer.frequency() / frequency_);

    // Get difference between desired frequency and the actual frequency the task is going to run at.
    float error_freq = fabs((float)sys_timer.frequency() / (float)delay_ticks_ - frequency_);
    // Convert to a percentage from 0 - 100 for normalization.
    float error_freq_percentage = error_freq / frequency_ * 100;

    assert_msg(error_freq_percentage < 1, ASSERT_STOP, "Invalid task frequency for current main timer frequency.");
}

//*****************************************************************************
bool PeriodicTask::needToRun(void)
{
    bool enough_ticks_elapsed = sys_timer.ticks() >= next_run_ticks_;
    return enough_ticks_elapsed || !currentStepIsDefault();
}

//*****************************************************************************
void PeriodicTask::decideWhenToRunNext(void)
{
    // Some periodic tasks can be split into smaller intermediate steps.  If it is then don't want
    // to decide when to run next until we're back at the first (default) step.
    if (!previousStepWasDefault())
    {
        return;
    }

    // Need to make sure task has ran once or else next_run_ticks_ won't be valid.
    if (num_times_ran_ > 1)
    {
        // How many ticks the task was later than we wanted it to be.
        late_ticks_ = started_first_step_tick_stamp_ - next_run_ticks_;

        // Keep track of how many times the task didn't get to run at all.
        times_tasked_skipped_ += late_ticks_ / delay_ticks_;
    }

    // Calculate the next time we want the task to run.  Don't just base it on the last time we wanted to run because
    // if a task ever over-runs a couple times in a row then that would cause it to keep getting rescheduled back to back.
    // Since this is a periodic task we don't care how many times it runs total, just that it runs as periodically as possible.
    // Example - want to run every 100 ticks. Last time we wanted to run at 500 but instead started at 643 ticks.
    // So we would do 643 % 100 = 43 ticks.  Then do 643 - 43 = 600 to get the nearest tick time and then add 100
    // to get 700 which is the next time we want to run.
    next_run_ticks_ = started_first_step_tick_stamp_ - (started_first_step_tick_stamp_ % delay_ticks_) + delay_ticks_;
}

//*****************************************************************************
bool PeriodicTask::throttle(float seconds)
{
    if (seconds <= 0) { return true; } // avoid undefined behavior.

    // If the current run counts is equal to a multiple of the mod counts then return
    // true that's it's time to process.  Need to make sure mod counts isn't zero or
    // that's undefined behavior.  Return true in that case since it means you're trying
    // to process faster than the task is running. Delay ticks can't be zero by invariant.
    uint32_t mod_counts = (uint32_t)(sys_timer.frequency() * seconds / delay_ticks_);

    if (mod_counts == 0) { return true; }

    return 0 == (num_times_ran_ % mod_counts);
}

//*****************************************************************************
bool PeriodicTask::throttleHz(float frequency)
{
    if (frequency <= 0) { return false; } // avoid division by zero.
    return throttle(1.0f / frequency);
}

} // Scheduler namespace
