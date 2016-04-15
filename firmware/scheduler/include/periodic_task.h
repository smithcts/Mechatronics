#ifndef SCHEDULER_PERIODIC_TASK_H_INCLUDED
#define SCHEDULER_PERIODIC_TASK_H_INCLUDED

// Includes
#include "task.h"

namespace Scheduler {

// Specialized task that is scheduled to run at a specified rate.
class PeriodicTask : public Task
{
  public: // methods

    // Constructor. Frequency must be greater than zero and not greater than the main system timer.
    // If the desired frequency can't be achieved with the current system timer than an assert will fail.
    PeriodicTask(char const * task_name, task_id_t task_id, float frequency);

    // Return true if enough ticks have elapsed since last time trying to run task.
    virtual bool needToRun(void);

    // Decide the tick stamp that the task should run at next. Called right after run().
    virtual void decideWhenToRunNext(void);

    // Return true if it's time to process a section of code at the specified rate.
    // If trying to use a rate faster than the task itself then will return true
    // every call.  Also be aware of clipping.. if a task is running at 10Hz then
    // you could throttle to 5Hz, 3.33Hz, 2.5 Hz, 2 Hz, etc.  But trying to throttle
    // at 7Hz won't throttle at all.
    // Usage example:
    // if (throttle(0.2))
    // {
    //     some code to process at 5Hz
    // }
    bool throttle(float seconds);

    // Same as above but parameter is in hertz.  If throttle frequency is 0 then
    // return false every time.
    bool throttleHz(float frequency);

  private: // methods

    // Called by scheduler at the desired task frequency.
    virtual void run(void) = 0;

  protected: // fields

    // Frequency that task should try to run at in Hz.
    float frequency_;

    // Desired period (in seconds) between successive runs.
    float delta_t_;

    // Number of ticks to wait before trying to schedule again.
    // i.e. 1 = run at same rate as main system timer.
    uint32_t delay_ticks_;

    // The next tick stamp that the task should run at.
    uint64_t next_run_ticks_;

};

} // Scheduler namespace

#endif

