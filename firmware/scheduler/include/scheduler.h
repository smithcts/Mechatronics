#ifndef SCHEDULER_H_INCLUDED
#define SCHEDULER_H_INCLUDED

// Includes
#include <cstddef>
#include "task.h"
#include "system_timer.h"

namespace Scheduler {

// Arbitrary limit.  Can be increased as necessary.
const uint8_t MAX_NUMBER_OF_TASKS = 16;

// Simple non-preemptive scheduler that supports task priorities.
// Each task is run-to-completion (RTC) and has logic built into to determine when it needs to run.
// There needs to be exactly one instance of this class defined by the user.
class Scheduler
{
  public: // methods

    // Constructor
    Scheduler(void);

    // Register task so that it's known to Scheduler.  The order that tasks are registered defines their
    // priorities.  Earlier registration = higher priority. Must be called for every task regardless of
    // how it's scheduled.  Return false if there was an error.
    bool registerTask(Task & task);

    // Continuously loops through tasks and determines which ones need to run.
    // This method will never return.
    void scheduleTasks(void);

    // Disable all interrupts and return the state of the interrupts before
    // interrupts were disabled.  Return true if interrupts were previously enabled.
    bool disableInterrupts(void) const;

    // Re-enable interrupts after they were disabled.  The enabled parameter should be
    // what was returned by the "disable interrupts" call, unless interrupts should
    // be restored regardless of the original state.. then 'enabled' should be true.
    void restoreInterrupts(bool enabled) const;

    // Return true if tasks should be recording information to use in analyzing scheduling.
    bool currentlyTimingTasks(void) const { return timing_tasks_; }

    // If not yet timing tasks then starts. If already timing tasks then finishes / sends the analysis.
    void timeTasks(void);

    // Flush all messages in send task.  Should usually only be called if a stop or restart assert fails.
    void flushOutgoingMessages(void);

    // Keep running receive task until no more data.  Should usually only be called if a stop or restart assert fails.
    void flushIncomingMessages(void);

    // Return the ID of the currently running task or TASK_ID_INVALID if no task is running.
    task_id_t runningTaskID(void) const { return running_task_id_; }

  private: // fields

    // Number of successfully registered tasks.
    uint8_t num_tasks_;

    // Task array that gets sequentially filled as tasks are registered.
    Task * tasks_[MAX_NUMBER_OF_TASKS];

    // Set to true when recording information about how well tasks are running.
    bool timing_tasks_;

    // The task that's currently being executed or TASK_ID_INVALID if one's not running.
    task_id_t running_task_id_;

};

} // Scheduler namespace

extern Scheduler::Scheduler scheduler;

#endif

