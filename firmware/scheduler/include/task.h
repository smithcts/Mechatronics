#ifndef SCHEDULER_TASK_H_INCLUDED
#define SCHEDULER_TASK_H_INCLUDED

// Includes
#include <cstdint>
#include "glob_types.h"
#include "task_ids.h"

namespace Scheduler {

// Provide base functionality for a run-to-completion (RTC) task.  Meaning each time the
// task is executed, it must run all the way through.  The task is designed to be non-preemptive
// which means it can't interrupt a lower priority task whenever it wants.  So the idea is to minimize
// how long it takes the task to run in order to keep it from delaying tasks that are more important.
//
// The logic for when the task needs to run is included in this class.  By default
// the task will never report that it wants to run.  So to make this class useful
// your task must do one of the following:
//    1) Override the needToRun() method so that it doesn't always return false.
//    2) Instead of inheriting from this class, use one of the specialized classes
//       such as PeriodicTask or QueuedTask that already overrides needToRun().
//
// Since it's important to make each run() method return as fast as possible, this base class
// supports the concept of breaking the task down into smaller 'steps'. So each time the run()
// method is called it will execute the next step, and then when all the steps are completed it
// will reset back to the first step.  If this task is split into smaller steps it should:
//    1) Use the current_step_ field to track what step the task is on.
//    2) Make sure that the first (default) step has a value of 0.
class Task
{
  // Give Scheduler a more advanced interface for task setup and execution.
  friend class Scheduler;

  public: // methods

    // Constructor.
    Task(char const * task_name, task_id_t task_id);

    // Return name of task.
    char const * name(void) const { return name_; }

    // Return unique task ID.
    task_id_t task_id(void) const { return id_; }

  protected: // methods - for Scheduler friend class.

    // Should be called by Scheduler before any other methods are called.
    // Will only initialize the task if it's not already initialized.
    void tryInitialize(void);

    // Should only be called from the Scheduler.
    // Provide a non virtual interface (NVI) that will call the run() method
    // and also update some of the generic task properties (e.g. number of times ran)
    void execute(void);

    // Should only be called from the Scheduler.
    // Provide a non virtual interface (NVI) that will call the needToRun() method
    // and also update some of the generic task properties (e.g. when the task first wanted to run)
    virtual bool readyToRun(void);

    // Start/stop recording information about how well task is running.
    void startTimingAnalysis(void);
    void stopTimingAnalysis(glo_task_timing_t & timing);

  protected: // methods

    // Subclass must override.  Where the task should setup any fields that don't
    // make sense to setup in constructor (e.g. hardware references).
    // Called from try_initialize()
    virtual void initialize(void) = 0;

    // Subclass must override.  Where the run-to-completion (RTC) code should be.
    // Called from execute()
    virtual void run(void) = 0;

    // Subclass must override.  Should return true if task is ready to run.
    // Called from readyToRun()
    virtual bool needToRun(void) = 0;

    // Subclass can override if it needs a chance to decide when to run next.
    // This should also calculate how late the task was in running.  The default
    // is to base it on the first time the task reported ready to run... but for
    // tasks that know in advance when they need to run (e.g. PeriodicTasks) they
    // can calculate the delay much more accurately.
    virtual void decideWhenToRunNext(void);

    // If the task is broken up into smaller steps than this will return true
    // if the task is currently set to run the first (default) step the next
    // time run() is called.  If a task isn't broken into smaller steps then
    // this always returns true.
    bool currentStepIsDefault(void) const { return current_step_ == 0; }

    // This is similar to currentStepIsDefault() but the 'current step' only gets
    // updated at the end of execute().  So if a task runs the default (first) step
    // this will return true until execute() is finished.  If a task isn't broken
    // into smaller steps then this always returns true.
    bool previousStepWasDefault(void) const { return current_executed_step_ == 0; }

  private: // methods

    // Update class fields that have to do with recording task timing info.
    void recordTimingInfo(void);

    // Reset class fields that have to do with recording task timing info.
    void resetTaskTimingFields(void);

  protected: // fields

    // Name of task and it's unique ID.
    const char * name_;
    task_id_t    id_;

    // Set to true once the task's initialize() method has been called.
    bool initialized_;

    // How many times task has ran. Incremented right before task is ran so the first run will be 1.
    uint32_t num_times_ran_;

    // If a task is broken into smaller steps then this is the step that will run next time.
    int32_t current_step_;

    // Same as current_step_ but doesn't change until the end of the execute() method.  Useful
    // because some logic after run() requires knowing the current_step_ at the start of run(),
    // not the end of it.
    int32_t current_executed_step_;

    // Set to true when task needs to run.  Cleared when task actually does run.
    bool scheduled_;

    // Tick count when task last wanted to run.
    uint64_t scheduled_tick_stamp_;

    // Tick count right before task was last ran.
    uint64_t started_tick_stamp_;

    // Tick count right before task was last ran when it was at it's first (default) step.
    // If a task isn't split into smaller steps then this should be the same as started_tick_stamp_
    uint64_t started_first_step_tick_stamp_;

    // Tick count when task last finished.
    uint64_t finished_tick_stamp_;

    // Tick count right before task was ran the previous time (only updated if the task is running it's first step).
    uint64_t previous_first_step_started_tick_stamp_;

    // How many ticks elapsed after the task wanted to run before it actually got to.
    uint32_t late_ticks_;

    // Number of times that task want to run, but never got a chance to before it was time to run again.
    uint32_t times_tasked_skipped_;

    // Time (in seconds) when task started collecting timing information.
    double task_timing_start_time_;

    // How many times task has been skipped at the time of starting to collect timing information.
    uint32_t task_timing_start_skip_count_;

    // How many times the task was ran when collecting timing information.
    uint32_t task_timing_execute_counts_;

    // The following fields are used for tracking task timing characteristics.
    uint32_t delay_ticks_max_, delay_ticks_min_, delay_ticks_sum_;
    uint32_t run_ticks_max_, run_ticks_min_, run_ticks_sum_;
    uint32_t interval_ticks_max_, interval_ticks_min_, interval_ticks_sum_;

};

} // Scheduler namespace

#endif
