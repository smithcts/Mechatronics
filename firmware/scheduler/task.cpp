// Includes
#include <cstring>
#include "task.h"
#include "system_timer.h"
#include "math_util.h"
#include "scheduler.h"

namespace Scheduler {

//*****************************************************************************
Task::Task(char const * task_name, task_id_t task_id) :
    name_(task_name),
    id_(task_id),
    initialized_(false),
    num_times_ran_(0),
    current_step_(0),
    current_executed_step_(0),
    scheduled_(false),
    scheduled_tick_stamp_(0),
    started_tick_stamp_(0),
    finished_tick_stamp_(0),
    previous_first_step_started_tick_stamp_(0),
    late_ticks_(0),
    times_tasked_skipped_(0),
    task_timing_start_skip_count_(0)
{
    resetTaskTimingFields();
}

//*****************************************************************************
void Task::tryInitialize(void)
{
    if (!initialized_)
    {
        initialize();
        initialized_ = true;
    }
}

//*****************************************************************************
void Task::execute(void)
{
    num_times_ran_++;

    started_tick_stamp_ = sys_timer.ticks();

    if (currentStepIsDefault())
    {
        // Save off the last time the task ran in case we need to use it for timing analysis.
        previous_first_step_started_tick_stamp_ = started_first_step_tick_stamp_;

        // If the task is split into smaller steps then it's useful to track the time the
        // first step started separately from the time the task was last ran.
        started_first_step_tick_stamp_ = started_tick_stamp_;
    }

    run();

    finished_tick_stamp_ = sys_timer.ticks();

    decideWhenToRunNext();

    scheduled_ = false;

    if (scheduler.currentlyTimingTasks())
    {
        recordTimingInfo();
    }

    // Now that execute() is finished we can update the current step.
    current_executed_step_ = current_step_;
}

//*****************************************************************************
bool Task::readyToRun(void)
{
    bool need_to_run = needToRun();

    if (need_to_run && !scheduled_)
    {
        // Record when task first wanted to run to use in task timing analysis.
        scheduled_tick_stamp_ = sys_timer.ticks();
        scheduled_ = true;
    }

    return need_to_run;
}

//*****************************************************************************
void Task::decideWhenToRunNext(void)
{
    // No default logic for running next (since default is for task to never run).
    // So just update the task delay.
    late_ticks_ = started_first_step_tick_stamp_ - scheduled_tick_stamp_;
}

//*****************************************************************************
void Task::startTimingAnalysis(void)
{
    task_timing_start_time_ = sys_timer.seconds();
    task_timing_start_skip_count_ = times_tasked_skipped_;
    task_timing_execute_counts_ = num_times_ran_;

    resetTaskTimingFields();
}

//*****************************************************************************
void Task::stopTimingAnalysis(glo_task_timing_t & timing)
{
    strcpy(timing.task_name, name_);
    //timing.task_id = id_;

    timing.timer_frequency = sys_timer.frequency();
    timing.recording_duration = sys_timer.seconds() - task_timing_start_time_;
    timing.execute_counts = task_timing_execute_counts_;
    timing.times_skipped = times_tasked_skipped_ - task_timing_start_skip_count_;

    timing.delay_ticks_max = delay_ticks_max_;
    timing.delay_ticks_min = delay_ticks_min_;
    timing.delay_ticks_avg = delay_ticks_sum_ / timing.execute_counts;
    timing.run_ticks_max = run_ticks_max_;
    timing.run_ticks_min = run_ticks_min_;
    timing.run_ticks_avg = run_ticks_sum_ / timing.execute_counts;
    timing.interval_ticks_max = interval_ticks_max_;
    timing.interval_ticks_min = interval_ticks_min_;
    timing.interval_ticks_avg = interval_ticks_sum_ / timing.execute_counts;
}

//*****************************************************************************
void Task::resetTaskTimingFields(void)
{
    task_timing_execute_counts_ = 0;
    delay_ticks_max_ = 0;
    delay_ticks_min_ = 0xFFFFFFFF;
    delay_ticks_sum_ = 0;
    run_ticks_max_ = 0;
    run_ticks_min_ = 0xFFFFFFFF;
    run_ticks_sum_ = 0;
    interval_ticks_max_ = 0;
    interval_ticks_min_ = 0xFFFFFFFF;
    interval_ticks_sum_ = 0;
}

//*****************************************************************************
void Task::recordTimingInfo(void)
{
    task_timing_execute_counts_++;

    // How long task took to run this time.
    uint32_t run_ticks = finished_tick_stamp_ - started_tick_stamp_;

    // How many ticks elapsed between the last run and this run.  If the task is split
    // up into smaller steps then ignore those sub-steps when calculating interval.
    uint32_t interval_ticks = started_first_step_tick_stamp_ - previous_first_step_started_tick_stamp_;

    // The following fields are used for tracking task timing characteristics.
    delay_ticks_max_ = max(delay_ticks_max_, late_ticks_);
    delay_ticks_min_ = min(delay_ticks_min_, late_ticks_);
    delay_ticks_sum_ += late_ticks_;
    run_ticks_max_ = max(run_ticks_max_, run_ticks);
    run_ticks_min_ = min(run_ticks_min_, run_ticks);
    run_ticks_sum_ += run_ticks;
    interval_ticks_max_ = max(interval_ticks_max_, interval_ticks);
    interval_ticks_min_ = min(interval_ticks_min_, interval_ticks);
    interval_ticks_sum_ += interval_ticks;
}

} // Scheduler namespace
