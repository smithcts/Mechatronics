// Includes
#include <cstdio>
#include <cstring>
#include "stm32f4xx.h"
#include "scheduler.h"
#include "math_util.h"
#include "debug_printf.h"
#include "globs.h"
#include "telemetry_receive_task.h"
#include "telemetry_send_task.h"

namespace Scheduler {

//*****************************************************************************
Scheduler::Scheduler(void) :
    num_tasks_(0),
    timing_tasks_(false),
    running_task_id_(TASK_ID_INVALID)
{
    for (uint8_t i = 0; i < MAX_NUMBER_OF_TASKS; i++)
    {
        tasks_[i] = NULL;
    }

    // Disable sub-priorities in NVIC (16 main priority levels)
    SCB->AIRCR = 0x05FA0300;
}

//*****************************************************************************
bool Scheduler::registerTask(Task & task)
{
    if (num_tasks_ >= MAX_NUMBER_OF_TASKS)
    {
        return false; // no more room for task.
    }

    tasks_[num_tasks_] = &task;

    num_tasks_++;

    return true; // task registered successfully
}

//*****************************************************************************
void Scheduler::scheduleTasks(void)
{
    // First go through and initialize all the tasks.
    for (uint8_t i = 0; i < num_tasks_; i++)
    {
        Task * task = tasks_[i];
        running_task_id_ = task->task_id();
        task->tryInitialize();
        running_task_id_ = TASK_ID_INVALID; // because task is done initializing.
    }

    while (true)
    {
        bool task_exectuted_this_loop = false;
        for (uint8_t i = 0; i < num_tasks_; i++)
        {
            Task * task = tasks_[i];

            // Every time we loop through all the tasks we only execute the first
            // one that wants to run. This is what allows certain tasks to be more important
            // than others.  Regardless of whether we have any intent of executing a task we
            // always need to check if it needs to run so it can mark if it does need to run
            // which can be useful in analyzing task scheduling conflicts.
            if (task->readyToRun() && !task_exectuted_this_loop)
            {
                running_task_id_ = task->task_id();
                task->execute();
                task_exectuted_this_loop = true;
                running_task_id_ = TASK_ID_INVALID; // because task is done running.
            }
        }
    }
}

//*****************************************************************************
bool Scheduler::disableInterrupts(void) const
{
    // Check state of interrupts before disabling so they can be restored later.
    bool already_enabled = (__get_PRIMASK() == 0);
    __disable_irq();
    return already_enabled;
}

//*****************************************************************************
void Scheduler::restoreInterrupts(bool enabled) const
{
    if (enabled)
    {
        __enable_irq();
    }
}

//*****************************************************************************
void Scheduler::timeTasks(void)
{
    if (timing_tasks_)
    {
        debug_printf("Stopping task timing.");
        glo_task_timing_t task_timing;
        for (uint8_t i = 0; i < num_tasks_; i++)
        {
            tasks_[i]->stopTimingAnalysis(task_timing);

            // Publish and send the results.
            send_task.handle(task_timing);
        }

        // Tell UI we're done sending task timing information.
        strcpy(task_timing.task_name, "done");
        //task_timing.id = TASK_ID_INVALID;
        send_task.handle(task_timing);
    }
    else
    {
        debug_printf("Starting task timing");
        for (uint8_t i = 0; i < num_tasks_; i++)
        {
            tasks_[i]->startTimingAnalysis();
        }
    }

    timing_tasks_ = !timing_tasks_;
}

//*****************************************************************************
void Scheduler::flushOutgoingMessages(void)
{
    running_task_id_ = send_task.task_id();

    // Make sure send task has been initialized.
    send_task.tryInitialize();

    while (send_task.readyToRun())
    {
        send_task.execute();
    }

    running_task_id_ = TASK_ID_INVALID;
}

//*****************************************************************************
void Scheduler::flushIncomingMessages(void)
{
    running_task_id_ = receive_task.task_id();

    // Make sure task has been initialized.
    receive_task.tryInitialize();

    while (receive_task.readyToRun())
    {
        receive_task.execute();
    }

    running_task_id_ = TASK_ID_INVALID;
}

} // Scheduler namespace

