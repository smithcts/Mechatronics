#ifndef SCHEDULER_QUEUED_TASK_H_INCLUDED
#define SCHEDULER_QUEUED_TASK_H_INCLUDED

// Includes
#include "queue.h"
#include "task.h"

namespace Scheduler {

// Specialized task that has a built in queue used to pass data to the task.
// The task will be set pending whenever it has one or more items in its queue.
// The templated type is the type of data to store in the queue.
template<class T>
class QueuedTask : public Task
{
  public: // methods

    // Constructor.
    QueuedTask(char const * task_name, task_id_t task_id, uint32_t queue_size) :
        Task(task_name, task_id),
        queue_(queue_size)
    { }

    // Copy 'data' into queue and sets task pending. Return true if successful.
    bool enqueue(T & data);

    // Return true if there's anything in the task queue.
    virtual bool needToRun(void);

  private: // methods

    // Run by Scheduler when items are placed in the queue.
    virtual void run(void) = 0;

  protected: // fields

    // Putting items in this queue will cause the task to be scheduled to run.
    Queue<T> queue_;

};

//*****************************************************************************
template<class T>
bool QueuedTask<T>::enqueue(T & data)
{
    bool success = queue_.enqueue(data);
    return success;
}

//*****************************************************************************
template<class T>
bool QueuedTask<T>::needToRun(void)
{
    return (queue_.count() > 0) || !currentStepIsDefault();
}

} // Scheduler namespace

#endif

