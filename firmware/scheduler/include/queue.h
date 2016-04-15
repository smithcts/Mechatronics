#ifndef SCHEDULER_QUEUE_H_INCLUDED
#define SCHEDULER_QUEUE_H_INCLUDED

// Includes
#include "scheduler.h" // for enabling/disabling interrupts

namespace Scheduler {

// Atomic queue used to shared data (usually globs) between tasks.
template <class T>
class Queue
{
  public: // methods

    // Constructor. Dynamically allocates buffer with the specified size.
    Queue(uint32_t size) :
          size_(size)
    {
        data_ = new T[size_];
    }

    // Destructor.
    ~Queue(void)
    {
        delete[] data_;
    }

    // Copy 'data' into the queue. Return false if an error occurs, such as not
    // enough room in the buffer.
    bool enqueue(T & data);

    // Insert element directly in the front of the queue.  Which makes it not really a queue...
    bool enqueue_front(T & data);

    // Remove front item from queue and copies it to 'data' output parameter.
    // Return true if copy was successful.
    bool dequeue(T * data);

    // Copies front item from queue into 'data' output parameter but doesn't remove it.
    // Return true if copy was successful.
    bool peak(T * data);

    // Removes front element from queue. Useful if you've already peaked at it.
    bool remove(void);

    // Return number of elements currently stored in the queue.
    uint32_t count(void) const { return num_elements_; }

  private: // fields

      T * data_;      // buffer backing queue.
      uint32_t size_; // size of data buffer.

      uint32_t front_;        // index of next element to be dequeued.
      uint32_t back_;         // index of where to insert next element.
      uint32_t num_elements_; // how many valid elements are stored in queue.

};

//*****************************************************************************
template <class T>
bool Queue<T>::enqueue(T & data)
{
    bool interruptsEnabled = scheduler.disableInterrupts();

    if (num_elements_ >= size_)
    {
        scheduler.restoreInterrupts(interruptsEnabled);
        return false;  // no room
    }

    data_[back_] = data; // copy new data into queue.

    back_++;
    if (back_ == size_)
    {
        // Hit end of buffer so wrap back around to beginning.
        back_ = 0;
    }

    num_elements_++;

    scheduler.restoreInterrupts(interruptsEnabled);

    return true; // data successfully added
}

//*****************************************************************************
template <class T>
bool Queue<T>::enqueue_front(T & data)
{
    bool interruptsEnabled = scheduler.disableInterrupts();

    if (num_elements_ >= size_)
    {
        scheduler.restoreInterrupts(interruptsEnabled);
        return false;  // no roomsize_
    }

    // Make room in the front.
    if (front_ == 0)
    {
        front_ = size_-1;
    }
    else
    {
        front_--;
    }

    data_[front_] = data; // copy new data into queue.

    num_elements_++;

    scheduler.restoreInterrupts(interruptsEnabled);

    return true; // data successfully added
}

//*****************************************************************************
template <class T>
bool Queue<T>::dequeue(T * data)
{
    if (!peak(data))
    {
        return false;
    }

    return remove();
}

//*****************************************************************************
template <class T>
bool Queue<T>::peak(T * data)
{
    bool interruptsEnabled = scheduler.disableInterrupts();

    if (num_elements_ == 0)
    {
        scheduler.restoreInterrupts(interruptsEnabled);
        return false;
    }

    *data = data_[front_]; // copy data out of queue.

    scheduler.restoreInterrupts(interruptsEnabled);

    return true; // data successfully added
}

//*****************************************************************************
template <class T>
bool Queue<T>::remove(void)
{
    bool interruptsEnabled = scheduler.disableInterrupts();

    if (num_elements_ == 0)
    {
        scheduler.restoreInterrupts(interruptsEnabled);
        return false;
    }

    front_++;
    if (front_ == size_)
    {
        // Hit end of buffer so wrap back around to beginning.
        front_ = 0;
    }

    num_elements_--;

    scheduler.restoreInterrupts(interruptsEnabled);

    return true; // front element removed
}

} // Scheduler namespace

#endif

