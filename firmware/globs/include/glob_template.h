#ifndef GLOB_TEMPLATE_H_INCLUDED
#define GLOB_TEMPLATE_H_INCLUDED

// Includes
#include "glob_base.h"

// Define a template class that specializes the generic glob base for different
// data types and different number of instances of those types.
template <typename object_type, const uint16_t num_instances, class OwnerTask>
class GlobTemplate : public GlobBase
{
  // Specify an owner which is in charge of publishing new data to the glob.
  friend OwnerTask;

  public: // methods

    // Constructor.  Add glob base information to global 'globs' array.
    GlobTemplate(uint8_t id);

    // Perform atomic copy of glob into output 'copy' parameter.  If successful
    // then return the timestamp of object (i.e. last time it was written to),
    // otherwise return -1 to signify error.
    double read(object_type *copy, uint16_t instance=1);

  private: // methods

    // Atomically copy 'data' to the glob instance. Return true if successful.
    bool publish(object_type const * data, uint16_t instance=1);

    // Glob data array. Atomic read/writes using template methods.
    object_type instances_[num_instances];

};

//*****************************************************************************
template <typename object_type, uint16_t num_instances, class OwnerTask>
GlobTemplate<object_type, num_instances, OwnerTask>::GlobTemplate(uint8_t id) :
    GlobBase(id, sizeof(object_type), num_instances, (void *)&(instances_))
{
    globs[id] = this;
}

//*****************************************************************************
template <typename object_type, uint16_t num_instances, class OwnerTask>
double GlobTemplate<object_type, num_instances, OwnerTask>::read(object_type * copy, uint16_t instance)
{
    double time_stamp = 0.0;

    if ((instance == 0) || (instance > num_instances) || (copy == NULL))
    {
        return -1.0; // invalid read request so return invalid time stamp
    }

    bool enabled = scheduler.disableInterrupts();

    memcpy((void *)copy, (void const *)&(instances_[instance-1]), num_bytes_);
    time_stamp = time_stamp_;

    scheduler.restoreInterrupts(enabled);

    return time_stamp;
}

//*****************************************************************************
template <typename object_type, uint16_t num_instances, class OwnerTask>
bool GlobTemplate<object_type, num_instances, OwnerTask>::publish(object_type const * new_data, uint16_t instance)
{
    if ((instance == 0) || (instance > num_instances) || (new_data == NULL))
    {
        return false; // invalid instance number or bad data
    }

    bool enabled = scheduler.disableInterrupts();

    memcpy((void *)(instances_ + (instance-1)), (void const *)new_data, num_bytes_);
    time_stamp_ = sys_timer.seconds();

    scheduler.restoreInterrupts(enabled);

    return true; // successfully published
}

#endif
