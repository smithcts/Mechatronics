#ifndef GLOB_BASE_H_INCLUDED
#define GLOB_BASE_H_INCLUDED

// Includes
#include <cstring>
#include "scheduler.h"

// Give common meta data and functionality to all globs.
class GlobBase
{
  public: // methods

    // Constructor - see field descriptions
    GlobBase(uint8_t id, uint8_t num_bytes, uint16_t num_instances, void *data):
      id_(id),
      num_bytes_(num_bytes),
      num_instances_(num_instances),
      data_ptr_(data),
      time_stamp_(0)
    {}

    // Accessors
    uint8_t get_id(void) const { return id_; }
    uint16_t get_num_instances(void) const { return num_instances_; }
    uint8_t get_num_bytes(void) const { return num_bytes_; }
    const void * get_data_pointer(void) const { return data_ptr_; }

    // Return time the glob was last updated.  Atomic.
    double get_timestamp(void) const
    {
        double current_timestamp;
        bool enabled = scheduler.disableInterrupts();
        current_timestamp = time_stamp_;
        scheduler.restoreInterrupts(enabled);
        return current_timestamp;
    }

    // Copy 'instance' data directly to buffer.  Re-entrant.
    // Return false on failure.
    bool copy_to_buffer(void * buffer, uint16_t instance) const
    {
        if ((instance == 0) || (instance > num_instances_)) { return false; }

        // Need to subtract one from instance number since it's indexed off 1.
        uint32_t instance_offset = ((uint32_t)(instance-1)) * num_bytes_;

        uint32_t instance_ptr = (uint32_t)data_ptr_ + instance_offset;

        // Accessing non-atomic data so need to disable interrupts.
        bool enabled = scheduler.disableInterrupts();
        memcpy(buffer, (void const *)instance_ptr, num_bytes_);
        scheduler.restoreInterrupts(enabled);

        return true;
    }

  protected: // fields

    // Unique ID (ie 0, 1, 2, etc)
    const uint8_t id_;

    // Total number of bytes glob data takes up (so doesn't include base data)
    const uint8_t num_bytes_;

    // How many instances of the underlying data type is stored in the glob.
    const uint16_t num_instances_;

    // Pointer to first instance.
    const void * data_ptr_;

    // Time that the glob data was last updated.
    double time_stamp_;

};

// Externed here so template can use it to add new globs in the constructor.
// Defined in globs.cpp
extern GlobBase * globs[];

#endif
