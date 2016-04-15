#ifndef TELEMETRY_SEND_TASK_H_INCLUDED
#define TELEMETRY_SEND_TASK_H_INCLUDED

// Includes
#include "glo_tx_link.h"
#include "glob_types.h"
#include "queued_task.h"
#include "simple_array.h"

// Forward declarations
struct glob_queue_t;
struct glob_data_queue_t;

// Define easier to reference templated type.
typedef Scheduler::Queue<glob_queue_t> GlobQueue;

// Queued task (i.e. only runs when items are placed in its queue) that sends
// globs over a serial interface.
class TelemetrySendTask : public Scheduler::QueuedTask<glob_queue_t>
{
  public: // methods

    // Constructor
    TelemetrySendTask(uint32_t queue_size);

    // Take the data currently stored in the glob with specified id and instance, save off
    // a copy of it and put it in the send queue to be sent when the task runs.
    bool send_copy(uint8_t id, uint16_t instance=1);

    // Queue up glob with specified ID and instance number to the telemetry link.
    // If 'stop_instance' is non-zero and greater than 'instance' 5then the instances
    // up to the specified stop number will be sent.  For example if a glob has 30
    // instances and the instance argument is set to 5 and stop_instance is set to 23
    // then only instances 5 through 23 (inclusive) will be sent.  If stop_instance is
    // 0 then it has no effect. Return true if glob is successfully sent.
    // Note: This doesn't copy the glob data when this method is called. The data that is
    // sent is what's stored in the glob when it is dequeued in the run() method.
    bool send(uint8_t id, uint16_t instance=1, uint16_t stop_instance=0);

    // Send back all recent assert/debug messages in the order they were published.
    void send_cached_assert_messages(void);
    void send_cached_debug_messages(void);

    // Publish and send new assert message. Return true if message is sent.
    bool handle(glo_assert_message_t & message);

    // Publish and send new debug message. Return true if message is sent.
    bool handle(glo_debug_message_t & message);

    // Publish and send task timing glob. Return true if message is sent.
    bool handle(glo_task_timing_t const & timing);

  protected: // methods

    // Setup 'glo transfer link' with underlying serial port.
    virtual void initialize(void);

    // Pull items out of queue and send them over 'glo transfer link'.
    virtual void run(void);

  private: // fields

    // Transfer link for sending glob messages.
    GloTxLink * glo_tx_link_;

    // Serial bus wrapped by glo link.
    usart_bus_t bus_;
    Usart * serial_port_;

    // Buffer to save globs in until they can be sent.
    SimpleArray<glob_data_queue_t> save_buffer_;

    // Next instance numbers to publish debug/assert messages to.
    // Used for caching messages for the UI to request on connect.
    uint16_t next_assert_instance_;
    uint16_t next_debug_instance_;

};

// Data type stored in task queue. Stores glob meta-data so multiple glob types
// can be stored in the same queue. Individual tasks can define queue types
// for passing the same glob type, or any other data type.
struct glob_queue_t
{
    uint8_t     id;            // Unique ID associated with glob.
    uint16_t    instance;      // Instance number to send.
    uint16_t    stop_instance; // Instance number to stop sending at.  If 0 then will be ignored.
    array_idx_t storage_idx;   // Index associated with 'copy' of data or INVALID_ARRAY_INDEX.

    // Default Constructor.
    glob_queue_t(void) : id(0), instance(0), stop_instance(0), storage_idx(INVALID_ARRAY_INDEX) {}

    // Constructor. If no copy is required then 'data' must be null.
    glob_queue_t(uint8_t id, uint16_t instance, uint16_t stop_instance, array_idx_t storage_idx) :
        id(id), instance(instance), stop_instance(stop_instance), storage_idx(storage_idx) {}

};

// The data type stored in the 'save_buffer'.  256 bytes since that's the maximum size of glob data.
// Kind of wasteful since small globs will always take up the full space, but it's simpler to implement.
struct glob_data_queue_t
{
    uint8_t data[256];
};

// Task instance - defined in main.cpp
extern TelemetrySendTask send_task;

#endif
