// Includes
#include "dma_rx.h"
#include "dma_tx.h"
#include "globs.h"
#include "telemetry_send_task.h"
#include "usart.h"
#include "util_assert.h"

//******************************************************************************
TelemetrySendTask::TelemetrySendTask(uint32_t queue_size) :
        QueuedTask<glob_queue_t>("Send", TASK_ID_TELEM_SEND, queue_size),
        glo_tx_link_(NULL),
        bus_(USART_BUS_2),
        serial_port_(NULL),
        save_buffer_(15),
        next_assert_instance_(1),
        next_debug_instance_(1)
{
}

//******************************************************************************
bool TelemetrySendTask::send_copy(uint8_t id, uint16_t instance)
{
    // Request a spot to save data.  If there aren't any available then it won't be saved
    // and when it comes time to send it will just send what's currently stored.
    array_idx_t storage_idx = INVALID_ARRAY_INDEX;
    void * storage_buffer = save_buffer_.requestStorage(&storage_idx);
    globs[id]->copy_to_buffer(storage_buffer, instance);
    assert_msg(storage_idx != INVALID_ARRAY_INDEX, ASSERT_CONTINUE, "Telem save buffer too small.");

    glob_queue_t new_element(id, instance, 0, storage_idx);

    return enqueue(new_element);
}

//******************************************************************************
bool TelemetrySendTask::send(uint8_t id, uint16_t instance, uint16_t stop_instance)
{
    glob_queue_t new_element(id, instance, stop_instance, INVALID_ARRAY_INDEX);

    return enqueue(new_element);
}

//******************************************************************************
void TelemetrySendTask::initialize(void)
{
    // Setup transfer link.
    // Make sure interrupts are disabled for initialization because the
    // first dma_tx error occurs before the fields the dma_tx are populated
    bool enabled = scheduler.disableInterrupts();
    serial_port_ = Usart::instance(bus_);
    scheduler.restoreInterrupts(enabled);
    assert(serial_port_ != NULL, ASSERT_STOP);

    glo_tx_link_ = new GloTxLink(serial_port_);
    assert(glo_tx_link_ != NULL, ASSERT_STOP);
}

//******************************************************************************
void TelemetrySendTask::run(void)
{
    glob_queue_t glob;
    if (queue_.peak(&glob))
    {
        // The ID signifying the result (e.g. success, failure) from sending the current glob.
        int send_result;

        if (glob.storage_idx != INVALID_ARRAY_INDEX)
        {
            // The glob data was saved so read it back out and send it.
            void * saved_data = save_buffer_.reference(glob.storage_idx);
            send_result = glo_tx_link_->send(glob.id, glob.instance, saved_data);
            save_buffer_.remove(glob.storage_idx);
        }
        else // just send what's currently stored in glob.
        {
            send_result = glo_tx_link_->send(glob.id, glob.instance);
        }

        if (send_result != SEND_ERROR_NO_ROOM)
        {
            // Remove element since we either sent it or won't be able to send it.
            queue_.remove();

            // Check if we need to keep sending more instances of this glob.
            // This just puts another element in the queue so it lets the task return quickly.
            // Need to put in front of the queue because some messages rely on being sent all at once.
            if ((glob.stop_instance > 0) && (glob.stop_instance > glob.instance))
            {
                glob_queue_t next_glob(glob.id, glob.instance+1, glob.stop_instance, INVALID_ARRAY_INDEX);
                queue_.enqueue_front(next_glob);
            }
        }
    }
}

//******************************************************************************
void TelemetrySendTask::send_cached_assert_messages(void)
{
    for (uint16_t i = 0; i < glo_assert_message.get_num_instances(); ++i)
    {
        uint16_t instance = (next_assert_instance_ + i) % (glo_assert_message.get_num_instances() + 1);
        if (instance < next_assert_instance_) { instance++; }
        this->send(GLO_ID_ASSERT_MESSAGE, instance);
    }
}

//******************************************************************************
void TelemetrySendTask::send_cached_debug_messages(void)
{
    for (uint16_t i = 0; i < glo_debug_message.get_num_instances(); ++i)
    {
        uint16_t instance = (next_debug_instance_ + i) % (glo_debug_message.get_num_instances() + 1);
        if (instance < next_debug_instance_) { instance++; }
        this->send(GLO_ID_DEBUG_MESSAGE, instance);
    }
}

//******************************************************************************
bool TelemetrySendTask::handle(glo_assert_message_t & message)
{
    message.valid = true;
    glo_assert_message.publish(&message, next_assert_instance_);
    bool success = this->send_copy(glo_assert_message.get_id(), next_assert_instance_);
    next_assert_instance_ = (next_assert_instance_ % glo_assert_message.get_num_instances()) + 1;
    return success;
}

//******************************************************************************
bool TelemetrySendTask::handle(glo_debug_message_t & message)
{
    message.valid = true;
    glo_debug_message.publish(&message, next_debug_instance_);
    bool success = this->send_copy(glo_debug_message.get_id(), next_debug_instance_);
    next_debug_instance_ = (next_debug_instance_ % glo_debug_message.get_num_instances()) + 1;
    return success;
}

//******************************************************************************
bool TelemetrySendTask::handle(glo_task_timing_t const & timing)
{
    glo_task_timing.publish(&timing);
    return this->send_copy(glo_task_timing.get_id());
}
