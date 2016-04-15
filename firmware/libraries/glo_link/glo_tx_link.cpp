// Includes
#include "crc.h"
#include "glo_tx_link.h"
#include "globs.h"
#include "util_assert.h"

//*****************************************************************************
// Constructor
GloTxLink::GloTxLink(Usart * port) :
    port_(port),
    num_messages_sent_(0),
    num_messages_failed_(0),
    next_packet_num_(0)
{
}

//*****************************************************************************
uint8_t GloTxLink::send(uint8_t glob_id, uint16_t glob_instance, void * data_buffer)
{
    assert(port_ != NULL, ASSERT_STOP);

    if (glob_id >= NUM_GLOBS)
    {
        num_messages_failed_++;
        assert_always_msg(ASSERT_CONTINUE, "Can't send glob %d because highest glob ID is %d", (int)glob_id, (int)NUM_GLOBS-1);
        return SEND_ERROR_BAD_ID;
    }

    GlobBase * glob = globs[glob_id];

    uint8_t num_data_bytes = glob->get_num_bytes();

    const uint16_t num_header_bytes = 7;
    send_buffer_[0] = 0xFE; // message start byte
    send_buffer_[1] = 1;    // non-zero since packet number and CRC are valid.
    send_buffer_[2] = glob_id;
    send_buffer_[3] = (uint8_t)glob_instance;
    send_buffer_[4] = (uint8_t)(glob_instance >> 8);
    send_buffer_[5] = next_packet_num_;
    send_buffer_[6] = num_data_bytes;

    if (data_buffer == NULL)
    {
        glob->copy_to_buffer(send_buffer_ + num_header_bytes, glob_instance);
    }
    else
    {
        memcpy(send_buffer_ + num_header_bytes, data_buffer, num_data_bytes);
    }

    uint16_t footer_start = num_data_bytes + num_header_bytes;

    uint16_t crc = calculate_crc(send_buffer_, footer_start, 0xFFFF);

    const uint16_t num_footer_bytes = 2;
    send_buffer_[footer_start]   = (uint8_t)crc;
    send_buffer_[footer_start+1] = (uint8_t)(crc >> 8);

    uint16_t packet_size = num_data_bytes + num_header_bytes + num_footer_bytes;

    if (!port_->sendBuffer(send_buffer_, packet_size))
    {
        num_messages_failed_++;
        return SEND_ERROR_NO_ROOM;
    }

    num_messages_sent_++;
    next_packet_num_++;

    return SEND_SUCCESS;
}
