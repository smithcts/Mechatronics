// Includes
#include "crc.h"
#include "glo_rx_link.h"
#include "globs.h"
#include "telemetry_send_task.h"
#include "util_assert.h"

// Constructor
GloRxLink::GloRxLink(Usart * port, new_message_callback_t new_message_callback) :
    port_(port),
    new_message_callback_(new_message_callback),
    parse_state_(-1),
    num_body_bytes_(0),
    body_start_idx_(0),
    data_idx_(0),
    num_messages_received_(0),
    last_rx_packet_num_(0)
{
    resetParse();
}

//*****************************************************************************
void GloRxLink::parse(void)
{
    const uint8_t msg_start_byte = 0xFE;

    uint8_t in_byte = 0; // new byte read in from port
    bool complete_message_received = false; // true if received new glob

    if (port_ == NULL)
    {
        assert_always_msg(ASSERT_CONTINUE, "Parse failed due to null port.");
        return;
    }

    if (port_->getByte(&in_byte))
    {
        switch (parse_state_)
        {
            case -1:  // looking for start byte
                if (in_byte == msg_start_byte)
                {
                    message_data_[data_idx_++] = in_byte;
                    advanceParse();
                }
                break;
            case 0:  // pull out id (fall through)
            case 1:  // pull out byte indicating whether packet num and CRC are valid (fall through)
            case 2:  // pull out byte one of instance (fall through)
            case 3:  // pull out byte two of instance (fall through)
            case 4:  // pull out packet number
                message_data_[data_idx_++] = in_byte;
                advanceParse();
                break;
            case 5:  // pull out length of data
                num_body_bytes_ = in_byte;
                message_data_[data_idx_++] = in_byte;
                body_start_idx_ = data_idx_;
                advanceParse();
                if (num_body_bytes_ == 0)
                {
                    advanceParse(); // skip to getting CRC bytes
                }
                break;
            case 6:  // pull out body
                message_data_[data_idx_++] = in_byte;
                if ((data_idx_ - body_start_idx_) >= num_body_bytes_)
                {
                    advanceParse(); // received all body bytes
                }
                break;
            case 7: // pull out lower byte of CRC
                expected_crc1_ = in_byte;
                advanceParse();
                break;
            case 8: // pull out upper byte of CRC
                expected_crc2_ = in_byte;
                complete_message_received = true;
                // reset parse after handling new message
                break;
            default: // safety reset
                resetParse();
                break;
        }

        if (complete_message_received)
        {
            complete_message_received = false;

            uint8_t crc_reliable = message_data_[1];

            if (!crc_reliable || verifyCRC())
            {
                handleReceivedMessage();
            }

            resetParse();
        }
    }
}

//*****************************************************************************
bool GloRxLink::verifyCRC(void)
{
    uint16_t expected_crc = (uint16_t)expected_crc1_ + (uint16_t)(expected_crc2_ << 8);

    uint16_t actual_crc = calculate_crc(message_data_, data_idx_, 0xFFFF);

    bool crc_matches = (actual_crc == expected_crc);

    assert_msg(crc_matches, ASSERT_CONTINUE, "Bad CRC. Expected %x actual %x", expected_crc, actual_crc);

    return crc_matches;
}

//*****************************************************************************
void GloRxLink::handleReceivedMessage(void)
{
    uint8_t packet_num_reliable = message_data_[1];
    uint8_t object_id = message_data_[2];
    uint16_t instance = message_data_[3] + (uint16_t)(message_data_[4] << 8);
    uint8_t packet_num = message_data_[5];
    void * glob_data = message_data_ + body_start_idx_;

    if (!packet_num_reliable)
    {
        packet_num = last_rx_packet_num_ + 1;
    }

    if (new_message_callback_)
    {
        new_message_callback_(object_id, instance, glob_data);
    }

    num_messages_received_++;

    // Can't detect dropped packets unless we know when GUI first connects so we can
    // reset the number of message received.
//    if (num_messages_received_ > 1)
//    {
//        // Check for dropped packet's
//        uint8_t expected_packet_num = last_rx_packet_num_ + 1;
//        uint8_t num_dropped_messages = packet_num - expected_packet_num;
//        assert_msg(num_dropped_messages == 0, ASSERT_CONTINUE, "Eeva dropped %d packet(s) %d %d.", num_dropped_messages, packet_num, expected_packet_num);
//    }

    last_rx_packet_num_ = packet_num;

}

//*****************************************************************************
void GloRxLink::resetParse(void)
{
    body_start_idx_ = 0;
    data_idx_ = 0;
    parse_state_ = -1;
}

//*****************************************************************************
void GloRxLink::advanceParse(void)
{
    parse_state_++;
}
