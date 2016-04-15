#ifndef GLO_TX_LINK_H_INCLUDED
#define GLO_TX_LINK_H_INCLUDED

// Includes
#include <cstdint>
#include <cstdio>
#include "usart.h"

// IDs for possible outcomes of sending a message.
enum
{
    SEND_SUCCESS,
    SEND_ERROR_BAD_ID,
    SEND_ERROR_NO_ROOM,
};

// Transmit globs over serial port.
class GloTxLink
{
  public: // methods

    // Constructor
    explicit GloTxLink(Usart * port);

    // Copy glob information and data that is associated with ID and instance
    // into transfer buffer. If 'data_buffer' is specified than will use that as
    // glob data, otherwise will use what's currently stored in the glob.
    // Return true if successful.
    uint8_t send(uint8_t id, uint16_t instance, void * data_buffer=NULL);

    void set_port(Usart * new_port) { port_ = new_port; }

  private: // fields

    // Serial port that data gets sent/received over
    Usart * port_;

    uint32_t num_messages_sent_;
    uint32_t num_messages_failed_;

    // incrementing number from 0 to 255 that's used to detect dropped packets
    uint8_t next_packet_num_;

    // Make larger than necessary to protect against future changes.
    uint8_t send_buffer_[300];

};

#endif

