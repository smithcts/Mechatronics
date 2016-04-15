#ifndef GLO_RX_LINK_H_INCLUDED
#define GLO_RX_LINK_H_INCLUDED

#include <cstdint>
#include "usart.h"

typedef void (*new_message_callback_t)(uint8_t, uint16_t, void *);

// Receive data off a serial port and parse it into complete globs.  When the a
// complete glob is received it is passed off to its owner to handle it.
class GloRxLink
{
  public: // methods

    // Constructor
    explicit GloRxLink(Usart * port, new_message_callback_t new_message_callback);

    // Look for newly transmitted object information and progresses through
    // state machine.  Needs be called by a periodic task that runs at least
    // fast enough to handle data stream rate.  Currently this method only
    // processes a single byte each time it's called.  Can call dataReady()
    // to determine if need to parse more data.
    void parse(void);

    // Return true if there is data ready to be parsed.
    bool dataReady(void) const { return !port_->empty(); }

    void setPort(Usart * new_port) { port_ = new_port; }

  private: // methods

      void handleReceivedMessage(void);

      // Return true if actual CRC matches expected CRC stored in message.
      bool verifyCRC(void);

      void advanceParse(void);

      void resetParse(void);

  private: // fields

    // Serial port that data gets received over.
    Usart * port_;

    // Function to call when a new message is received
    new_message_callback_t new_message_callback_;

    // Message info that needs to be remembered when parsing.
    int8_t parse_state_;
    uint8_t num_body_bytes_;
    uint8_t body_start_idx_;
    uint8_t expected_crc1_;
    uint8_t expected_crc2_;

    // Data of entire message excluding checksum.
    // Larger than necessary to protect against future changes.
    uint8_t message_data_[300];

    // Current index to store next byte in 'message data'.
    uint16_t data_idx_;

    // How many complete messages have been parsed from serial port.
    uint32_t num_messages_received_;

    // Last received packet number.  Used to detect dropped packets.
    uint8_t last_rx_packet_num_;

};

#endif

