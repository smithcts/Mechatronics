#ifndef ENCODER_H_INCLUDED
#define ENCODER_H_INCLUDED

// Includes
#include "stm32f4xx.h"

// IDs for possible setups for an encoder
typedef enum
{
    EncoderA,      // Uses PC6 and PC7 on TIM3 as encoder channels
    EncoderB       // Uses PD12 and PD13 on TIM4 as encoder channels
} encoder_id_t;

// Implement a quadrature encoder interface with 4x counting, and a signed 32 bit count.
class Encoder
{
  public: // methods

    // Constructor - setups up pin and timer hardware.
    explicit Encoder(encoder_id_t id);

    // Read the count and look for overflow/underflow
    int32_t read(void);

    // Set the current encoder count to a value
    void set(int32_t count32);

  private: // fields

    encoder_id_t encoder_id_;   // ID for what hardware is associated with encoder.
    uint16_t prev_counter_;     // counter value from previous read
    int16_t overflows_;         // number of overflows/underflows (for 32 bit count)
};

#endif
