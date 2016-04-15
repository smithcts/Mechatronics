#ifndef DMA_RX_H_INCLUDED
#define DMA_RX_H_INCLUDED

// Includes
#include "stm32f4xx.h"

// Wraps a circular buffer that works with direct memory access.  All the user has to do
// is instantiate this class and then read out the bytes when they are ready.
class DmaRx
{
public: // methods

    // Constructor - dynamically allocates buffer and sets up DMA hardware.
    DmaRx
        (
            DMA_Stream_TypeDef * dma_stream,           // DMAy_StreamX where y[1:2] and X[0:7]
            uint32_t             channel,              // Channel associated with stream.
            uint32_t             periph_base_address,  // Base data address of peripheral.
            uint32_t             buff_length           // Length of memory buffer needed.
        );

    // Destructor
    ~DmaRx(void);

    // Return true if there aren't any remaining elements in the DMA buffer.
    bool empty(void) const;

    // Get the next available byte from buffer. Returns false if nothing available.
    // Should be called at high enough rate that the circular buffer is not overwritten
    // by the DMA between clearing it.
    // 'byte' is a pointer to byte to be returned.
    bool getByte(uint8_t * byte);

private: // fields

    uint8_t  * buff_;        // Pointer to dynamically allocated DMA receive buffer.
    uint32_t   buff_length_; // Length of receive buffer.
    uint32_t   buff_bottom_; // Index to the next available byte in rx buffer.

    DMA_Stream_TypeDef * dma_stream_; // DMAy_StreamX where y[1:2] and X[0:7]

};

#endif
