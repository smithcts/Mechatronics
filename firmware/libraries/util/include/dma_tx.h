#ifndef DMA_TX_H_INCLUDED
#define DMA_TX_H_INCLUDED

// Includes
#include "stm32f4xx.h"

// Wraps a buffer that works with direct memory access. Once that user places data into
// the buffer it will be sent over the configured stream.
class DmaTx
{
public: // methods

    // Constructor - dynamically allocates buffer and sets up DMA hardware.
    // Note: The transfer complete/error bits are referenced from the transfer stream number NOT the peripheral number.
    DmaTx
        (
            DMA_Stream_TypeDef * dma_stream,            // DMAy_StreamX where y[1:2] and X[0:7]
            uint32_t             channel,               // Channel associated with stream.
            IRQn                 dma_irq_num,           // Interrupt request number
            uint32_t             periph_base_address,   // Base data address of peripheral.
            uint32_t             transfer_complete_bit, // DMA_IT_TCIFx where x is the Stream number
            uint32_t             transfer_error_bit,    // DMA_IT_TEIFx where x is the Stream number
            uint32_t             buff_length            // Length of memory buffer needed.
        );

    // Destructor
    ~DmaTx(void);

    // Copy 'len' bytes from array at 'data' to the transfer buffer.
    // Return false if not enough room in the buffer.
    // The buffer is cleared by DMA transfers set up in the tx DMA ISR.
    bool sendBuffer(uint8_t const * data, uint16_t len);

    // Check error and transfer complete bits and if there's more data to be sent then
    // calls activateDMATransfer().  This needs to be called by client whenever they
    // receive a TX ISR.
    void handleISR(void);

private: // methods

    // Attempt to place data into transfer buffer.  If not enough room then return false.
    bool copyToTransferBuffer
        (
            uint8_t  const * data,   // Buffer containing new data.
            uint16_t         len,    // How many bytes to transfer.
            uint16_t       * start,  // Start index of next transfer.
            uint16_t       * end     // Ending index of next transfer.
        );

    // Start new transfer at specified index of the transfer buffer.
    bool activateDMATransfer(uint16_t start_index);

private: // fields

    uint8_t  * buff_;        // Pointer to dynamically allocated DMA transfer buffer.
    uint32_t   buff_length_; // Length of transfer buffer.
    uint32_t   buff_top_;    // Index to the last byte in the tx buffer.
    uint32_t   dma_top_;     // Index to the last byte to be transferred by current dma cycle.
    bool       dma_active_;  // Indicates if a dma transfer is currently active.

    IRQn dma_irq_num_;                // Interrupt number.
    DMA_Stream_TypeDef * dma_stream_; // DMAy_StreamX where y[1:2] and X[0:7]

    uint32_t transfer_complete_bit_; // ISR bit (e.g. DMA_IT_TCIFx)
    uint32_t transfer_error_bit_;    // ISR bit (e.g. DMA_IT_TEIFx)
    uint32_t error_count_;           // How many errors have occurred.

};

#endif
