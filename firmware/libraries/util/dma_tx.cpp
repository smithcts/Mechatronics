// Includes
#include <stdio.h>
#include <string.h>
#include "debug_printf.h"
#include "dma_tx.h"
#include "stm32f4xx.h"
#include "util_assert.h"

//*****************************************************************************
DmaTx::DmaTx
    (
        DMA_Stream_TypeDef * dma_stream,            // DMAy_StreamX where y[1:2] and X[0:7]
        uint32_t             channel,               // Channel associated with stream.
        IRQn                 dma_irq_num,           // Interrupt request number
        uint32_t             periph_base_address,   // Base data address of peripheral.
        uint32_t             transfer_complete_bit, // DMA_IT_TCIFx where x is the Stream number
        uint32_t             transfer_error_bit,    // DMA_IT_TEIFx where x is the Stream number
        uint32_t             buff_length            // Length of memory buffer needed.
    )
{
    buff_length_ = buff_length;
    buff_top_ = 0;
    dma_top_ = 0;
    dma_active_ = false;
    transfer_complete_bit_ = transfer_complete_bit;
    transfer_error_bit_ = transfer_error_bit;
    error_count_ = 0;
    dma_irq_num_ = dma_irq_num;
    dma_stream_ = dma_stream;

    buff_ = new uint8_t[buff_length];
    assert(buff_ != NULL, ASSERT_STOP);

    // Enable DMA clock
    // TODO: Selectively enable clocks based on what stream is used.
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(dma_stream_);
    DMA_InitStructure.DMA_Channel = channel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)periph_base_address;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0; // Gets set before each transfer.
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 0; // Gets set before each transfer.
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(dma_stream_, &DMA_InitStructure);

    // Enable Tx DMA transfer complete and transfer error.
    DMA_ITConfig(dma_stream_, DMA_IT_TC|DMA_IT_TE, ENABLE);

    DMA_Cmd(dma_stream_, ENABLE);
}

//*****************************************************************************
DmaTx::~DmaTx(void)
{
    if (buff_ != NULL)
    {
        delete[] buff_;
    }
}

//*****************************************************************************
bool DmaTx::sendBuffer(uint8_t const * data, uint16_t len)
{
    if ((len > buff_length_) || (len == 0))
    {
        return false;
    }

    // Turn off the DMA interrupt since will be messing with transfer buffer.
    // **Note DMA may still be running.
    NVIC_DisableIRQ(dma_irq_num_);

    uint16_t start, end;
    bool copy_successful = copyToTransferBuffer(data, len, &start, &end);

    if (copy_successful && !dma_active_)
    {
        activateDMATransfer(start);
    }

    NVIC_EnableIRQ(dma_irq_num_);

    return copy_successful;
}

//*****************************************************************************
bool DmaTx::copyToTransferBuffer
    (
         uint8_t  const * data,   // Buffer containing new data.
         uint16_t         len,    // How many bytes to transfer.
         uint16_t       * start,  // Start index of next transfer.
         uint16_t       * end     // Ending index of next transfer.
    )
{
    uint16_t num_in_buff = 0;  // How many bytes are already in transfer buffer.

    // Default start and end indices assuming will fit.
    *start = buff_top_ + 1;
    *end   = buff_top_ + len;

    if (dma_active_)
    {
        // Find the space in the buffer.
        uint16_t num_data_left_to_transfer = dma_stream_->NDTR;
        if (dma_top_ > buff_top_)
        {
            num_in_buff = buff_top_ + buff_length_ + num_data_left_to_transfer - dma_top_;
        }
        else
        {
            num_in_buff = buff_top_ - dma_top_ + num_data_left_to_transfer;
        }
    }
    else
    {
        // Reset to bottom to minimize the rollovers.
        num_in_buff = buff_top_ = dma_top_ = *start = 0;
        *end = len - 1;
    }

    // Check for enough room in the dma buffer.
    // The -1 is to prevent a very rare edge case where there is just enough room in top
    // of buffer and the previous copy had split, putting part on top and part on bottom
    // -- and -- the dma has just finished transferring the last byte on the top.
    if (len > (buff_length_ - num_in_buff - 1))
    {
        return false; // won't fit in buffer.
    }

    if (*start == buff_length_)
    {
        *start = 0;
        *end = len - 1;
        // Copy to beginning of buffer.
        memcpy(buff_, data, len);
    }
    else if (*end >= buff_length_)
    {
        *end -= buff_length_;
        // Need to copy to end of buffer and then copy the rest into the beginning.
        memcpy(buff_ + *start, data, buff_length_ - *start);
        memcpy(buff_, data + buff_length_ - *start, len + *start - buff_length_);
    }
    else
    {
        // Everything's fine.  Just copy into transfer buffer.
        memcpy(buff_ + *start, data, len);
    }

    buff_top_ = *end;

    return true;
}

//*****************************************************************************
bool DmaTx::activateDMATransfer(uint16_t start_index)
{
    uint16_t length = 0; // How many bytes to transfer.

    if (start_index > buff_top_)
    {
        // Buffer has rolled over, transfer the remaining top.
        length = buff_length_ - start_index;
        dma_top_ = buff_length_ - 1;
    }
    else
    {
        length = buff_top_ - start_index + 1;
        dma_top_ = buff_top_;
    }

    // Need to disable stream before can modify any configuration settings.
    DMA_Cmd(dma_stream_, DISABLE);

    // M0AR = Memory 0 address register
    // NDTR = Number data register
    dma_stream_->M0AR = (uint32_t)(buff_ + start_index);
    dma_stream_->NDTR = length;

    // Enable the DMA transfer stream and mark as active so if another client request
    // arrives before finished then won't restart in middle.
    // NOTE: Event flags must be cleared before DMA stream is enabled
    dma_active_ = true;
    DMA_ClearITPendingBit(dma_stream_, transfer_complete_bit_);
    DMA_ClearITPendingBit(dma_stream_, transfer_error_bit_);
    DMA_Cmd(dma_stream_, ENABLE);

    return true;
}

//*****************************************************************************
void DmaTx::handleISR(void)
{
    if (DMA_GetITStatus(dma_stream_, transfer_error_bit_))
    {
        // Clear transfer error interrupt pending bit
        DMA_ClearITPendingBit(dma_stream_, transfer_error_bit_);
        error_count_++;
        // TODO:  figure out why or reset the DMA controller?
    }

    if (DMA_GetITStatus(dma_stream_, transfer_complete_bit_))
    {
        // Transfer complete so clear interrupt pending bit.
        DMA_ClearITPendingBit(dma_stream_, transfer_complete_bit_);

        if (dma_top_ != buff_top_)
        {
            // There is more in the buffer that needs to be sent.
            activateDMATransfer((dma_top_ + 1) % buff_length_);
        }
        else
        {
            dma_active_ = false;
        }
    }
}
