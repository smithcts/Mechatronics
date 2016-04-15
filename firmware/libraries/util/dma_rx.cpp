// Includes
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "dma_rx.h"
#include "util_assert.h"

//*****************************************************************************
DmaRx::DmaRx
    (
        DMA_Stream_TypeDef * dma_stream,           // DMAy_StreamX where y[1:2] and X[0:7]
        uint32_t             channel,              // Channel associated with stream.
        uint32_t             periph_base_address,  // Base data address of peripheral.
        uint32_t             buff_length           // Length of memory buffer needed.
    )
{
    buff_length_ = buff_length;
    buff_bottom_ = 0;
    dma_stream_ = dma_stream;
    buff_ = new uint8_t[buff_length];
    assert(buff_ != NULL, ASSERT_STOP);

    // Enable DMA clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(dma_stream);
    DMA_InitStructure.DMA_Channel = channel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)periph_base_address;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buff_;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = buff_length;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(dma_stream, &DMA_InitStructure);

    DMA_Cmd(dma_stream, ENABLE);
}


//*****************************************************************************
DmaRx::~DmaRx(void)
{
    if (buff_ != NULL)
    {
        delete[] buff_;
    }
}

//*****************************************************************************
bool DmaRx::empty(void) const
{
    // NDTR = Number of data items left to transfer register.
    uint32_t buff_top = buff_length_ - (uint16_t)dma_stream_->NDTR;

    // If top and bottom are at the same point then there's no elements in buffer.
    return (buff_top == buff_bottom_);
}

//*****************************************************************************
bool DmaRx::getByte(uint8_t * byte)
{
    if (empty()) { return false; }

    *byte = buff_[buff_bottom_];

    // Increment and wrap bottom index for next time.
    buff_bottom_ = (buff_bottom_ + 1) % buff_length_;

    return true;
}
