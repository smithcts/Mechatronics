// Includes
#include <cstdint>
#include <cstdio>
#include "spi.h"
#include "util_assert.h"

#define SPI_MAX_ATTEMPTS (100) // Timeout count until bus timeout

#define SPI3_SCLK_GPIO_PORT (GPIOC               )
#define SPI3_SCLK_GPIO_CLK  (RCC_AHB1Periph_GPIOC)
#define SPI3_SCLK_GPIO_PIN  (GPIO_Pin_10         )
#define SPI3_SCLK_AF_PIN    (GPIO_PinSource10    )

#define SPI3_MISO_GPIO_PORT (GPIOC               )
#define SPI3_MISO_GPIO_CLK  (RCC_AHB1Periph_GPIOC)
#define SPI3_MISO_GPIO_PIN  (GPIO_Pin_11         )
#define SPI3_MISO_AF_PIN    (GPIO_PinSource11    )

#define SPI3_MOSI_GPIO_PORT (GPIOC               )
#define SPI3_MOSI_GPIO_CLK  (RCC_AHB1Periph_GPIOC)
#define SPI3_MOSI_GPIO_PIN  (GPIO_Pin_12         )
#define SPI3_MOSI_AF_PIN    (GPIO_PinSource12    )

// Static class variable definitions
bool SPI::init[SPI_BUS_COUNT];
SPI  SPI::objs[SPI_BUS_COUNT];

//*****************************************************************************
SPI * SPI::instance(spi_bus_id_t bus)
{
    if (bus >= SPI_BUS_COUNT)
    {
        assert_always_msg(ASSERT_CONTINUE, "Invalid SPI bus ID");
        return NULL;
    }

    if (init[bus])
    {
        return &objs[bus];
    }

    switch (bus)
    {
        case SPI_BUS_1:
            return NULL;

        case SPI_BUS_2:
            return NULL;

        case SPI_BUS_3:
            objs[bus].base_ = SPI3;

            // Deselect and Initialize GPIO chip select
            GPIO_InitTypeDef GPIO_InitStructure;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIOA->BSRRL = GPIO_Pin_15;

            // Enable Required clocks for SPI
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
            RCC_AHB1PeriphClockCmd(SPI3_SCLK_GPIO_CLK, ENABLE);
            RCC_AHB1PeriphClockCmd(SPI3_MISO_GPIO_CLK, ENABLE);
            RCC_AHB1PeriphClockCmd(SPI3_MOSI_GPIO_CLK, ENABLE);

            // Set GPIO Alternate Function for MOSI, MISO, SCLK
            GPIO_PinAFConfig(SPI3_SCLK_GPIO_PORT, SPI3_SCLK_AF_PIN, GPIO_AF_SPI3);
            GPIO_PinAFConfig(SPI3_MOSI_GPIO_PORT, SPI3_MOSI_AF_PIN, GPIO_AF_SPI3);
            GPIO_PinAFConfig(SPI3_MISO_GPIO_PORT, SPI3_MISO_AF_PIN, GPIO_AF_SPI3);

            // Configure GPIO for MOSI, MISO, SCLK
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
            GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

            // SPI SCK pin configuration
            GPIO_InitStructure.GPIO_Pin = SPI3_SCLK_GPIO_PIN;
            GPIO_Init(SPI3_SCLK_GPIO_PORT, &GPIO_InitStructure);

            // SPI MOSI pin configuration
            GPIO_InitStructure.GPIO_Pin =  SPI3_MOSI_GPIO_PIN;
            GPIO_Init(SPI3_MOSI_GPIO_PORT, &GPIO_InitStructure);

            // SPI MISO pin configuration
            GPIO_InitStructure.GPIO_Pin =  SPI3_MISO_GPIO_PIN;
            GPIO_Init(SPI3_MISO_GPIO_PORT, &GPIO_InitStructure);

            // Configure SPI port
            SPI_I2S_DeInit(SPI3);
            SPI_InitTypeDef SPI_InitStructure;
            SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
            SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
            SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
            SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
            SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
            SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
            SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // set below by setFrequency
            SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
            SPI_Init(SPI3, &SPI_InitStructure);

            objs[bus].setFrequency(DEFAULT_SPI_FREQUENCY);

            SPI_Cmd(SPI3, ENABLE);

            // Set bus as initialized and return SPI object
            init[bus] = true;
            return(&objs[bus]);

        case SPI_BUS_4:
            return NULL;

    } // end of 'switch (bus)'

    return NULL; // Invalid BUS ID - should never reach here as it's checked above

}

//*****************************************************************************
void SPI::setFrequency(uint32_t frequency)
{
    // Obtain PCLK2
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    uint32_t pclk = RCC_Clocks.PCLK2_Frequency;

    // Get current CR1 settings
    uint16_t reg = base_->CR1 & ~(0x0038);

    // Find closest frequency that does not exceed the given frequency
    if (frequency >= (pclk >> 1))
    {
        reg |= (0x00 << 3);
    }
    else if (frequency >= (pclk >> 2))
    {
        reg |= (0x01 << 3);
    }
    else if (frequency >= (pclk >> 3))
    {
        reg |= (0x02 << 3);
    }
    else if (frequency >= (pclk >> 4))
    {
        reg |= (0x03 << 3);
    }
    else if (frequency >= (pclk >> 5))
    {
        reg |= (0x04 << 3);
    }
    else if (frequency >= (pclk >> 6))
    {
        reg |= (0x05 << 3);
    }
    else if (frequency >= (pclk >> 7))
    {
        reg |= (0x06 << 3);
    }
    else if (frequency >= (pclk >> 8))
    {
        reg |= (0x07 << 3);
    }
    else
    {
        assert_always_msg(ASSERT_CONTINUE, "[SPI]: Invalid frequency attempted to be set.");
        return;
    }

    base_->CR1 = reg;

}

//*****************************************************************************
uint8_t SPI::sendByte(uint8_t byte_to_send)
{
    uint32_t attempts = 0;

    // Loop while DR register in not empty
    while (SPI_I2S_GetFlagStatus(base_, SPI_I2S_FLAG_TXE) == RESET)
    {
        attempts++;
        if (attempts >= SPI_MAX_ATTEMPTS)
        {
            // TODO: Reset SPI, possibly not just return 0
            //assert_always_msg(ASSERT_CONTINUE, "[SPI] bus timeout on TX.");
            return 0;
        }
    }

    // Send byte through the SPI peripheral
    SPI_I2S_SendData(base_, byte_to_send);

    // Wait to receive a byte
    attempts = 0;
    while (SPI_I2S_GetFlagStatus(base_, SPI_I2S_FLAG_RXNE) == RESET)
    {
        attempts++;
        if (attempts >= SPI_MAX_ATTEMPTS)
        {
            // TODO: Reset SPI, possibly not just return 0
            //assert_always_msg(ASSERT_CONTINUE, "[SPI] bus timeout on RX.");
            return 0;
        }
    }

    // Return the byte read from the SPI bus
    return (SPI_I2S_ReceiveData(base_) & 0x00FF);

}
