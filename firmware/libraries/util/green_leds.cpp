// Includes
#include "green_leds.h"
#include "stm32f4xx_gpio.h"

//*****************************************************************************
GreenLeds::GreenLeds(void)
{
    // GPIOB and GPIOD Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // Configure the GPIO_LED pins
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    this->set(0);
}

//*****************************************************************************
void GreenLeds::set(uint8_t pattern)
{
    // handle bits 12 to 15 of GPIOB
    for (uint8_t i = 0; i < 4; i++)
    {
        if (pattern & (1 << i))
        {
            GPIOB->BSRRH = 1 << (i+12);   // clear the bit to turn on
        }
        else
        {
            GPIOB->BSRRL = 1 << (i+12);   // set the bit to turn off
        }
    }
    // handle bits 8 to 11 of GPIOD
    for (uint8_t i = 4; i < 8; i++)
    {
        if (pattern & (1 << i))
        {
            GPIOD->BSRRH = 1 << (i+4);   // clear the bit to turn on
        }
        else
        {
            GPIOD->BSRRL = 1 << (i+4);   // set the bit to turn off
        }
    }
}
