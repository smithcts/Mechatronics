// Includes
#include "digital_in.h"
#include "util_assert.h"

//*****************************************************************************
DigitalIn::DigitalIn(digital_in_pin_t pin, GPIOPuPd_TypeDef pull_up_pull_down)
{
    switch (pin)
    {
        case PB1:
            gpio_pin_ = GPIO_Pin_1;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOB;
            port_ = GPIOB;
            break;
        case PC3:
            gpio_pin_ = GPIO_Pin_3;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOC;
            port_ = GPIOC;
            break;
        default:
            assert_always_msg(ASSERT_STOP, "Unsupported type for digital input pin.");
            break;
    }

    // GPIOx periph clock enable
    RCC_AHB1PeriphClockCmd(rcc_ahb1periph_, ENABLE);

    // Configure the GPIO pin
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = gpio_pin_;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      // doesn't matter - input
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // doesn't matter - input
    GPIO_InitStructure.GPIO_PuPd = pull_up_pull_down;

    GPIO_Init(port_, &GPIO_InitStructure);
}


