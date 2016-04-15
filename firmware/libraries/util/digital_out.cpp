// Includes
#include "digital_out.h"

//*****************************************************************************
DigitalOut::DigitalOut(digital_out_pin_t pin,
                       GPIOOType_TypeDef out_type,
                       GPIOPuPd_TypeDef pull_up_pull_down,
                       uint8_t initial_state)
{
    switch (pin)
    {
        case PE13:
            gpio_pin_ = GPIO_Pin_13;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOE;
            port_ = GPIOE;
            break;
        case PE14:
            gpio_pin_ = GPIO_Pin_14;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOE;
            port_ = GPIOE;
            break;
        case PE15:
            gpio_pin_ = GPIO_Pin_15;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOE;
            port_ = GPIOE;
            break;
        case PB10:
            gpio_pin_ = GPIO_Pin_10;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOB;
            port_ = GPIOB;
            break;
        case PB4:
            gpio_pin_ = GPIO_Pin_4;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOB;
            port_ = GPIOB;
            break;
        case PB5:
            gpio_pin_ = GPIO_Pin_5;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOB;
            port_ = GPIOB;
            break;
        case PB8:
            gpio_pin_ = GPIO_Pin_8;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOB;
            port_ = GPIOB;
            break;
        case PB9:
            gpio_pin_ = GPIO_Pin_9;
            rcc_ahb1periph_ = RCC_AHB1Periph_GPIOB;
            port_ = GPIOB;
            break;
    }

    // GPIOx Periph clock enable
    RCC_AHB1PeriphClockCmd(rcc_ahb1periph_, ENABLE);

    // Configure the GPIO pin
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = gpio_pin_;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = out_type;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = pull_up_pull_down;

    GPIO_Init(port_, &GPIO_InitStructure);

    if (initial_state)
    {
        set();
    }
    else
    {
        clear();
    }
}

