#ifndef DIGITAL_IN_H_INCLUDED
#define DIGITAL_IN_H_INCLUDED

// Includes
#include "stm32f4xx_gpio.h"

// Names of supported input pins and their uses.
typedef enum
{
    PB1,  // left motor sign
    PC3,  // right motor sign
} digital_in_pin_t;

// Used to configure and read a digital input pin.
class DigitalIn
{
  public: // methods

    // Constructor - You can define the operating Pull-up/Pull down to be:
    //  GPIO_PuPd_NOPULL
    //  GPIO_PuPd_UP
    //  GPIO_PuPd_DOWN
    explicit DigitalIn(digital_in_pin_t pin, GPIOPuPd_TypeDef pull_up_pull_down = GPIO_PuPd_NOPULL);

    // Return true if input pin is in a low state.
    bool read(void) const { return ((port_->IDR & gpio_pin_) == gpio_pin_); }

  private: // fields

    // Individual pin ID (e.g. GPIO_Pin_1)
    uint16_t gpio_pin_;

    // Bus peripheral for pin port. (e.g RCC_AHB1Periph_GPIOB)
    uint32_t rcc_ahb1periph_;

    // Port that pin belongs to. (e.g. GPIOB)
    GPIO_TypeDef * port_;

};

#endif
