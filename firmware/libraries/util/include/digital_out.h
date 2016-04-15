#ifndef DIGITAL_OUT_H_INCLUDED
#define DIGITAL_OUT_H_INCLUDED

// Includes
#include "stm32f4xx_gpio.h"

// Names of supported output pins and their uses.
typedef enum
{
    // These next for pins are used by the TB6612FNG (hbridge)
    PE13,
    PE14,
    PE15,
    PB10,

    // These next pins are defined for debugging purposes (e.g. function profiling)
    PB4,
    PB5,
    PB8,
    PB9,

} digital_out_pin_t;

// Used to configure and control a digital output pin.
class DigitalOut
{
  public: // methods

    // Constructor
    // You can define the 'out_type' to be:
    //  GPIO_OType_PP
    //  GPIO_OType_OD
    // You can define the 'pull_up_pull_down' to be:
    //  GPIO_PuPd_NOPULL
    //  GPIO_PuPd_UP
    //  GPIO_PuPd_DOWN
    // The 'initial_state' is 0 if the pin should be low to start out.
    DigitalOut(digital_out_pin_t pin,
               GPIOOType_TypeDef out_type = GPIO_OType_PP,
               GPIOPuPd_TypeDef pull_up_pull_down = GPIO_PuPd_NOPULL,
               uint8_t initial_state = 0);

    // Return true if pin is in a low state.
    bool read(void) const {return ((port_->ODR & gpio_pin_) == gpio_pin_); }

    // Set output pin high or low.
    void set(void) { port_->BSRRL = gpio_pin_; }
    void clear(void) { port_->BSRRH = gpio_pin_; }

  private: // fields

    // Individual pin ID (e.g. GPIO_Pin_1)
    uint16_t gpio_pin_;

    // Bus peripheral for pin port. (e.g RCC_AHB1Periph_GPIOB)
    uint32_t rcc_ahb1periph_;

    // Port that pin belongs to. (e.g. GPIOB)
    GPIO_TypeDef * port_;
};

#endif
