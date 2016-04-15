#ifndef USER_LEDS_H_INCLUDED
#define USER_LEDS_H_INCLUDED

// Includes
#include "stm32f4xx.h"

// LEDs next to pushbuttons and USB port on Eeva.
typedef enum
{
    USER_LED_ORANGE,   // PC13
    USER_LED_YELLOW,   // PC14
    USER_LED_GREEN,    // PC15
    USER_LED_RED       // PH0
} user_led_id_t;

// Setup and control of User LEDs on Eeva
class UserLeds
{
  public: // methods

    // Constructor
    UserLeds(void);

    // Control the specified LED
    void set(user_led_id_t led);
    void clear(user_led_id_t led);
    void toggle(user_led_id_t led);
};

#endif  //USER_LEDS_H
