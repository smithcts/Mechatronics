// Includes
#include "user_leds.h"

//*****************************************************************************
UserLeds::UserLeds(void)
{
    // GPIOD Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    // Configure the GPIO_LED pins
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    set(USER_LED_ORANGE);
    set(USER_LED_YELLOW);
    set(USER_LED_RED);
    set(USER_LED_GREEN);
}

//*****************************************************************************
void UserLeds::set(user_led_id_t led)
{
    switch (led)
    {
        case USER_LED_ORANGE:
          GPIOC->BSRRH = GPIO_Pin_13;
          break;
        case USER_LED_YELLOW:
          GPIOC->BSRRH = GPIO_Pin_14;
          break;
        case USER_LED_GREEN:
          GPIOC->BSRRH = GPIO_Pin_15;
          break;
        case USER_LED_RED:
          GPIOE->BSRRH = GPIO_Pin_1;
          break;
    }
}

//*****************************************************************************
void UserLeds::clear(user_led_id_t led)
{
    switch (led)
    {
        case USER_LED_ORANGE:
          GPIOC->BSRRL = GPIO_Pin_13;
          break;
        case USER_LED_YELLOW:
          GPIOC->BSRRL = GPIO_Pin_14;
          break;
        case USER_LED_GREEN:
          GPIOC->BSRRL = GPIO_Pin_15;
          break;
        case USER_LED_RED:
          GPIOE->BSRRL = GPIO_Pin_1;
          break;
    }
}

//*****************************************************************************
void UserLeds::toggle(user_led_id_t led)
{
    switch (led)
    {
        case USER_LED_ORANGE:
          GPIOC->ODR ^= GPIO_Pin_13;
          break;
        case USER_LED_YELLOW:
          GPIOC->ODR ^= GPIO_Pin_14;
          break;
        case USER_LED_GREEN:
          GPIOC->ODR ^= GPIO_Pin_15;
          break;
        case USER_LED_RED:
          GPIOE->ODR ^= GPIO_Pin_1;
          break;
    }
}
