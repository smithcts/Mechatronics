//-------------------------------------------------------------------------
//
//-------------------------------------------------------------------------
#include "user_pb.h"
#include "stm32f4xx.h"

//******************************************************************************
UserPushButton::UserPushButton(user_button_id_t button_id) :
    button_id_(button_id),
    button_was_pressed_(false)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if (button_id == USER_PB_TOP)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
    else if (button_id == USER_PB_BOTTOM)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_Init(GPIOE, &GPIO_InitStructure);
    }
 }

//******************************************************************************
bool UserPushButton::read(void)
{
    if (button_id_ == USER_PB_TOP)
    {
        return !(bool)(GPIOB->IDR & GPIO_Pin_2);
    }
    if (button_id_ == USER_PB_BOTTOM)
    {
        return !(bool)(GPIOE->IDR & GPIO_Pin_0);
    }

    return false; // shouldn't reach here
}

//******************************************************************************
bool UserPushButton::activated(void)
{
    bool button_activated = false;

    bool is_pressed = read();
    if (!is_pressed and button_was_pressed_)
    {
        button_activated = true; // button was just released
    }

    button_was_pressed_ = is_pressed;

    return button_activated;
}
