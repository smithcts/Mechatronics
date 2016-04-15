// Includes
#include "stm32f4xx.h"
#include "rc_servo_timer.h"
#include "math_util.h"

//*****************************************************************************
RcServoTimer::RcServoTimer(void)
{
    // Set the variables below to try to optimize the prescalar and
    // auto reload register. We want a large auto reload register for good
    // resolution on the timer, so this usually means finding a small
    // divisor of the input_clock_frequency that doesn't require an
    // auto reload register greater than 16 or 32 bits (depends on timer).
    // The timer freq = timer_input_clock / ((prescalar +1)*(auto_reload_reg +1))

    // Assuming a 72Mhz timer input clock the prescalar = 143, and
    // auto_reload_reg = 9999 gives a timer frequency of 50 Hz

    uint16_t prescalar = 143;
    auto_reload_reg_ = 9999;
    frequency_ = 72000000/(prescalar+1)/(auto_reload_reg_+1);

    // TIM9 clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

    // GPIOE clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    // GPIO Configuration: TIM9 CH (PE6)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // Connect TIM9 pins using alternate functions
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

    // Time base configuration
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = auto_reload_reg_;
    TIM_TimeBaseStructure.TIM_Prescaler = prescalar;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

    // PWM Mode configuration
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    // Channel 2
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM9, ENABLE);

    // Enable counter
    TIM_Cmd(TIM9, ENABLE);
}

//*****************************************************************************
void RcServoTimer::setPulse(float pulse_width_ms)
{
    pulse_width_ms = limit(pulse_width_ms, 0.75f, 2.25f);
    uint32_t ccr = (uint32_t)(pulse_width_ms/1000.0f * frequency_ * auto_reload_reg_);

    TIM9->CCR2 = ccr;
}
