// Includes
#include "stm32f4xx.h"
#include "pwm_out_advanced_timer.h"
#include "math_util.h"
#include "util_assert.h"

//*****************************************************************************
PwmOutAdvancedTimer::PwmOutAdvancedTimer(void)
{
    // Set the variables below to try to optimize the prescalar and
    // auto reload register. We want a large auto reload register for good
    // resolution on the timer, so this usually means finding a small
    // divisor of the input_clock_frequency that doesn't require an
    // auto reload register greater than 16 or 32 bits (depends on timer).
    // The timer freq = timer_input_clock / ((prescalar +1)*(auto_reload_reg +1))

    // Assuming a 72Mhz timer input clock the prescalar = 0, and
    // auto_reload_reg = 3599 gives a timer frequency of 20kHz

    uint16_t prescalar = 0;
    auto_reload_reg_ =  3599;

    // TIM1 clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // GPIOA clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // GPIO Configuration: TIM1 CH1 (PA8), CH2 (PA9)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Connect TIM1 pins using alternate functions
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);

    // Time base configuration
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = auto_reload_reg_;
    TIM_TimeBaseStructure.TIM_Prescaler = prescalar;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

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
    // Channel 1
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // Channel 2
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // Configure automatic output state, break, dead time and lock configuration
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = 0;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    // Make sure the MOE bit is set to enable outputs on advanced timers.
    TIM1->BDTR |= 0x8000;

    // Enable counter
    TIM_Cmd(TIM1, ENABLE);
}

//*****************************************************************************
void PwmOutAdvancedTimer::setDuty(float duty, uint8_t channel_num)
{
    duty = limit(duty, 0.0f, 1.0f);
    uint32_t ccr = (uint32_t)(duty * auto_reload_reg_);
    switch (channel_num)
    {
        case 1:
            TIM1->CCR1 = ccr;
            break;
        case 2:
            TIM1->CCR2 = ccr;
            break;
        case 3:
            TIM1->CCR3 = ccr;
            break;
        case 4:
            TIM1->CCR4 = ccr;
            break;
        default:
            assert_always_msg(ASSERT_STOP, "Invalid CCR num for PWM output timer.");
            break;

    }
}
