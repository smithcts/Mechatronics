// Includes
#include "stm32f4xx.h"
#include "system_timer.h"
#include "scheduler.h"

//*****************************************************************************
SystemTimer::SystemTimer(void) :
    rollover_count_(0),
    last_reported_ticks_(0)
{
    // Request the clock speed of the systick timer so we know many ticks are in each second.
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    timer_frequency_ = RCC_Clocks.HCLK_Frequency;

    // Systick reload value since it's a 24 bit timer.
    reload_value_ = 0xFFFFFF;

    // Pre-compute to make calculating current time faster.
    seconds_per_tick_ = 1.0 / timer_frequency_;

    // Tell timer to interrupt after it has count down to zero.
    // This also enables the timer.
    uint32_t tick_between_interrupts = reload_value_;
    uint32_t config_error = SysTick_Config(tick_between_interrupts);
    if (config_error != 0)
    {
        while (true); // Error configuring timer.
    }

    // Set to highest priority so can use time references in interrupt handlers.
    NVIC_SetPriority(SysTick_IRQn, 0x00);
}

//*****************************************************************************
extern "C" void SysTick_Handler(void)
{
    sys_timer.rollover_count_++;
}

//******************************************************************************
uint64_t SystemTimer::ticks(void)
{
    // Need to make sure that the reported ticks are always increasing.  If we
    // detect they didn't increase then the timer rolled over, but the interrupt
    // we have setup hasn't incremented the rollover count yet... so do it ourselves
    // by adding 1 to the rollover count.  Need to disable interrupts because the
    // timer update interrupt could occur between the two calculations of current_ticks
    // which would essentially make us increase the rollover count one too much.
    bool interrupts_enabled = scheduler.disableInterrupts();

    // Need to subtract current ticks from reload value since systick is a count-down timer.
    uint64_t current_ticks = (rollover_count_*reload_value_) + (reload_value_ - SysTick->VAL);

    if (current_ticks < last_reported_ticks_)
    {
        current_ticks = ((rollover_count_+1)*reload_value_) + (reload_value_ - SysTick->VAL);
    }

    scheduler.restoreInterrupts(interrupts_enabled);

    last_reported_ticks_ = current_ticks;

    return current_ticks;
}

//******************************************************************************
void SystemTimer::busyWait(double seconds_to_wait)
{
    double start_seconds = this->seconds();
    while (this->seconds() < (start_seconds + seconds_to_wait)) {}
}
