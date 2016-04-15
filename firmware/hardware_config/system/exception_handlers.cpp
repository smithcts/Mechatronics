
// Includes
#include "util_assert.h"
#include "stm32f4xx.h"

void setup_exception_handlers(void)
{
    // Cause 'Division by 0' and 'Alignment faults' to generate a Usage Fault
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk |
                SCB_CCR_UNALIGN_TRP_Msk;

    // Set the fault priorities pretty high, but not as high as usart/dma/systick priorities
    // so that we can still send out messages in the fault handlers.
    // Hard fault priority is fixed really high so not much we can do about that.
    NVIC_SetPriority(MemoryManagement_IRQn, 0x05);
    NVIC_SetPriority(BusFault_IRQn, 0x05);
    NVIC_SetPriority(UsageFault_IRQn, 0x05);

     // Enable usage, bus and memory faults.
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_MEMFAULTENA_Msk;
}

extern "C" {

void HardFault_Handler(void)
{
    // Pointless to do an assert here since this handler has such a high priority the message will never be sent.
    // This handler should only happen if one of the below handlers can't be executed.
    while (1);
}
void MemManage_Handler(void)
{
    assert_always_msg(ASSERT_STOP, "Memory Access Violation!");
}
void BusFault_Handler(void)
{
    assert_always_msg(ASSERT_STOP, "Bus Fault!");
}
void UsageFault_Handler(void)
{
    assert_always_msg(ASSERT_STOP, "Usage Fault! Examples are division by 0 or misaligned memory access.");
}

}
