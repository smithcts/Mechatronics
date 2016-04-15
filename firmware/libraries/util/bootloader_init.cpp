// Includes
#include "bootloader_init.h"
#include <cstdint>
#include "stm32f4xx_rcc.h"


void bootloader_init(void)
{
    // For STM32F407VE
    //uint32_t ram_start = 0x20000000;
    //uint32_t ram_length = 0x20000; // 128K
    uint32_t flag_location = 0x2001FFF1;
    *((uint32_t *)flag_location) = 0xDEADBEEF;

    NVIC_SystemReset();
}

void bootloader_init_option2(bool using_usb)
{
    // Create function label to bootloader reset vector.
    // This is 4 bytes after the starting address of the bootloader since the first 4
    // bytes are the MSP value described below.
    typedef void (*boot_jump_t)(void);
    boot_jump_t sys_mem_boot_jump = (boot_jump_t)(*((uint32_t *) 0x1fff0004));

    // Change clock configuration to the default reset state.
    RCC_DeInit();

    // Reset the systick timer
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // Select HSI as the system clock source
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

    if (using_usb)
    {
        // USB needs interrupts enabled to work correctly.
        __set_PRIMASK(0);
    }
    else
    {
        // Not using USB so make sure interrupts are disabled since can't have
        // an interrupt when jumping to bootloader.
        __set_PRIMASK(1);
    }

    // Change main stack pointer to bootloader default value.
    // This can be found by analyzing the first word at the bootloader starting address.
    __set_MSP(0x20001D80);

    // Change the Program Counter to the bootloader reset vector.
    sys_mem_boot_jump();

    while (true);
}
