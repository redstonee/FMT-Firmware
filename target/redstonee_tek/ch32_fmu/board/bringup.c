#include "ch32h417_pwr.h"
#include "core_riscv.h"

#define CTLR_DS_MASK ((uint32_t)0xFFFFFFFE)

extern uint32_t _start_v5f;

__attribute__((section(".init_v3f.bringup"))) 
int bringup(void)
{
    SystemInit();
    SystemAndCoreClockUpdate();
    RCC->HB1PCENR |= RCC_HB1Periph_PWR;

    NVIC->WAKEIP[1] = (uint32_t)&_start_v5f;
    NVIC->SCTLR |= (1 << 5);

    PWR->CTLR &= CTLR_DS_MASK;

    NVIC->SCTLR |= (1 << 2);

    __asm volatile ("wfi");
}
