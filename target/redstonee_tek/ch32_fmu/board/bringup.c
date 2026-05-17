#include "ch32h417_pwr.h"
#include "core_riscv.h"
#include "debug.h"

extern uint32_t _start_v5f;

__attribute__((section(".init_v3f.bringup"))) int bringup(void)
{
    SystemInit();
    SystemAndCoreClockUpdate();
    initDebugUART();

    NVIC_WakeUp_V5F((uint32_t)&_start_v5f); // wake up V5
    PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFE);

    while (1)
        ;
}
