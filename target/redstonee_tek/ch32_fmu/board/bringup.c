#include "ch32h417_gpio.h"
#include "ch32h417_pwr.h"
#include "core_riscv.h"

extern void _v5f_start(void);

void softDelay(uint32_t count)
{
    while (count--) {
        __asm__ volatile("nop");
    }
}

void bringup(void)
{
    SystemInit();
    SystemAndCoreClockUpdate();

    GPIO_InitTypeDef gInit = {
        .GPIO_Pin = GPIO_Pin_0,
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_Very_High
    };
    GPIO_Init(GPIOA, &gInit);

    NVIC_WakeUp_V5F((uint32_t)_v5f_start); // wake up V5
    // PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFE);
    while (1) {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        softDelay(1000000);
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        softDelay(1000000);
    }
}