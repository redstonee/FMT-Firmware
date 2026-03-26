#include "ch32h417_gpio.h"
#include "ch32h417_pwr.h"
#include "core_riscv.h"

__attribute__((section(".bringup"))) void softDelay(uint32_t count)
{
    while (count--) {
        __asm__ volatile("nop");
    }
}

__attribute__((section(".bringup"))) void bringup(void)
{
    SystemInit();

    GPIO_InitTypeDef gInit = {
        .GPIO_Pin = GPIO_Pin_0,
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_Very_High
    };
    GPIO_Init(GPIOA, &gInit);

    // NVIC_WakeUp_V5F(V5F_StartAddr); // wake up V5
    // PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFE);
    while (1) {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        softDelay(1000000);
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        softDelay(1000000);
    }
}