#include "debug.h"
#include "ch32h417_gpio.h"
#include "ch32h417_rcc.h"
#include "ch32h417_usart.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

void initDebugUART(void)
{
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOA, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART6, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF8);

    GPIO_InitTypeDef GPIO_InitStructure = {
        .GPIO_Pin = GPIO_Pin_0,
        .GPIO_Speed = GPIO_Speed_Very_High,
        .GPIO_Mode = GPIO_Mode_AF_PP,
    };
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure = {
        .USART_BaudRate = 115200,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_1,
        .USART_Parity = USART_Parity_No,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
        .USART_Mode = USART_Mode_Tx,

    };

    USART_Init(USART6, &USART_InitStructure);
    USART_Cmd(USART6, ENABLE);
}

void debugUARTSend(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    char str[256];
    int len = vsnprintf(str, sizeof(str), fmt, args);
    for (int i = 0; i < len; i++) {
        while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
            ;
        USART_SendData(USART6, str[i]);
    }
    va_end(args);
}

void rt_hw_console_output(const char* str)
{
    debugUARTSend(str);
}

void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((section(".fast_code")));
void HardFault_Handler(void)
{
    uint32_t mcause, mepc, mtval, mstatus;
    mcause = __get_MCAUSE();
    mepc = __get_MEPC();
    mtval = __get_MTVAL();
    mstatus = __get_MSTATUS();

    debug_print("HARD!!!");
    debug_print("MCAUSE: %08x, MEPC: %08x, MTVAL: %08x, MSTATUS: %08x\n", mcause, mepc, mtval, mstatus);
    while (1) {
    }
}