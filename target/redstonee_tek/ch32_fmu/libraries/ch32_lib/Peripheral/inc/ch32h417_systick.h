/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32h417_exti.h
 * Author             : RedStonee
 * Version            : V1.0.0
 * Date               : 2025/03/01
 * Description        : This file contains all the functions prototypes for the
 *                      SysTick firmware library.
 *********************************************************************************/

#ifndef __CH32H417_SYSTICK_H
#define __CH32H417_SYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32h417.h"

typedef enum {
    SysTick_ClockSource_HCLK_Div8,
    SysTick_ClockSource_HCLK
} SysTick_ClkSrc_TypeDef;

typedef enum {
    SysTick_Count_Up,
    SysTick_Count_Down,
} SysTick_CntDir_TypeDef;

static inline void SysTick_SetEnable(SysTick_Type* SysTickx, FunctionalState NewState)
{
    SysTickx->CTLR = (SysTickx->CTLR & ~(1 << 0)) | (NewState << 0);
}

static inline void SysTick_SetITEnable(SysTick_Type* SysTickx, FunctionalState NewState)
{
    SysTickx->CTLR = (SysTickx->CTLR & ~(1 << 1)) | (NewState << 1);
}

static inline void SysTick_SetClockSource(SysTick_Type* SysTickx, SysTick_ClkSrc_TypeDef ClockSource)
{
    SysTickx->CTLR = (SysTickx->CTLR & ~(1 << 2)) | (ClockSource << 2);
}

static inline void SysTick_SetAutoReload(SysTick_Type* SysTickx, FunctionalState NewState)
{
    SysTickx->CTLR = (SysTickx->CTLR & ~(1 << 3)) | (NewState << 3);
}

static inline void SysTick_SetCountDirection(SysTick_Type* SysTickx, SysTick_CntDir_TypeDef CountDirection)
{
    SysTickx->CTLR = (SysTickx->CTLR & ~(1 << 4)) | (CountDirection << 4);
}


static inline void SysTick_Config(SysTick_Type* SysTickx, uint32_t ticks)
{
    SysTickx->CMP = ticks - 1;
    SysTickx->CNT = 0;
    if (SysTickx == SysTick0) /* set reload register */
    {
        NVIC_SetPriority(SysTick0_IRQn, (1 << 4) - 1); /* set Priority for Systick Interrupt */
        NVIC_EnableIRQ(SysTick0_IRQn);
    } else if (SysTickx == SysTick1) {
        NVIC_SetPriority(SysTick1_IRQn, (1 << 4) - 1); /* set Priority for Systick Interrupt */
        NVIC_EnableIRQ(SysTick1_IRQn);
    }

    // Enable SysTick IRQ and SysTick Timer, upcounting, auto-reload, HCLK as clock source
    SysTickx->CTLR = 0b1111;
}

#ifdef __cplusplus
}
#endif

#endif
