/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-09-09     WCH        the first version
 * 2023-01-04     WangShun   Remove redundant files
 */
#include "rtconfig.h"
#include "ch32h417.h"
#include "core_riscv.h"
#include "debug.h"

void rt_trigger_software_interrupt(void)
{
    debug_print("triggering software interrupt\n");
    NVIC_SetPendingIRQ(Software_IRQn);
}

void rt_hw_do_after_save_above(void)
{
    __asm volatile ("li t0,0x20" );
    __asm volatile ("csrs 0x804, t0");
    debug_print("clearing software interrupt\n");
    NVIC_ClearPendingIRQ(Software_IRQn);
}