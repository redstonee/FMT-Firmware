/******************************************************************************
 * Copyright 2022 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <firmament.h>

#include "hal/rc/rc.h"
#include "hal/rc/sbus.h"

/* default config for rc device */
#define RC_CONFIG_DEFAULT                      \
    {                                          \
        RC_PROTOCOL_SBUS, /* auto */           \
        6,                /* 6 channel */      \
        0.05f,            /* sample time */    \
        1000,             /* minimal 1000us */ \
        2000,             /* maximal 2000us */ \
    }

// Not gonna support PPM
static sbus_decoder_t sbus_decoder;

void USART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USART4_IRQHandler(void)
{
    uint8_t ch;
    /* enter interrupt */
    rt_interrupt_enter();

    if (USART_GetITStatus(USART4, USART_IT_RXNE)) {
        while (USART_GetFlagStatus(USART4, USART_FLAG_RXNE) != RESET) {
            ch = (uint8_t)USART_ReceiveData(USART4);
            sbus_input(&sbus_decoder, &ch, 1);
        }

        /* if it's reading sbus data, we just parse it later */
        if (!sbus_islock(&sbus_decoder)) {
            sbus_update(&sbus_decoder);
        }
    }

    /* leave interrupt */
    rt_interrupt_leave();
}

static rt_err_t sbus_lowlevel_init(void)
{
    // PE3: INV_EN
    // PF3: RX
    // PF4: TX

    /* enable periph clock */
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART4, ENABLE);

    /* initialize gpio */
    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = GPIO_Pin_4,
        .GPIO_Speed = GPIO_Speed_Very_High,
        .GPIO_Mode = GPIO_Mode_AF_PP,
    };
    // GPIO_Init(GPIOF, &GPIO_InitStruct); // TX

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOF, &GPIO_InitStruct); // RX

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStruct); // INV_EN

    /* configure GPIO alternate function */
    // GPIO_PinAFConfig(GPIOF, GPIO_Pin_4, GPIO_AF7);
    GPIO_PinAFConfig(GPIOF, GPIO_Pin_3, GPIO_AF7);

    /* config usart */
    /* 100000bps, even parity, two stop bits */
    USART_InitTypeDef USART_InitStruct = {
        .USART_BaudRate = 100000,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_2,
        .USART_Parity = USART_Parity_Even,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
        .USART_Mode = USART_Mode_Rx,
    };

    GPIO_SetBits(GPIOE, GPIO_Pin_3); // set INV_EN high to invert signal

    USART_Init(USART4, &USART_InitStruct);
    USART_Cmd(USART4, ENABLE);

    /* initialize isr */
    NVIC_SetPriority(USART4_IRQn, 1);
    /* enable interrupt */
    NVIC_EnableIRQ(USART4_IRQn);

    return RT_EOK;
}

static rt_err_t rc_control(rc_dev_t rc, int cmd, void* arg)
{
    switch (cmd) {
    case RC_CMD_CHECK_UPDATE: {
        uint8_t updated = 0;

        if (rc->config.protocol == RC_PROTOCOL_SBUS) {
            updated = sbus_data_ready(&sbus_decoder);
        } else if (rc->config.protocol == RC_PROTOCOL_PPM) {
            updated = 0;
        }

        *(uint8_t*)arg = updated;
    } break;

    default:
        break;
    }

    return RT_EOK;
}

static rt_uint16_t rc_read(rc_dev_t rc, rt_uint16_t chan_mask, rt_uint16_t* chan_val)
{
    uint16_t* index = chan_val;
    rt_uint16_t rb = 0;

    if (rc->config.protocol == RC_PROTOCOL_PPM) {
        return 0;
    }

    if (sbus_data_ready(&sbus_decoder) == 0) {
        /* no data received, just return */
        return 0;
    }

    sbus_lock(&sbus_decoder);

    for (uint8_t i = 0; i < min(rc->config.channel_num, sbus_decoder.rc_count); i++) {
        *(index++) = sbus_decoder.sbus_val[i];
        rb += 2;
    }
    sbus_data_clear(&sbus_decoder);

    sbus_unlock(&sbus_decoder);

    return rb;
}

const static struct rc_ops rc_ops = {
    .rc_init = NULL,
    .rc_config = NULL,
    .rc_control = rc_control,
    .rc_read = rc_read,
};

static struct rc_device rc_dev = {
    .config = RC_CONFIG_DEFAULT,
    .ops = &rc_ops,
};

rt_err_t drv_rc_init(void)
{
    RT_TRY(sbus_lowlevel_init());
    RT_TRY(sbus_decoder_init(&sbus_decoder));

    RT_CHECK(hal_rc_register(&rc_dev, "rc", RT_DEVICE_FLAG_RDWR, NULL));

    return RT_EOK;
}