/******************************************************************************
 * Copyright 2020 The Firmament Authors.All Rights Reserved.
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

#include "drv_gpio.h"
#include "drv_usart.h"
#include "hal/serial/serial.h"

#define SET_BIT(REG, BIT)   ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)  ((REG) & (BIT))

#define UART_ENABLE_IRQ(n)  NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n) NVIC_DisableIRQ((n))

// #define USING_UART2         // ESC
#define USING_UART3         // GPS
// #define USING_UART4         // RC
#define USING_UART5         // MAVLINK
// #define USING_UART6         // Some Peripheral
#define USING_UART7         // Console

#define SERIAL_CONFIG_115200                   \
    {                                          \
        BAUD_RATE_115200, /* 115200 bits/s */  \
        DATA_BITS_8,      /* 8 databits */     \
        STOP_BITS_1,      /* 1 stopbit */      \
        PARITY_NONE,      /* No parity  */     \
        BIT_ORDER_LSB,    /* LSB first sent */ \
        NRZ_NORMAL,       /* Normal mode */    \
        SERIAL_RB_BUFSZ,  /* Buffer size */    \
        0                                      \
    }

static int usart_getc(struct serial_device* serial);

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*UART DMA request number*/
enum dma_request_num {
    DMA_REQUEST_USART1_TX = 85,
    DMA_REQUEST_USART1_RX,
    DMA_REQUEST_USART2_TX,
    DMA_REQUEST_USART2_RX,
    DMA_REQUEST_USART3_TX,
    DMA_REQUEST_USART3_RX,
    DMA_REQUEST_USART4_TX,
    DMA_REQUEST_USART4_RX,
    DMA_REQUEST_USART5_TX,
    DMA_REQUEST_USART5_RX,
    DMA_REQUEST_USART6_TX,
    DMA_REQUEST_USART6_RX,
    DMA_REQUEST_USART7_TX,
    DMA_REQUEST_USART7_RX,
    DMA_REQUEST_USART8_TX,
    DMA_REQUEST_USART8_RX
};

static const uint32_t _dma_mux_channel_to_TC[] = {
    DMA1_FLAG_TC1,
    DMA1_FLAG_TC2,
    DMA1_FLAG_TC3,
    DMA1_FLAG_TC4,
    DMA1_FLAG_TC5,
    DMA1_FLAG_TC6,
    DMA1_FLAG_TC7,
    DMA1_FLAG_TC8,
    DMA2_FLAG_TC1,
    DMA2_FLAG_TC2,
    DMA2_FLAG_TC3,
    DMA2_FLAG_TC4,
    DMA2_FLAG_TC5,
    DMA2_FLAG_TC6,
    DMA2_FLAG_TC7,
    DMA2_FLAG_TC8
};

static DMA_Channel_TypeDef* const _dma_mux_channel_to_channel[] = {
    DMA1_Channel1,
    DMA1_Channel2,
    DMA1_Channel3,
    DMA1_Channel4,
    DMA1_Channel5,
    DMA1_Channel6,
    DMA1_Channel7,
    DMA1_Channel8,
    DMA2_Channel1,
    DMA2_Channel2,
    DMA2_Channel3,
    DMA2_Channel4,
    DMA2_Channel5,
    DMA2_Channel6,
    DMA2_Channel7,
    DMA2_Channel8
};

static const IRQn_Type _dma_mux_channel_to_irqn[] = {
    DMA1_Channel1_IRQn,
    DMA1_Channel2_IRQn,
    DMA1_Channel3_IRQn,
    DMA1_Channel4_IRQn,
    DMA1_Channel5_IRQn,
    DMA1_Channel6_IRQn,
    DMA1_Channel7_IRQn,
    DMA1_Channel8_IRQn,
    DMA2_Channel1_IRQn,
    DMA2_Channel2_IRQn,
    DMA2_Channel3_IRQn,
    DMA2_Channel4_IRQn,
    DMA2_Channel5_IRQn,
    DMA2_Channel6_IRQn,
    DMA2_Channel7_IRQn,
    DMA2_Channel8_IRQn
};

static inline DMA_TypeDef* _dma_mux_channel_to_periph(uint8_t mux_ch)
{
    return (mux_ch <= DMA_MuxChannel8) ? DMA1 : DMA2;
}

/* CH32H41x uart driver */
struct ch32_uart {
    USART_TypeDef* uart_periph;
    IRQn_Type irqn;
    uint32_t clock;
    // GPIO clock already enabled by GPIO driver
    GPIO_TypeDef* tx_port;
    uint16_t tx_af;
    uint16_t tx_pin;
    GPIO_TypeDef* rx_port;
    uint16_t rx_af;
    uint16_t rx_pin;
    struct ch32_uart_dma {
        /* dma rx channel */
        uint8_t rx_mux_ch;
        /* dma rx request number */
        uint8_t rx_req_num;
        /* dma tx channel */
        uint8_t tx_mux_ch;
        /* dma tx request number */
        uint8_t tx_req_num;
        /* setting receive len */
        rt_size_t setting_recv_len;
        /* last receive index */
        rt_size_t last_recv_index;
    } dma;
};

/**
 * Serial port receive idle process. This need add to uart idle ISR.
 *
 * @param serial serial device
 */
static void dma_uart_rx_idle_isr(struct serial_device* serial)
{
    struct ch32_uart* uart = (struct ch32_uart*)serial->parent.user_data;
    DMA_Channel_TypeDef* dma_channel = _dma_mux_channel_to_channel[uart->dma.tx_mux_ch];
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();
    /* total received bytes */
    recv_total_index = uart->dma.setting_recv_len - DMA_GetCurrDataCounter(dma_channel);
    /* received bytes at this time */
    recv_len = recv_total_index - uart->dma.last_recv_index;
    /* update last received total bytes */
    uart->dma.last_recv_index = recv_total_index;
    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    if (recv_len) {
        /* high-level ISR routine */
        hal_serial_isr(serial, SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
    }
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_rx_done_isr(struct serial_device* serial)
{
    struct ch32_uart* uart = (struct ch32_uart*)serial->parent.user_data;
    rt_size_t recv_len;
    rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();
    /* received bytes at this time */
    recv_len = uart->dma.setting_recv_len - uart->dma.last_recv_index;
    /* reset last recv index */
    uart->dma.last_recv_index = 0;
    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    if (recv_len) {
        /* high-level ISR routine */
        hal_serial_isr(serial, SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
    }
}

/**
 * DMA transmit done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_tx_done_isr(struct serial_device* serial)
{
    /* high-level ISR routine */
    hal_serial_isr(serial, SERIAL_EVENT_TX_DMADONE);
}

static void uart_isr(struct serial_device* serial)
{
    struct ch32_uart* uart = (struct ch32_uart*)serial->parent.user_data;

    if (USART_GetITStatus(uart->uart_periph, USART_IT_RXNE) != RESET) {
        hal_serial_isr(serial, SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_periph, USART_IT_RXNE);
    }

    if (USART_GetITStatus(uart->uart_periph, USART_IT_IDLE) != RESET) {
        dma_uart_rx_idle_isr(serial);
        /* read a data for clear receive idle interrupt flag */
        USART_ReceiveData(uart->uart_periph);
    }

    if (USART_GetITStatus(uart->uart_periph, USART_IT_TC) != RESET) {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_periph, USART_IT_TC);
    }

    if (USART_GetFlagStatus(uart->uart_periph, USART_FLAG_ORE) == SET) {
        usart_getc(serial);
    }
}

#ifdef USING_UART2
static struct serial_device serial2;
static struct ch32_uart uart2 = {
    .uart_periph = USART2,
    .irqn = USART2_IRQn,
    .clock = RCC_HB1Periph_USART2,
    .tx_port = GPIOA,
    .tx_af = GPIO_AF7,
    .tx_pin = GPIO_Pin_2,
    .rx_port = GPIOA,
    .rx_af = GPIO_AF7,
    .rx_pin = GPIO_Pin_3,
    .dma = {
        .rx_mux_ch = DMA_MuxChannel1,
        .rx_req_num = DMA_REQUEST_USART2_RX,
        .tx_mux_ch = DMA_MuxChannel2,
        .tx_req_num = DMA_REQUEST_USART2_TX,
    }
};

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* uart isr routine */
    uart_isr(&serial2);
    /* leave interrupt */
    rt_interrupt_leave();
}
void DMA1_Channel1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (DMA_GetFlagStatus(DMA1, DMA1_FLAG_TC1)) {
        dma_rx_done_isr(&serial2);
        DMA_ClearFlag(DMA1, DMA1_FLAG_TC1);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (DMA_GetFlagStatus(DMA1, DMA1_FLAG_TC2)) {
        dma_tx_done_isr(&serial2);
        DMA_ClearFlag(DMA1, DMA1_FLAG_TC2);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* USING_UART0 */

#ifdef USING_UART3
static struct serial_device serial3;
static struct ch32_uart uart3 = {
    .uart_periph = USART3,
    .irqn = USART3_IRQn,
    .clock = RCC_HB1Periph_USART3,
    .tx_port = GPIOC,
    .tx_af = GPIO_AF7,
    .tx_pin = GPIO_Pin_10,
    .rx_port = GPIOC,
    .rx_af = GPIO_AF7,
    .rx_pin = GPIO_Pin_11,
    .dma = {
        .rx_mux_ch = DMA_MuxChannel3,
        .rx_req_num = DMA_REQUEST_USART3_RX,
        .tx_mux_ch = DMA_MuxChannel4,
        .tx_req_num = DMA_REQUEST_USART3_TX,
    }
};

void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* uart isr routine */
    uart_isr(&serial3);
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (DMA_GetFlagStatus(DMA1, DMA1_FLAG_TC3)) {
        dma_rx_done_isr(&serial3);
        DMA_ClearFlag(DMA1, DMA1_FLAG_TC3);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (DMA_GetFlagStatus(DMA1, DMA1_FLAG_TC4)) {
        dma_tx_done_isr(&serial3);
        DMA_ClearFlag(DMA1, DMA1_FLAG_TC4);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* USING_UART3 */

#ifdef USING_UART5
static struct serial_device serial5;
static struct ch32_uart uart5 = {
    .uart_periph = USART5,
    .irqn = USART5_IRQn,
    .clock = RCC_HB1Periph_USART5,
    .tx_port = GPIOE,
    .tx_af = GPIO_AF4,
    .tx_pin = GPIO_Pin_0,
    .rx_port = GPIOF,
    .rx_af = GPIO_AF4,
    .rx_pin = GPIO_Pin_5,
    .dma = {
        .rx_mux_ch = DMA_MuxChannel5,
        .rx_req_num = DMA_REQUEST_USART5_RX,
        .tx_mux_ch = DMA_MuxChannel6,
        .tx_req_num = DMA_REQUEST_USART5_TX,
    }
};

void USART5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* uart isr routine */
    uart_isr(&serial5);
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (DMA_GetFlagStatus(DMA1, DMA1_FLAG_TC5)) {
        dma_rx_done_isr(&serial5);
        DMA_ClearFlag(DMA1, DMA1_FLAG_TC5);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (DMA_GetFlagStatus(DMA1, DMA1_FLAG_TC6)) {
        dma_tx_done_isr(&serial5);
        DMA_ClearFlag(DMA1, DMA1_FLAG_TC6);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* USING_UART5 */

#ifdef USING_UART6
static struct serial_device serial6;
static struct ch32_uart uart6 = {
    .uart_periph = USART6,
    .irqn = USART6_IRQn,
    .clock = RCC_HB1Periph_USART6,
    .tx_port = GPIOA,
    .tx_af = GPIO_AF8,
    .tx_pin = GPIO_Pin_0,
    .rx_port = GPIOA,
    .rx_af = GPIO_AF8,
    .rx_pin = GPIO_Pin_1,
};

void UART6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* uart isr routine */
    uart_isr(&serial6);
    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* USING_UART6 */

#ifdef USING_UART7
static struct serial_device serial7;
static struct ch32_uart uart7 = {
    .uart_periph = USART7,
    .irqn = USART7_IRQn,
    .clock = RCC_HB1Periph_USART7,
    .tx_port = GPIOB,
    .tx_af = GPIO_AF14,
    .tx_pin = GPIO_Pin_13,
    .rx_port = GPIOB,
    .rx_af = GPIO_AF14,
    .rx_pin = GPIO_Pin_12,
    .dma = {
        .rx_mux_ch = DMA_MuxChannel7,
        .rx_req_num = DMA_REQUEST_USART7_RX,
        .tx_mux_ch = DMA_MuxChannel8,
        .tx_req_num = DMA_REQUEST_USART7_TX,
    }
};

void USART7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* uart isr routine */
    uart_isr(&serial7);
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (DMA_GetFlagStatus(DMA1, DMA1_FLAG_TC7)) {
        dma_rx_done_isr(&serial7);
        DMA_ClearFlag(DMA1, DMA1_FLAG_TC7);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel8_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (DMA_GetFlagStatus(DMA1, DMA1_FLAG_TC8)) {
        dma_tx_done_isr(&serial7);
        DMA_ClearFlag(DMA1, DMA1_FLAG_TC8);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* USING_UART7 */


static void _dma_transmit(struct ch32_uart* uart, rt_uint8_t* buf, rt_size_t size)
{
    DMA_Channel_TypeDef* dma_channel = _dma_mux_channel_to_channel[uart->dma.tx_mux_ch];

    DMA_DeInit(dma_channel);

    DMA_InitTypeDef dma_init_struct = {
        .DMA_PeripheralBaseAddr = (uint32_t)(uart->uart_periph->DATAR),
        .DMA_Memory0BaseAddr = 0, /* will be configured later */
        .DMA_DIR = DMA_DIR_PeripheralDST,
        .DMA_BufferSize = 0, /* will be configured later */
        .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
        .DMA_MemoryInc = DMA_MemoryInc_Enable,
        .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
        .DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte,
        .DMA_Mode = DMA_Mode_Normal,
        .DMA_Priority = DMA_Priority_VeryHigh,
        .DMA_M2M = DMA_M2M_Disable
    };
    DMA_Init(dma_channel, &dma_init_struct);
    DMA_ClearFlag(_dma_mux_channel_to_periph(uart->dma.tx_mux_ch), _dma_mux_channel_to_TC[uart->dma.tx_mux_ch]);

    /* enable DMA channel7 */
    DMA_Cmd(dma_channel, ENABLE);
}

static void _dma_tx_config(struct ch32_uart* uart)
{
    if (uart->dma.tx_mux_ch <= 8)
        RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    else
        RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA2, ENABLE);

    DMA_MuxChannelConfig(uart->dma.tx_mux_ch, uart->dma.tx_req_num);
    /* USART DMA enable for transmission */
    USART_DMACmd(uart->uart_periph, USART_DMAReq_Tx, ENABLE);
    /* enable DMA channel transfer complete interrupt */
    DMA_Channel_TypeDef* dma_channel = _dma_mux_channel_to_channel[uart->dma.tx_mux_ch];
    DMA_ITConfig(dma_channel, DMA_IT_TC, ENABLE);

    IRQn_Type irq = _dma_mux_channel_to_irqn[uart->dma.tx_mux_ch];
    NVIC_EnableIRQ(irq);
    NVIC_SetPriority(irq, 0);
}

static void _dma_rx_config(struct ch32_uart* uart, rt_uint8_t* buf, rt_size_t size)
{
    /* set expected receive length */
    uart->dma.setting_recv_len = size;

    /* enable DMA clock */
    if (uart->dma.rx_mux_ch <= 8)
        RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    else
        RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA2, ENABLE);

    /* enable DMA channel transfer complete interrupt */
    IRQn_Type irq = _dma_mux_channel_to_irqn[uart->dma.rx_mux_ch];
    NVIC_EnableIRQ(irq);
    NVIC_SetPriority(irq, 0);

    /* Enable USART IDLE interrupt, which is used by DMA rx */
    USART_ITConfig(uart->uart_periph, USART_IT_IDLE, ENABLE);

    DMA_Channel_TypeDef* dma_channel = _dma_mux_channel_to_channel[uart->dma.rx_mux_ch];
    DMA_DeInit(dma_channel);
    DMA_InitTypeDef dma_init_struct = {
        .DMA_PeripheralBaseAddr = (uint32_t)(&uart->uart_periph->DATAR),
        .DMA_Memory0BaseAddr = (uint32_t)buf, /* will be configured later */
        .DMA_DIR = DMA_DIR_PeripheralSRC,
        .DMA_BufferSize = size, /* will be configured later */
        .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
        .DMA_MemoryInc = DMA_MemoryInc_Enable,
        .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
        .DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte,
        .DMA_Mode = DMA_Mode_Normal,
        .DMA_Priority = DMA_Priority_VeryHigh,
        .DMA_M2M = DMA_M2M_Disable
    };
    DMA_Init(dma_channel, &dma_init_struct);

    /* enable DMA channel transfer complete interrupt */
    DMA_ITConfig(dma_channel, DMA_IT_TC, ENABLE);
    /* enable DMA channel */
    DMA_MuxChannelConfig(uart->dma.rx_mux_ch, uart->dma.rx_req_num);
    DMA_Cmd(dma_channel, ENABLE);
    /* USART DMA enable for reception */
    USART_DMACmd(uart->uart_periph, USART_DMAReq_Rx, ENABLE);
}

static void _close_usart(struct serial_device* serial)
{
    struct ch32_uart* uart = (struct ch32_uart*)serial->parent.user_data;
    DMA_Channel_TypeDef* dma_channel = _dma_mux_channel_to_channel[uart->dma.rx_mux_ch];

    if (serial->parent.open_flag & RT_DEVICE_FLAG_INT_RX) {
        /* disable rx irq */
        USART_ITConfig(uart->uart_periph, USART_IT_RXNE, DISABLE);
    }

    if (serial->parent.open_flag & RT_DEVICE_FLAG_DMA_RX) {
        /* disable DMA channel transfer complete interrupt */
        DMA_ITConfig(dma_channel, DMA_IT_TC, DISABLE);
        /* disable rx idle interrupt */
        USART_ITConfig(uart->uart_periph, USART_IT_IDLE, DISABLE);
        /* disable DMA channel */
        DMA_Cmd(dma_channel, DISABLE);
        /* USART DMA disable for reception */
        USART_DMACmd(uart->uart_periph, USART_DMAReq_Rx, DISABLE);
    }

    if (serial->parent.open_flag & RT_DEVICE_FLAG_DMA_TX) {
        /* disable DMA channel transfer complete interrupt */
        DMA_ITConfig(dma_channel, DMA_IT_TC, DISABLE);
        /* disable DMA channel */
        DMA_Cmd(dma_channel, DISABLE);
        /* USART DMA disable for transmission */
        USART_DMACmd(uart->uart_periph, USART_DMAReq_Tx, DISABLE);
    }
    /* reset last recv index */
    uart->dma.last_recv_index = 0;
}

static void ch32_uart_gpio_init(struct ch32_uart* uart)
{
    GPIO_PinAFConfig(uart->tx_port, gpio_pin_to_source(uart->tx_pin), uart->tx_af);
    GPIO_PinAFConfig(uart->rx_port, gpio_pin_to_source(uart->rx_pin), uart->rx_af);

    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = uart->tx_pin,
        .GPIO_Speed = GPIO_Speed_Very_High,
        .GPIO_Mode = GPIO_Mode_AF_PP,
    };

    GPIO_Init(uart->tx_port, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = uart->rx_pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(uart->rx_port, &GPIO_InitStruct);

    NVIC_SetPriority(uart->irqn, 0);
    NVIC_EnableIRQ(uart->irqn);
}

static rt_err_t usart_configure(struct serial_device* serial, struct serial_configure* cfg)
{
    struct ch32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct ch32_uart*)serial->parent.user_data;

    ch32_uart_gpio_init(uart);

    USART_InitTypeDef USART_InitStruct = {
        .USART_BaudRate = cfg->baud_rate,
        .USART_WordLength = (cfg->data_bits == DATA_BITS_9) ? USART_WordLength_9b : USART_WordLength_8b,
        .USART_StopBits = (cfg->stop_bits == STOP_BITS_2) ? USART_StopBits_2 : USART_StopBits_1,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
    };

    switch (cfg->parity) {
    case PARITY_ODD:
        USART_InitStruct.USART_Parity = USART_Parity_Odd;
        break;
    case PARITY_EVEN:
        USART_InitStruct.USART_Parity = USART_Parity_Even;
        break;
    default:
        USART_InitStruct.USART_Parity = USART_Parity_No;
        break;
    }

    /* enable USART clock */
    RCC_HB1PeriphClockCmd(uart->clock, ENABLE);

    USART_Init(uart->uart_periph, &USART_InitStruct);
    USART_Cmd(uart->uart_periph, ENABLE);
    return RT_EOK;
}

static rt_err_t usart_control(struct serial_device* serial, int cmd, void* arg)
{
    struct ch32_uart* uart;
    rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);

    RT_ASSERT(serial != RT_NULL);
    uart = (struct ch32_uart*)serial->parent.user_data;

    switch (cmd) {
    case RT_DEVICE_CTRL_CLR_INT:
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) {
            /* disable rx irq */
            NVIC_DisableIRQ(uart->irqn);
            /* disable interrupt */
            USART_ITConfig(uart->uart_periph, USART_IT_RXNE, DISABLE);
        }
        break;

    case RT_DEVICE_CTRL_SET_INT:
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) {
            /* enable rx irq */
            NVIC_EnableIRQ(uart->irqn);
            /* enable interrupt */
            USART_ITConfig(uart->uart_periph, USART_IT_RXNE, ENABLE);
        }
        break;

        /* USART DMA config */
    case RT_DEVICE_CTRL_CONFIG:
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) {
            struct serial_rx_fifo* rx_fifo = (struct serial_rx_fifo*)serial->serial_rx;
            struct ch32_uart* uart = (struct ch32_uart*)serial->parent.user_data;

            _dma_rx_config(uart, rx_fifo->buffer, serial->config.bufsz);
        }

        if (ctrl_arg == RT_DEVICE_FLAG_DMA_TX) {
            struct ch32_uart* uart = (struct ch32_uart*)serial->parent.user_data;

            _dma_tx_config(uart);
        }
        break;

    case RT_DEVICE_CTRL_SUSPEND:
        _close_usart(serial);
        break;

    default:
        break;
    }

    return RT_EOK;
}

static int usart_putc(struct serial_device* serial, char ch)
{
    struct ch32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct ch32_uart*)serial->parent.user_data;

    USART_SendData(uart->uart_periph, ch);

    /* wait transmit finish */

    while (!USART_GetFlagStatus(uart->uart_periph, USART_FLAG_TC))
        ;

    return RT_EOK;
}

static int usart_getc(struct serial_device* serial)
{
    int ch = -1;
    struct ch32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct ch32_uart*)serial->parent.user_data;

    if (USART_GetFlagStatus(uart->uart_periph, USART_FLAG_RXNE))
        ch = USART_ReceiveData(uart->uart_periph);

    return ch;
}

static rt_size_t usart_dma_transmit(struct serial_device* serial, rt_uint8_t* buf, rt_size_t size, int direction)
{
    if (direction == SERIAL_DMA_TX) {
        _dma_transmit(serial->parent.user_data, buf, size);
        return size;
    }

    return 0;
}

/* usart driver operations */
static const struct usart_ops __usart_ops = {
    .configure = usart_configure,
    .control = usart_control,
    .putc = usart_putc,
    .getc = usart_getc,
    .dma_transmit = usart_dma_transmit,
};

rt_err_t drv_usart_init(void)
{
    rt_err_t rt_err = RT_EOK;
    struct serial_configure defconfig = SERIAL_DEFAULT_CONFIG;

#ifdef USING_UART2
    serial2.ops = &__usart_ops;
    serial2.config = defconfig;

    /* register serial device */
    rt_err |= hal_serial_register(&serial2,
                                  "serial2",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                                  &uart2);
#endif /* USING_UART2 */

#ifdef USING_UART3
    serial3.ops = &__usart_ops;
    struct serial_configure serial3_config = SERIAL_CONFIG_115200;
    serial3.config = serial3_config;

    /* register serial device */
    rt_err |= hal_serial_register(&serial3,
                                  "serial3  ",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                                  &uart3);
#endif /* USING_UART2 */

#ifdef USING_UART4
    serial4.ops = &__usart_ops;
    #ifdef SERIAL4_DEFAULT_CONFIG
    struct serial_configure serial4_config = SERIAL4_DEFAULT_CONFIG;
    serial4.config = serial4_config;
    #else
    serial4.config = defconfig;
    #endif

    /* register serial device */
    rt_err |= hal_serial_register(&serial4,
                                  "serial4",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                                  &uart4);
#endif /* USING_UART4 */

#ifdef USING_UART5
    serial5.ops = &__usart_ops;
    serial5.config = defconfig;

    /* register serial device */
    rt_err |= hal_serial_register(&serial5,
                                  "serial5",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX,
                                  &uart5);
#endif /* USING_UART5 */

    return rt_err;

#ifdef USING_UART6
    serial6.ops = &__usart_ops;
    struct serial_configure serial6_config = SERIAL_CONFIG_115200;
    serial6.config = serial6_config;

    /* register serial device */
    rt_err |= hal_serial_register(&serial6,
                                  "serial6",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX,
                                  &uart6);
#endif /* USING_UART6 */

#ifdef USING_UART7
    serial7.ops = &__usart_ops;
    serial7.config = defconfig;

    /* register serial device */
    rt_err |= hal_serial_register(&serial7,
                                  "serial7",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX,
                                  &uart7);
#endif /* USING_UART7 */
}
