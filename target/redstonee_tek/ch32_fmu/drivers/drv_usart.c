*Copyright 2020 The Firmament Authors.All Rights Reserved.
/******************************************************************************
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

#include "drv_usart.h"
#include "hal/serial/serial.h"

#define SET_BIT(REG, BIT)   ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)  ((REG) & (BIT))

#define UART_ENABLE_IRQ(n)  NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n) NVIC_DisableIRQ((n))

#define USING_UART2
#define USING_UART3
#define USING_UART4
#define USING_UART5
#define USING_UART6

#define SERIAL3_DEFAULT_CONFIG                 \
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

/* CH32H41x uart driver */
struct ch32_uart {
    USART_TypeDef* uart_periph;
    IRQn_Type irqn;
    uint32_t per_clk;
    uint32_t tx_gpio_clk;
    uint32_t rx_gpio_clk;
    uint32_t tx_port;
    uint16_t tx_af;
    uint16_t tx_pin;
    uint32_t rx_port;
    uint16_t rx_af;
    uint16_t rx_pin;
    struct ch32_uart_dma {
        /* dma clock */
        uint32_t clock;
        /* dma rx irq */
        uint8_t rx_irq;
        /* dma rx channel */
        uint8_t rx_mux_ch;
        /* dma rx request number */
        uint8_t rx_req_num;
        /* dma tx irq */
        uint8_t tx_irq;
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
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;
    uint32_t remain_bytes;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();
    /* check remain bytes to receive */
    remain_bytes = dma_transfer_number_get(uart->dma.dma_periph, uart->dma.rx_ch);
    /* total received bytes */
    recv_total_index = uart->dma.setting_recv_len - remain_bytes;
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

    if ((usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_RBNE) != RESET) && (usart_flag_get(uart->uart_periph, USART_FLAG_RBNE) != RESET)) {
        /* high-level ISR routine */
        hal_serial_isr(serial, SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        usart_flag_clear(uart->uart_periph, USART_FLAG_RBNE);
    }

    if ((usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_IDLE) != RESET) && (usart_flag_get(uart->uart_periph, USART_FLAG_IDLE) != RESET)) {
        dma_uart_rx_idle_isr(serial);
        /* Read USART_DATA to clear IDLE interrupt flag */
        usart_data_receive(uart->uart_periph);
    }

    if ((usart_interrupt_flag_get(uart->uart_periph, USART_INT_FLAG_TC) != RESET) && (usart_flag_get(uart->uart_periph, USART_FLAG_TC) != RESET)) {
        /* Clear TC interrupt flag */
        usart_flag_clear(uart->uart_periph, USART_FLAG_TC);
    }
}

#ifdef USING_UART2
static struct serial_device serial2;
static struct ch32_uart uart2 = {
    .uart_periph = USART2,
    .irqn = USART2_IRQn,
    .per_clk = RCC_HB1Periph_USART2,
    .tx_gpio_clk = RCC_HB2Periph_GPIOA,
    .rx_gpio_clk = RCC_HB2Periph_GPIOA,
    .tx_port = GPIOA,
    .tx_af = GPIO_AF7,
    .tx_pin = GPIO_Pin_2,
    .rx_port = GPIOA,
    .rx_af = GPIO_AF7,
    .rx_pin = GPIO_Pin_3,
    .dma = {
        .clock = RCC_HBPeriph_DMA1,
        .rx_mux_ch = DMA_MuxChannel1,
        .rx_irq = DMA1_Channel1_IRQn,
        .rx_req_num = DMA_REQUEST_USART2_RX,
        .tx_mux_ch = DMA_MuxChannel2,
        .tx_irq = DMA1_Channel2_IRQn,
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
    if (dma_interrupt_flag_get(uart2.dma.dma_periph, uart2.dma.rx_ch, DMA_INT_FLAG_FTF)) {
        dma_rx_done_isr(&serial2);
        dma_interrupt_flag_clear(uart2.dma.dma_periph, uart2.dma.rx_ch, DMA_INT_FLAG_FTF);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (dma_interrupt_flag_get(uart2.dma.dma_periph, uart2.dma.tx_ch, DMA_INT_FLAG_FTF)) {
        dma_tx_done_isr(&serial2);
        dma_interrupt_flag_clear(uart2.dma.dma_periph, uart2.dma.tx_ch, DMA_INT_FLAG_FTF);
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
    .per_clk = RCC_HB1Periph_USART3,
    .tx_gpio_clk = RCC_HB2Periph_GPIOC,
    .rx_gpio_clk = RCC_HB2Periph_GPIOC,
    .tx_port = GPIOC,
    .tx_af = GPIO_AF7,
    .tx_pin = GPIO_Pin_10,
    .rx_port = GPIOC,
    .rx_af = GPIO_AF7,
    .rx_pin = GPIO_Pin_11,
    .dma = {
        .clock = RCC_HBPeriph_DMA1,
        .rx_mux_ch = DMA_MuxChannel3,
        .rx_irq = DMA1_Channel3_IRQn,
        .rx_req_num = DMA_REQUEST_USART3_RX,
        .tx_mux_ch = DMA_MuxChannel4,
        .tx_irq = DMA1_Channel4_IRQn,
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
    if (dma_interrupt_flag_get(uart3.dma.dma_periph, uart3.dma.rx_ch, DMA_INT_FLAG_FTF)) {
        dma_rx_done_isr(&serial3);
        dma_interrupt_flag_clear(uart3.dma.dma_periph, uart3.dma.rx_ch, DMA_INT_FLAG_FTF);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (dma_interrupt_flag_get(uart3.dma.dma_periph, uart3.dma.tx_ch, DMA_INT_FLAG_FTF)) {
        dma_tx_done_isr(&serial3);
        dma_interrupt_flag_clear(uart3.dma.dma_periph, uart3.dma.tx_ch, DMA_INT_FLAG_FTF);
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
    .per_clk = RCC_HB1Periph_USART5,
    .tx_gpio_clk = RCC_HB2Periph_GPIOE,
    .rx_gpio_clk = RCC_HB2Periph_GPIOF,
    .tx_port = GPIOE,
    .tx_af = GPIO_AF4,
    .tx_pin = GPIO_Pin_0,
    .rx_port = GPIOF,
    .rx_af = GPIO_AF4,
    .rx_pin = GPIO_Pin_5,
    .dma = {
        .clock = RCC_HBPeriph_DMA1,
        .rx_mux_ch = DMA_MuxChannel5,
        .rx_irq = DMA1_Channel5_IRQn,
        .rx_req_num = DMA_REQUEST_USART5_RX,
        .tx_mux_ch = DMA_MuxChannel6,
        .tx_irq = DMA1_Channel6_IRQn,
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
    if (dma_interrupt_flag_get(uart5.dma.dma_periph, uart5.dma.rx_ch, DMA_INT_FLAG_FTF)) {
        dma_rx_done_isr(&serial5);
        dma_interrupt_flag_clear(uart5.dma.dma_periph, uart5.dma.rx_ch, DMA_INT_FLAG_FTF);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (dma_interrupt_flag_get(uart5.dma.dma_periph, uart5.dma.tx_ch, DMA_INT_FLAG_FTF)) {
        dma_tx_done_isr(&serial5);
        dma_interrupt_flag_clear(uart5.dma.dma_periph, uart5.dma.tx_ch, DMA_INT_FLAG_FTF);
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
    .per_clk = RCC_HB1Periph_USART6,
    .tx_gpio_clk = RCC_HB2Periph_GPIOA,
    .rx_gpio_clk = RCC_HB2Periph_GPIOA,
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
static struct serial_device serial4;
static struct ch32_uart uart7 = {
    .uart_periph = UART7,
    .irqn = UART7_IRQn,
    .per_clk = RCU_UART7,
    .tx_gpio_clk = RCU_GPIOE,
    .rx_gpio_clk = RCU_GPIOE,
    .tx_port = GPIOE,
    .tx_af = GPIO_AF_8,
    .tx_pin = GPIO_PIN_1,
    .rx_port = GPIOE,
    .rx_af = GPIO_AF_8,
    .rx_pin = GPIO_PIN_0,
    // .dma = {
    //     .dma_periph = DMA0,
    //     .clock = RCU_DMA0,
    //     .rx_ch = DMA_CH1,
    //     .rx_irq = DMA0_Channel1_IRQn,
    //     .tx_ch = DMA_CH3,
    //     .tx_irq = DMA0_Channel3_IRQn,
    //     .sub_periph = DMA_SUBPERI4,
    // }
};

void UART7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* uart isr routine */
    uart_isr(&serial4);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* USING_UART7 */

static const DMA_Channel_TypeDef* _dma_mux_channel_to_channel[] = {
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

static uint8_t _gpio_pin_to_source(uint16_t gpio_pin)
{
    uint8_t pin_source = 0;

    while (gpio_pin > 1) {
        gpio_pin >>= 1;
        pin_source++;
    }
    return pin_source;
}

static void _dma_transmit(struct ch32_uart* uart, rt_uint8_t* buf, rt_size_t size)
{
    /* wait current transfers are finished */
    while (READ_BIT(DMA_CHCTL(uart->dma.dma_periph, uart->dma.tx_ch), BIT(0)))
        ;

    dma_memory_address_config(uart->dma.dma_periph, uart->dma.tx_ch, 0, (uint32_t)buf);
    dma_transfer_number_config(uart->dma.dma_periph, uart->dma.tx_ch, size);

    /* enable DMA channel transfer complete interrupt */
    dma_interrupt_enable(uart->dma.dma_periph, uart->dma.tx_ch, DMA_CHXCTL_FTFIE);
    /* enable DMA channel7 */
    dma_channel_enable(uart->dma.dma_periph, uart->dma.tx_ch);
}

static void _dma_tx_config(struct ch32_uart* uart)
{
    DMA_Channel_TypeDef* dma_channel = _dma_mux_channel_to_channel[uart->dma.tx_mux_ch];

    /* enable dma tx interrupt */
    nvic_irq_enable(uart->dma.tx_irq, 0, 1);

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

    DMA_MuxChannelConfig(uart->dma.tx_mux_ch, uart->dma.tx_req_num);
    DMA_Cmd(dma_channel, ENABLE);
    /* USART DMA enable for transmission */
    USART_DMACmd(uart->uart_periph, USART_DMAReq_Tx, ENABLE);
}

static void _dma_rx_config(struct ch32_uart* uart, rt_uint8_t* buf, rt_size_t size)
{
    DMA_Channel_TypeDef* dma_channel = _dma_mux_channel_to_channel[uart->dma.rx_mux_ch];

    /* set expected receive length */
    uart->dma.setting_recv_len = size;

    nvic_irq_enable(uart->dma.rx_irq, 0, 0);
    /* Enable USART IDLE interrupt, which is used by DMA rx */
    USART_ITConfig(uart->uart_periph, USART_IT_IDLE, ENABLE);

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
    /* enable USART clock */
    RCC_HBPeriphClockCmd(uart->tx_gpio_clk, ENABLE);
    RCC_HBPeriphClockCmd(uart->rx_gpio_clk, ENABLE);
    RCC_HBPeriphClockCmd(uart->per_clk, ENABLE);
    if (uart->dma.clock == RCC_HBPeriph_DMA1 || uart->dma.clock == RCC_HBPeriph_DMA2) {
        /* enable DMA1 clock */
        RCC_HBPeriphClockCmd(uart->dma.clock, ENABLE);
    }

    GPIO_PinAFConfig(uart->tx_port, _gpio_pin_to_source(uart->tx_pin), uart->tx_af);
    GPIO_PinAFConfig(uart->rx_port, _gpio_pin_to_source(uart->rx_pin), uart->rx_af);

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

            if (READ_BIT(DMA_CHCTL(uart->dma.dma_periph, uart->dma.rx_ch), BIT(0))) {
                return RT_EBUSY;
            }

            _dma_rx_config(uart, rx_fifo->buffer, serial->config.bufsz);
        }

        if (ctrl_arg == RT_DEVICE_FLAG_DMA_TX) {
            struct ch32_uart* uart = (struct ch32_uart*)serial->parent.user_data;

            if (READ_BIT(DMA_CHCTL(uart->dma.dma_periph, uart->dma.tx_ch), 0)) {
                return RT_EBUSY;
            }

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

    usart_data_transmit(uart->uart_periph, ch);

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
        ch = usart_data_receive(uart->uart_periph);

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
    struct serial_configure config = SERIAL_DEFAULT_CONFIG;

#ifdef USING_UART2
    serial2.ops = &__usart_ops;
    #ifdef SERIAL2_DEFAULT_CONFIG
    struct serial_configure serial2_config = SERIAL2_DEFAULT_CONFIG;
    serial2.config = serial2_config;
    #else
    serial2.config = config;
    #endif

    /* register serial device */
    rt_err |= hal_serial_register(&serial2,
                                  "serial2",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                                  &uart2);
#endif /* USING_UART2 */

#ifdef USING_UART3
    serial3.ops = &__usart_ops;
    #ifdef SERIAL3_DEFAULT_CONFIG
    struct serial_configure serial3_config = SERIAL3_DEFAULT_CONFIG;
    serial3.config = serial3_config;
    #else
    serial3.config = config;
    #endif

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
    serial4.config = config;
    #endif

    /* register serial device */
    rt_err |= hal_serial_register(&serial4,
                                  "serial4",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                                  &uart4);
#endif /* USING_UART4 */

#ifdef USING_UART5
    serial5.ops = &__usart_ops;
    #ifdef SERIAL5_DEFAULT_CONFIG
    struct serial_configure serial5_config = SERIAL5_DEFAULT_CONFIG;
    serial5.config = serial5_config;
    #else
    serial5.config = config;
    #endif

    /* register serial device */
    rt_err |= hal_serial_register(&serial5,
                                  "serial5",
                                  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX,
                                  &uart5);
#endif /* USING_UART5 */

    return rt_err;
}

#ifdef USING_UART6
serial0.ops = &__usart_ops;
    #ifdef SERIAL0_DEFAULT_CONFIG
struct serial_configure serial0_config = SERIAL0_DEFAULT_CONFIG;
serial0.config = serial0_config;
    #else
serial0.config = config;
    #endif

/* register serial device */
rt_err |= hal_serial_register(&serial0,
                              "serial0",
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_INT_RX,
                              &uart6);
#endif /* USING_UART6 */
