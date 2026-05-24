/******************************************************************************
 * Copyright 2020-2021 The Firmament Authors. All Rights Reserved.
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
#include "drv_i2c.h"
#include "drv_gpio.h"
#include "hal/i2c/i2c.h"

#define USE_I2C1
// #define USE_I2C2
#define USE_I2C3

#define DRV_DBG(...)
// #define DRV_DBG(...)   rt_kprintf(__VA_ARGS__)

/* We want to ensure the real-time performace, so the i2c timeout here is
 * relatively short */
#define I2C_TIMEOUT_US (1000)

struct ch32_i2c_bus {
    struct rt_i2c_bus parent;
    I2C_TypeDef* i2c_periph;
    uint32_t clock;

    GPIO_TypeDef* sda_port;
    uint16_t sda_pin;
    uint8_t sda_af;

    GPIO_TypeDef* scl_port;
    uint16_t scl_pin;
    uint8_t scl_af;
};

static void i2c_hw_init(struct ch32_i2c_bus* bus)
{
    /* I2C0 Configuration */

    /* enable clock */
    RCC_HB1PeriphClockCmd(bus->clock, ENABLE);

    /* GPIO configuration */
    GPIO_InitTypeDef gInit = {
        .GPIO_Pin = bus->sda_pin,
        .GPIO_Speed = GPIO_Speed_Very_High,
        .GPIO_Mode = GPIO_Mode_AF_OD,
    };
    GPIO_Init(bus->sda_port, &gInit);

    gInit.GPIO_Pin = bus->scl_pin;
    GPIO_Init(bus->scl_port, &gInit);

    /* GPIO AF set */
    GPIO_PinAFConfig(bus->sda_port, gpio_pin_to_source(bus->sda_pin), bus->sda_af);
    GPIO_PinAFConfig(bus->scl_port, gpio_pin_to_source(bus->scl_pin), bus->scl_af);

    I2C_InitTypeDef iInit = {
        .I2C_ClockSpeed = 400000,
        .I2C_Mode = I2C_Mode_I2C,
        .I2C_DutyCycle = I2C_DutyCycle_2,
        .I2C_OwnAddress1 = 0x00,
        .I2C_Ack = I2C_Ack_Enable,
        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit
    };
    I2C_Init(bus->i2c_periph, &iInit);
    I2C_Cmd(bus->i2c_periph, ENABLE);
}

static fmt_err_t wait_flag_until_timeout(I2C_TypeDef* i2c_periph, uint32_t flag, FlagStatus status)
{
    uint64_t time_start = systime_now_us();

    while (I2C_GetFlagStatus(i2c_periph, flag) == status) {
        if ((systime_now_us() - time_start) >= I2C_TIMEOUT_US) {
            return FMT_ETIMEOUT;
        }
    }
    return FMT_EOK;
}

static fmt_err_t wait_for_event_until_timeout(I2C_TypeDef* i2c_periph, uint32_t event)
{
    uint64_t time_start = systime_now_us();

    while (!I2C_CheckEvent(i2c_periph, event)) {
        if ((systime_now_us() - time_start) >= I2C_TIMEOUT_US) {
            return FMT_ETIMEOUT;
        }
    }
    return FMT_EOK;
}

static fmt_err_t i2c_read(I2C_TypeDef* i2c_periph,
                          rt_uint16_t addr,
                          rt_uint8_t flags,
                          rt_uint16_t len,
                          rt_uint8_t* buf)
{
    FMT_TRY(wait_flag_until_timeout(i2c_periph, I2C_FLAG_BUSY, SET));

    I2C_GenerateSTART(i2c_periph, ENABLE);

    FMT_TRY(wait_for_event_until_timeout(i2c_periph, I2C_EVENT_MASTER_MODE_SELECT));

    if (flags & RT_I2C_ADDR_10BIT) // 10-bit address mode
    {
        rt_uint8_t frame_head = 0xF0 + (addr >> 8);                                         // 11110XX0
        I2C_SendData(i2c_periph, frame_head);                                               // FrameHead
        FMT_TRY(wait_for_event_until_timeout(i2c_periph, I2C_EVENT_MASTER_MODE_ADDRESS10)); // EVT9

        I2C_SendData(i2c_periph, 0xff & addr);
        FMT_TRY(wait_for_event_until_timeout(i2c_periph, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // EVT6

        I2C_GenerateSTART(i2c_periph, ENABLE);                                           // Sr
        FMT_TRY(wait_for_event_until_timeout(i2c_periph, I2C_EVENT_MASTER_MODE_SELECT)); // EVT5

        I2C_SendData(i2c_periph, frame_head); // Resend FrameHead
    } else {
        I2C_Send7bitAddress(i2c_periph, addr, I2C_Direction_Receiver);
    }

    FMT_TRY(wait_for_event_until_timeout(i2c_periph, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    while (len-- > 0) {
        FMT_TRY(wait_flag_until_timeout(i2c_periph, I2C_FLAG_RXNE, RESET));
        if (len == 0) {
            I2C_AcknowledgeConfig(i2c_periph, DISABLE);
        }

        *(buf++) = I2C_ReceiveData(i2c_periph);
    }

    I2C_GenerateSTOP(i2c_periph, ENABLE);
    I2C_AcknowledgeConfig(i2c_periph, ENABLE);

    return FMT_EOK;
}

static fmt_err_t i2c_write(I2C_TypeDef* i2c_periph,
                           rt_uint16_t addr,
                           rt_uint8_t flags,
                           rt_uint8_t* buf,
                           rt_uint16_t len)
{
    FMT_TRY(wait_flag_until_timeout(i2c_periph, I2C_FLAG_BUSY, SET));

    I2C_GenerateSTART(i2c_periph, ENABLE);

    FMT_TRY(wait_for_event_until_timeout(i2c_periph, I2C_EVENT_MASTER_MODE_SELECT)); // EVT5

    if (flags & RT_I2C_ADDR_10BIT) // 10-bit address mode
    {
        I2C_SendData(i2c_periph, 0xF0 + (addr >> 8));                                       // FrameHead 11110XX0
        FMT_TRY(wait_for_event_until_timeout(i2c_periph, I2C_EVENT_MASTER_MODE_ADDRESS10)); // EVT9
        I2C_SendData(i2c_periph, 0xff & addr);
    } else // 7-bit address mode
    {
        I2C_Send7bitAddress(i2c_periph, addr, I2C_Direction_Transmitter);
    }

    FMT_TRY(wait_for_event_until_timeout(i2c_periph, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // EVT6

    // Missing Evt8_1 (No definition)

    while (len-- > 0) {
        FMT_TRY(wait_flag_until_timeout(i2c_periph, I2C_FLAG_TXE, RESET)); // Got ACK For the Last Byte

        I2C_SendData(i2c_periph, *(buf++));
    }

    FMT_TRY(wait_for_event_until_timeout(i2c_periph, I2C_EVENT_MASTER_BYTE_TRANSMITTING)); // Last byte sent successfully

    I2C_GenerateSTOP(i2c_periph, ENABLE);
    return FMT_EOK;
}

static rt_size_t i2c_master_transfer(struct rt_i2c_bus* bus, rt_uint16_t slave_addr, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg* msg;
    uint32_t msg_idx = 0;
    struct ch32_i2c_bus* ch32_i2c = (struct ch32_i2c_bus*)bus;

    for (msg_idx = 0; msg_idx < num; msg_idx++) {
        msg = &msgs[msg_idx];
        if (msg->flags & RT_I2C_RD) {
            if (i2c_read(ch32_i2c->i2c_periph, slave_addr, msg->flags, msg->len, msg->buf) != FMT_EOK) {
                goto _error;
            }

        } else {
            if (i2c_write(ch32_i2c->i2c_periph, slave_addr, msg->flags, msg->buf, msg->len) != FMT_EOK) {
                goto _error;
            }
        }
    }

    return msg_idx;
_error:
    I2C_GenerateSTOP(ch32_i2c->i2c_periph, ENABLE);
    I2C_AcknowledgeConfig(ch32_i2c->i2c_periph, ENABLE);
    return msg_idx;
}

static const struct rt_i2c_bus_device_ops i2c_bus_ops = {
    i2c_master_transfer,
    RT_NULL,
    RT_NULL
};

/* i2c device instances */
static struct rt_i2c_device i2c3_dev0 = {
    .slave_addr = 0x30, /* MMC5983MA 7 bit address */
    .flags = 0
};

static struct rt_i2c_device i2c3_dev1 = {
    .slave_addr = 0x77, /* SPL06 7 bit address */
    .flags = 0
};

static struct rt_i2c_device i2c3_dev2 = {
    .slave_addr = 0x45, /* AW2023 7 bit address */
    .flags = 0
};

rt_err_t drv_i2c_init(void)
{
    /* i2c bus instances */

#ifdef USE_I2C1
    static struct ch32_i2c_bus ch32_i2c1 = {
        .parent.ops = &i2c_bus_ops,
        .i2c_periph = I2C1,
        .clock = RCC_HB1Periph_I2C1,

        .sda_port = GPIOB,
        .sda_pin = GPIO_Pin_7,
        .sda_af = GPIO_AF4,

        .scl_port = GPIOB,
        .scl_pin = GPIO_Pin_6,
        .scl_af = GPIO_AF4,
    };
    i2c_hw_init(&ch32_i2c1);
    RT_TRY(rt_i2c_bus_device_register(&ch32_i2c1.parent, "i2c1"));
#endif // USE_I2C1

#ifdef USE_I2C2
    static struct ch32_i2c_bus ch32_i2c2 = {
        .parent.ops = &i2c_bus_ops,
        .i2c_periph = I2C2,
        .clock = RCC_HB1Periph_I2C2,

        .sda_port = GPIOC,
        .sda_pin = GPIO_Pin_1,
        .sda_af = GPIO_AF9,

        .scl_port = GPIOC,
        .scl_pin = GPIO_Pin_0,
        .scl_af = GPIO_AF9,
    };
    i2c_hw_init(&ch32_i2c2);
    RT_TRY(rt_i2c_bus_device_register(&ch32_i2c2.parent, "i2c2"));
#endif // USE_I2C2

#ifdef USE_I2C3
    static struct ch32_i2c_bus ch32_i2c3 = {
        .parent.ops = &i2c_bus_ops,
        .i2c_periph = I2C3,
        .clock = RCC_HB1Periph_I2C3,

        .sda_port = GPIOA,
        .sda_pin = GPIO_Pin_13,
        .sda_af = GPIO_AF7,

        .scl_port = GPIOA,
        .scl_pin = GPIO_Pin_14,
        .scl_af = GPIO_AF7,
    };
    i2c_hw_init(&ch32_i2c3);
    RT_TRY(rt_i2c_bus_device_register(&ch32_i2c3.parent, "i2c3"));
#endif // USE_I2C3

    /* attach i2c devices */

    RT_TRY(rt_i2c_bus_attach_device(&i2c3_dev0, "i2c3_dev0", "i2c3", RT_NULL));
    RT_TRY(rt_i2c_bus_attach_device(&i2c3_dev1, "i2c3_dev1", "i2c3", RT_NULL));
    RT_TRY(rt_i2c_bus_attach_device(&i2c3_dev2, "i2c3_dev2", "i2c3", RT_NULL));

    return RT_EOK;
}
