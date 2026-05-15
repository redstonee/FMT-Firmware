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
#include "drv_spi.h"
#include "drv_gpio.h"
#include "hal/spi/spi.h"

#include "ch32h417_conf.h"

// #define SPI_USE_DMA

struct ch32_spi_bus {
    struct rt_spi_bus parent;
    SPI_TypeDef* spi_periph;
    struct rt_spi_configuration bus_config;

    GPIO_TypeDef* mosi_port;
    GPIO_TypeDef* miso_port;
    GPIO_TypeDef* clk_port;

    uint16_t mosi_pin;
    uint16_t miso_pin;
    uint16_t clk_pin;

    uint8_t mosi_af;
    uint8_t miso_af;
    uint8_t clk_af;
#ifdef SPI_USE_DMA
    // TODO
#endif
};

struct spi_cs {
    GPIO_TypeDef* port;
    uint16_t pin;
};

/**
 * @brief Configure spi device
 *
 * @param device SPI device
 * @param configuration SPI device configuration
 * @return rt_err_t RT_EOK for success
 */
static rt_err_t configure(struct rt_spi_device* device,
                          struct rt_spi_configuration* configuration)
{
    struct ch32_spi_bus* ch32_spi_bus = (struct ch32_spi_bus*)device->bus;

    SPI_InitTypeDef sInit;

    if (ch32_spi_bus->bus_config.mode == configuration->mode
        && ch32_spi_bus->bus_config.data_width == configuration->data_width
        && ch32_spi_bus->bus_config.max_hz == configuration->max_hz) {
        /* same configuration, do not need re-configure */
        return RT_EOK;
    }

    if (configuration->data_width <= 8) {
        sInit.SPI_DataSize = SPI_DataSize_8b;
    } else if (configuration->data_width <= 16) {
        sInit.SPI_DataSize = SPI_DataSize_16b;
    } else {
        return RT_EIO;
    }

    /* baudrate */
    uint32_t max_hz;
    RCC_ClocksTypeDef rcc_clocks;

    RCC_GetClocksFreq(&rcc_clocks);
    max_hz = configuration->max_hz;

    if (max_hz >= rcc_clocks.HCLK_Frequency / 2) {
        sInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode0;
    } else if (max_hz >= rcc_clocks.HCLK_Frequency / 4) {
        sInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode1;
    } else if (max_hz >= rcc_clocks.HCLK_Frequency / 8) {
        sInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode2;
    } else if (max_hz >= rcc_clocks.HCLK_Frequency / 16) {
        sInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode3;
    } else if (max_hz >= rcc_clocks.HCLK_Frequency / 32) {
        sInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode4;
    } else if (max_hz >= rcc_clocks.HCLK_Frequency / 64) {
        sInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode5;
    } else if (max_hz >= rcc_clocks.HCLK_Frequency / 128) {
        sInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode6;
    } else {
        /*  min prescaler 256 */
        sInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode7;
    }

    sInit.SPI_CPOL = (configuration->mode & RT_SPI_CPOL) ? SPI_CPOL_High : SPI_CPOL_Low;
    sInit.SPI_CPHA = (configuration->mode & RT_SPI_CPHA) ? SPI_CPHA_2Edge : SPI_CPHA_1Edge;
    sInit.SPI_FirstBit = (configuration->mode & RT_SPI_MSB) ? SPI_FirstBit_MSB : SPI_FirstBit_LSB;
    sInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    sInit.SPI_Mode = SPI_Mode_Master;
    sInit.SPI_NSS = SPI_NSS_Soft;

    SPI_I2S_DeInit(ch32_spi_bus->spi_periph);
    SPI_Init(ch32_spi_bus->spi_periph, &sInit);
    SPI_Cmd(ch32_spi_bus->spi_periph, ENABLE);

    ch32_spi_bus->bus_config = *configuration;

    return RT_EOK;
}

/**
 * @brief SPI transfer function
 *
 * @param device SPI device instance
 * @param message SPI message to be transfered
 * @return rt_uint32_t bytes have been transfered
 */
static rt_uint32_t transfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    struct ch32_spi_bus* ch32_spi_bus = (struct ch32_spi_bus*)device->bus;
    struct rt_spi_configuration* config = &device->config;
    struct spi_cs* ch32_spi_cs = (struct spi_cs*)device->parent.user_data;
    rt_uint32_t size = message->length;

    /* take CS */
    if (message->cs_take) {
        GPIO_ResetBits(ch32_spi_cs->port, ch32_spi_cs->pin);
    }

#ifdef SPI_USE_DMA
    #error Not support SPI DMA.
#endif

    {
        if (config->data_width <= 8) {
            const rt_uint8_t* send_ptr = message->send_buf;
            rt_uint8_t* recv_ptr = message->recv_buf;

            while (size--) {
                rt_uint8_t data = 0xFF;

                if (send_ptr != RT_NULL) {
                    data = *send_ptr++;
                }

                /* Wait until the transmit buffer is empty */
                while (!SPI_I2S_GetFlagStatus(ch32_spi_bus->spi_periph, SPI_I2S_FLAG_TXE))
                    ;

                /* Send the byte */
                SPI_I2S_SendData(ch32_spi_bus->spi_periph, data);

                /* Wait until a data is received */
                while (!SPI_I2S_GetFlagStatus(ch32_spi_bus->spi_periph, SPI_I2S_FLAG_RXNE))
                    ;

                /* Get the received data */
                data = SPI_I2S_ReceiveData(ch32_spi_bus->spi_periph);

                if (recv_ptr != RT_NULL) {
                    *recv_ptr++ = data;
                }
            }
        } else if (config->data_width <= 16) {
            const rt_uint16_t* send_ptr = message->send_buf;
            rt_uint16_t* recv_ptr = message->recv_buf;

            while (size--) {
                rt_uint16_t data = 0xFFFF;

                if (send_ptr != RT_NULL) {
                    data = *send_ptr++;
                }

                /* Wait until the transmit buffer is empty */
                while (!SPI_I2S_GetFlagStatus(ch32_spi_bus->spi_periph, SPI_I2S_FLAG_TXE))
                    ;

                /* Send the byte */
                SPI_I2S_SendData(ch32_spi_bus->spi_periph, data);

                /* Wait until a data is received */
                while (!SPI_I2S_GetFlagStatus(ch32_spi_bus->spi_periph, SPI_I2S_FLAG_RXNE))
                    ;

                /* Get the received data */
                data = SPI_I2S_ReceiveData(ch32_spi_bus->spi_periph);

                if (recv_ptr != RT_NULL) {
                    *recv_ptr++ = data;
                }
            }
        }
    }

    /* release CS */
    if (message->cs_release) {
        GPIO_SetBits(ch32_spi_cs->port, ch32_spi_cs->pin);
    }

    return message->length;
}

static struct rt_spi_ops __spi_ops = {
    configure,
    transfer
};

/** \brief init GPIOs of a ch32 SPI bus
 *
 * \param ch32_spi: ch32 SPI bus struct.
 * \param spi_cs: array of SPI CSes
 * \param spi_cs: the number of SPI CSes
 */
static void spi_hw_init(struct ch32_spi_bus* ch32_spi, struct spi_cs** spi_cs, uint8_t num_cs)
{
    GPIO_InitTypeDef gInit = {
        .GPIO_Pin = ch32_spi->miso_pin,
        .GPIO_Speed = GPIO_Speed_Very_High,
        .GPIO_Mode = GPIO_Mode_IN_FLOATING,
    };
    GPIO_Init(ch32_spi->miso_port, &gInit);
    GPIO_PinAFConfig(ch32_spi->miso_port, gpio_pin_to_source(ch32_spi->miso_pin), ch32_spi->miso_af);

    gInit.GPIO_Mode = GPIO_Mode_AF_PP;
    gInit.GPIO_Pin = ch32_spi->mosi_pin;
    GPIO_Init(ch32_spi->mosi_port, &gInit);
    GPIO_PinAFConfig(ch32_spi->mosi_port, gpio_pin_to_source(ch32_spi->mosi_pin), ch32_spi->mosi_af);

    gInit.GPIO_Pin = ch32_spi->clk_pin;
    GPIO_Init(ch32_spi->clk_port, &gInit);
    GPIO_PinAFConfig(ch32_spi->clk_port, gpio_pin_to_source(ch32_spi->clk_pin), ch32_spi->clk_af);

    for (uint8_t i = 0; i < num_cs; i++) {
        gInit.GPIO_Pin = spi_cs[i]->pin;
        GPIO_Init(spi_cs[i]->port, &gInit);
        GPIO_SetBits(spi_cs[i]->port, spi_cs[i]->pin);
    }
#ifdef SPI_USE_DMA
    // TODO
#endif
}

/**
 * @brief Initialize spi bus and device
 *
 * @return rt_err_t RT_EOK for success
 */
rt_err_t drv_spi_init(void)
{
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_SPI1, ENABLE);
    static struct ch32_spi_bus ch32_spi1 = {
        .parent.ops = &__spi_ops,
        .spi_periph = SPI1,

        .miso_port = GPIOA,
        .mosi_port = GPIOA,
        .clk_port = GPIOA,

        .miso_pin = GPIO_Pin_6,
        .mosi_pin = GPIO_Pin_7,
        .clk_pin = GPIO_Pin_5,

        .miso_af = GPIO_AF5,
        .mosi_af = GPIO_AF5,
        .clk_af = GPIO_AF5,
    };

    /* BMI088 ACCEL */
    static struct rt_spi_device rt_spi1_dev0;
    static struct spi_cs spi1_cs0 = { GPIOC, GPIO_Pin_4 };

    /* BMI088 GYRO) */
    static struct rt_spi_device rt_spi0_dev1;
    static struct spi_cs spi1_cs1 = { GPIOC, GPIO_Pin_4 };

    struct spi_cs* cses[] = { &spi1_cs0, &spi1_cs1 };
    spi_hw_init(&ch32_spi1, cses, sizeof(cses) / sizeof(cses[0]));

    RT_TRY(rt_spi_bus_register(&(ch32_spi1.parent), "spi1", &__spi_ops));
    RT_TRY(rt_spi_bus_attach_device(&rt_spi1_dev0, "spi1_dev1", "spi1", (void*)&spi1_cs0));
    RT_TRY(rt_spi_bus_attach_device(&rt_spi0_dev1, "spi0_dev1", "spi0", (void*)&spi1_cs1));

    return RT_EOK;
}
