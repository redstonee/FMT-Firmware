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

#include "drv_gpio.h"
#include "hal/pin/pin.h"

#define PIN_PORT(pin)   ((uint8_t)(((pin) >> 4) & 0xFu))
#define PIN_NO(pin)     ((uint8_t)((pin) & 0xFu))

#define PIN_CHPORT(pin) ((GPIO_TypeDef*)(PERIPH_BASE + 0x10800u + (0x400u * PIN_PORT(pin))))
#define PIN_CHPIN(pin)  ((uint16_t)(1u << PIN_NO(pin)))

#if defined(GPIOZ)
    #define __CH32_PORT_MAX 12u
#elif defined(GPIOK)
    #define __CH32_PORT_MAX 11u
#elif defined(GPIOJ)
    #define __CH32_PORT_MAX 10u
#elif defined(GPIOI)
    #define __CH32_PORT_MAX 9u
#elif defined(GPIOH)
    #define __CH32_PORT_MAX 8u
#elif defined(GPIOG)
    #define __CH32_PORT_MAX 7u
#elif defined(GPIOF)
    #define __CH32_PORT_MAX 6u
#elif defined(GPIOE)
    #define __CH32_PORT_MAX 5u
#elif defined(GPIOD)
    #define __CH32_PORT_MAX 4u
#elif defined(GPIOC)
    #define __CH32_PORT_MAX 3u
#elif defined(GPIOB)
    #define __CH32_PORT_MAX 2u
#elif defined(GPIOA)
    #define __CH32_PORT_MAX 1u
#else
    #define __CH32_PORT_MAX 0u
    #error Unsupported CH32 GPIO peripheral.
#endif

#define PIN_CHPORT_MAX __CH32_PORT_MAX

static struct pin_device pin_device;

static void ch32_pin_write(pin_dev_t dev, rt_base_t pin, rt_base_t value)
{
    if (PIN_PORT(pin) < PIN_CHPORT_MAX) {
        if (value == 0) {
            GPIO_ResetBits(PIN_CHPORT(pin), PIN_CHPIN(pin));
        } else {
            GPIO_SetBits(PIN_CHPORT(pin), PIN_CHPIN(pin));
        }
    }
}

static int ch32_pin_read(pin_dev_t dev, rt_base_t pin)
{
    if (PIN_PORT(pin) < PIN_CHPORT_MAX) {
        return GPIO_ReadInputDataBit(PIN_CHPORT(pin), PIN_CHPIN(pin));
    }

    return PIN_LOW;
}

static void ch32_pin_mode(pin_dev_t dev, rt_base_t pin, rt_base_t mode, rt_base_t otype)
{
    if (PIN_PORT(pin) >= PIN_CHPORT_MAX) {
        return;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = PIN_CHPIN(pin),
        .GPIO_Speed = GPIO_Speed_Very_High,
    };

    switch (mode) {
    case PIN_MODE_OUTPUT:
        GPIO_InitStruct.GPIO_Mode = otype == PIN_OUT_TYPE_PP ? GPIO_Mode_Out_PP : GPIO_Mode_Out_OD;
        break;
    case PIN_MODE_INPUT_PULLUP:
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        break;
    case PIN_MODE_INPUT_PULLDOWN:
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
        break;
    case PIN_MODE_INPUT:
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        break;
    default: // Unsupported mode
        return;
    }

    GPIO_Init(PIN_CHPORT(pin), &GPIO_InitStruct);
}

const static struct pin_ops pin_ops = {
    .pin_mode = ch32_pin_mode,
    .pin_write = ch32_pin_write,
    .pin_read = ch32_pin_read,
};

rt_err_t drv_gpio_init(void)
{
    /* GPIO Ports Clock Enable */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOC, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOD, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOE, ENABLE);

    pin_device.ops = &pin_ops;

    return hal_pin_register(&pin_device, "pin", RT_DEVICE_FLAG_RDWR, RT_NULL);
}
