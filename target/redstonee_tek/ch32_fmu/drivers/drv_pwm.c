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
#include <firmament.h>

#include "drv_gpio.h"
#include "drv_pwm.h"
#include "hal/actuator/actuator.h"

/*
    PA9     TIM1_2  AF1
    PA10    TIM1_3  AF1
    PC6     TIM3_1  AF2
    PC7     TIM3_2  AF2
    PC8     TIM3_3  AF2
    PC9     TIM3_4  AF2
*/

// #define DRV_DBG(...) console_printf(__VA_ARGS__)
#define DRV_DBG(...)

#define PWM_FREQ_50HZ         (50)
#define PWM_FREQ_125HZ        (125)
#define PWM_FREQ_250HZ        (250)
#define PWM_FREQ_400HZ        (400)

#define TIMER_FREQUENCY       2500000       // Timer frequency: 2.5M
#define PWM_DEFAULT_FREQUENCY PWM_FREQ_50HZ // pwm default frequqncy
#define VAL_TO_DC(_val)       ((float)(_val * __pwm_freq) / 1000000.0f)
#define DC_TO_VAL(_dc)        (1000000.0f / __pwm_freq * _dc)

#define PWM_ARR(freq)         (TIMER_FREQUENCY / freq) // CCR reload value, Timer frequency = TIMER_FREQUENCY/(PWM_ARR+1)

static rt_err_t pwm_config(actuator_dev_t dev, const struct actuator_configure* cfg);
static rt_err_t pwm_control(actuator_dev_t dev, int cmd, void* arg);
static rt_size_t pwm_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t* chan_val, rt_size_t size);
static rt_size_t pwm_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size);

const static struct actuator_ops __act_ops = {
    .act_config = pwm_config,
    .act_control = pwm_control,
    .act_read = pwm_read,
    .act_write = pwm_write
};

static uint32_t __pwm_freq = PWM_DEFAULT_FREQUENCY;

typedef struct
{
    TIM_TypeDef* periph;
    uint16_t channel;

    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t af;

    float dc;
} ch32_timer;

static ch32_timer timers[] = {
    { TIM1, TIM_Channel_3, GPIOA, GPIO_Pin_10, GPIO_AF1 },
    { TIM1, TIM_Channel_2, GPIOA, GPIO_Pin_9, GPIO_AF1 },
    { TIM3, TIM_Channel_4, GPIOC, GPIO_Pin_9, GPIO_AF2 },
    { TIM3, TIM_Channel_1, GPIOC, GPIO_Pin_6, GPIO_AF2 },
    { TIM3, TIM_Channel_2, GPIOC, GPIO_Pin_7, GPIO_AF2 },
    { TIM3, TIM_Channel_3, GPIOC, GPIO_Pin_8, GPIO_AF2 },
};
#define N_TIMERS (sizeof(timers) / sizeof(timers[0]))

static struct actuator_device act_dev = {
    .chan_mask = 0x3FF,
    .range = { 1000, 2000 },
    .config = {
        .protocol = ACT_PROTOCOL_PWM,
        .chan_num = N_TIMERS,
        .pwm_config = { .pwm_freq = 50 },
        .dshot_config = { 0 } },
    .ops = &__act_ops
};

static void pwm_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Speed = GPIO_Speed_Very_High,
        .GPIO_Mode = GPIO_Mode_Out_PP,
    };

    for (uint8_t i = 0; i < N_TIMERS; i++) {
        GPIO_InitStruct.GPIO_Pin = timers[i].pin;
        GPIO_Init(timers[i].port, &GPIO_InitStruct);
        GPIO_PinAFConfig(timers[i].port, gpio_pin_to_source(timers[i].pin), timers[i].af);
    }
}

static void pwm_timer_init(void)
{
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_TIM3, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    uint16_t PrescalerValue;
    RCC_ClocksTypeDef rcc_clocks;

    /* TIMCLK = HCLK */
    RCC_GetClocksFreq(&rcc_clocks);

    /* Compute the prescaler value*/
    PrescalerValue = (uint16_t)((rcc_clocks.HCLK_Frequency / TIMER_FREQUENCY) - 1);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = PWM_ARR(PWM_DEFAULT_FREQUENCY) - 1; // PWM Frequency = 3M/60K = 50 Hz
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    for (uint8_t i = 0; i < N_TIMERS; i++) {
        timers[i].dc = 0;
        TIM_TimeBaseInit(timers[i].periph, &TIM_TimeBaseStructure);
        TIM_ARRPreloadConfig(timers[i].periph, ENABLE);

        switch (timers[i].channel) {
        case TIM_Channel_1:
            TIM_OC1Init(timers[i].periph, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(timers[i].periph, TIM_OCPreload_Enable);
            break;

        case TIM_Channel_2:
            TIM_OC2Init(timers[i].periph, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(timers[i].periph, TIM_OCPreload_Enable);
            break;

        case TIM_Channel_3:
            TIM_OC3Init(timers[i].periph, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(timers[i].periph, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(timers[i].periph, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(timers[i].periph, TIM_OCPreload_Enable);
            break;
        default:
            break;
        }
        TIM_Cmd(timers[i].periph, DISABLE);
    }
}

rt_inline void __read_pwm(uint8_t chan_id, float* dc)
{
    *dc = timers[chan_id].dc;
}

rt_inline void __write_pwm(uint8_t chan_id, float dc)
{
    if (chan_id >= N_TIMERS)
        return;

    ch32_timer* timer = &timers[chan_id];
    uint16_t compare = PWM_ARR(__pwm_freq) * dc - 1;

    switch (timer->channel) {
    case TIM_Channel_1:
        TIM_SetCompare1(timer->periph, compare);
        break;
    case TIM_Channel_2:
        TIM_SetCompare2(timer->periph, compare);
        break;
    case TIM_Channel_3:
        TIM_SetCompare3(timer->periph, compare);
        break;
    case TIM_Channel_4:
        TIM_SetCompare4(timer->periph, compare);
        break;
    default:
        break;
    }

    timer->dc = dc;
}

static rt_err_t __set_pwm_frequency(uint16_t freq)
{
    if (freq < PWM_FREQ_50HZ || freq > PWM_FREQ_400HZ) {
        /* invalid frequency */
        return RT_EINVAL;
    }

    __pwm_freq = freq;

    /* the timer compare value should be re-configured */
    for (uint8_t i = 0; i < N_TIMERS; i++) {
        __write_pwm(i, timers[i].dc);
        TIM_SetAutoreload(timers[i].periph, PWM_ARR(__pwm_freq) - 1);
    }

    return RT_EOK;
}

static rt_err_t pwm_config(actuator_dev_t dev, const struct actuator_configure* cfg)
{
    DRV_DBG("aux out configured: pwm frequency:%d\n", cfg->pwm_config.pwm_freq);

    if (__set_pwm_frequency(cfg->pwm_config.pwm_freq) != RT_EOK) {
        return RT_ERROR;
    }
    /* update device configuration */
    dev->config = *cfg;

    return RT_EOK;
}

static rt_err_t pwm_control(actuator_dev_t dev, int cmd, void* arg)
{
    rt_err_t ret = RT_EOK;

    switch (cmd) {
    case ACT_CMD_CHANNEL_ENABLE:
        /* set to lowest pwm before open */
        for (uint8_t i = 0; i < N_TIMERS; i++) {
            TIM_Cmd(timers[i].periph, ENABLE);
            __write_pwm(i, VAL_TO_DC(act_dev.range[0]));
        }
        break;
    case ACT_CMD_CHANNEL_DISABLE:
        /* auto-reload preload disable */
        for (uint8_t i = 0; i < N_TIMERS; i++) {
            TIM_Cmd(timers[i].periph, DISABLE);
        }
        break;
    case ACT_CMD_SET_PROTOCOL:
        /* TODO: Support dshot */
        ret = RT_EINVAL;
        break;
    default:
        ret = RT_EINVAL;
        break;
    }

    return ret;
}

static rt_size_t pwm_read(actuator_dev_t dev, rt_uint16_t chan_sel, rt_uint16_t* chan_val, rt_size_t size)
{
    float dc;

    for (uint8_t i = 0; i < N_TIMERS; i++) {
        if (chan_sel & (1 << i)) {
            __read_pwm(i, &dc);
            chan_val[i] = DC_TO_VAL(dc);
        }
    }

    return size;
}

static rt_size_t pwm_write(actuator_dev_t dev, rt_uint16_t chan_sel, const rt_uint16_t* chan_val, rt_size_t size)
{
    rt_uint16_t val;
    float dc;

    for (uint8_t i = 0; i < N_TIMERS; i++) {
        if (chan_sel & (1 << i)) {
            val = chan_val[i];
            /* calculate pwm duty cycle */
            dc = VAL_TO_DC(val);
            /* update pwm signal */
            __write_pwm(i, dc);
        }
    }

    return size;
}

rt_err_t drv_pwm_init(void)
{
    /* init pwm gpio pin */
    pwm_gpio_init();
    /* init pwm timer, pwm output mode */
    pwm_timer_init();

    /* register actuator hal device */
    return hal_actuator_register(&act_dev, "main_out", RT_DEVICE_FLAG_RDWR, NULL);
}