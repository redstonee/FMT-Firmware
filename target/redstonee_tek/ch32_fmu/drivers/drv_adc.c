/******************************************************************************
 * Copyright 2023 The Firmament Authors. All Rights Reserved.
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
#include "drv_adc.h"
#include "ch32h417_conf.h"
#include "hal/adc/adc.h"

#define ADC_CONVERSION_TIMEOUT_MS 2

static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t adc_ch;
} adcs[] = {
    { GPIOB, GPIO_Pin_1, ADC_Channel_9 }, // VSENS
    { GPIOB, GPIO_Pin_0, ADC_Channel_8 }, // ISENS
};

#define N_ADC sizeof(adcs) / sizeof(adcs[0])

static struct adc_device adc1;
static struct rt_completion convert_cplt;

void ADC1_2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void ADC1_2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

        /* inform the completion of adc convertion */
        rt_completion_done(&convert_cplt);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

static rt_err_t adc_hw_init(void)
{
    /* enable ADC clock */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_ADC1, ENABLE);
    /* config ADC clock */
    RCC_ADCCLKConfig(RCC_ADCCLKSource_HCLK);
    RCC_ADCHCLKCLKAsSourceConfig(RCC_PPRE2_DIV0, RCC_ADCPRE_DIV8);

    for (uint8_t i = 0; i < N_ADC; i++) {
        GPIO_InitTypeDef gInit = {
            .GPIO_Pin = adcs[i].pin,
            .GPIO_Speed = GPIO_Speed_Very_High,
            .GPIO_Mode = GPIO_Mode_AIN,
        };
        GPIO_Init(adcs[i].port, &gInit);
    }

    ADC_DeInit(ADC1);
    ADC_InitTypeDef aInit = {
        .ADC_Mode = ADC_Mode_Independent,
        .ADC_ScanConvMode = DISABLE,
        .ADC_ContinuousConvMode = ENABLE,
        .ADC_ExternalTrigConv = ADC_ExternalTrigConv_None,
        .ADC_DataAlign = ADC_DataAlign_Right,
        .ADC_NbrOfChannel = N_ADC,
    };
    ADC_Init(ADC1, &aInit);
    ADC_LowPowerModeCmd(ADC1, ENABLE); // Low power mode for sample rate under 2Msps
    ADC_BufferCmd(ADC1, DISABLE);

    /* enable ADC end of conversion interrupt */
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    NVIC_EnableIRQ(ADC1_2_IRQn);

    return RT_EOK;
}

static rt_err_t enable(adc_dev_t adc_dev, uint8_t enable)
{
    if (enable == ADC_CMD_ENABLE) {
        /* enable ADC interface */
        ADC_Cmd(ADC1, ENABLE);
        /* ADC calibration and reset calibration
         the ADC needs a stabilization time of tSTAB
         * before it starts converting accurately
         */
        ADC_ResetCalibration(ADC1);
        while (ADC_GetResetCalibrationStatus(ADC1))
            ;
        ADC_StartCalibration(ADC1);
        while (ADC_GetCalibrationStatus(ADC1))
            ;
    } else if (enable == ADC_CMD_DISABLE) {
        ADC_Cmd(ADC1, DISABLE);
    } else {
        return RT_EINVAL;
    }

    return RT_EOK;
}

static rt_err_t measure(adc_dev_t adc_dev, uint32_t channel, uint32_t* mVolt)
{
    if (channel > N_ADC) {
        return RT_EINVAL;
    }

    /* ADC routine channel config */
    ADC_RegularChannelConfig(ADC1, adcs[channel].adc_ch, 1, ADC_SampleTime_CyclesMode3); // 28.5 cycles

    /* ADC software trigger enable */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    if (rt_completion_wait(&convert_cplt, TICKS_FROM_MS(ADC_CONVERSION_TIMEOUT_MS)) != RT_EOK) {
        return RT_ERROR;
    }

    uint16_t adcData = ADC_GetConversionValue(ADC1);
    *mVolt = adcData * 3300 / 4095;

    return RT_EOK;
}

/* usart driver operations */
static const struct adc_ops _adc_ops = {
    .enable = enable,
    .measure = measure
};

rt_err_t drv_adc_init(void)
{
    RT_CHECK(adc_hw_init());

    rt_completion_init(&convert_cplt);

    adc1.ops = &_adc_ops;

    return hal_adc_register(&adc1, "adc1", RT_DEVICE_FLAG_RDONLY, RT_NULL);
}