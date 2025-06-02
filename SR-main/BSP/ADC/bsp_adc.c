#include "adc.h"
#include "bsp_adc.h"
#include "stdlib.h"
#include "string.h"

#define ADC_MAX_VALUE   4095.0f
#define ADC_REF_VOLTAGE 3.3f

static uint16_t adc_buf[ADC_CHANNEL_NUM]; // 仍是静态局部变量
static ADC_Instance *adc_instance[ADC_CHANNEL_NUM] = {0};

void ADC_Start(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_CHANNEL_NUM);
}

ADC_Instance *ADCRegister(uint8_t channel)
{
    if (channel >= ADC_CHANNEL_NUM) return NULL;

    ADC_Instance *instance = (ADC_Instance *)malloc(sizeof(ADC_Instance));
    if (!instance) return NULL;

    memset(instance, 0, sizeof(ADC_Instance));
    instance->channel   = channel;                        // 保存通道编号
    // instance->get_value = []() -> uint16_t { return 0; }; // 占位
    // instance->get_value = (uint16_t (*)())(void *)((uintptr_t)adc_buf + channel * sizeof(uint16_t));
    instance->adc_value = &adc_buf[channel];

    adc_instance[channel] = instance;
    return instance;
}

float ADC_GetVoltage(ADC_Instance *instance)
{
    if (!instance || !instance->adc_value)
        return 0.0f;

    return ((float)(*(instance->adc_value)) / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
}
