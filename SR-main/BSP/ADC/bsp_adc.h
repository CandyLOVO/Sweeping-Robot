#pragma once

#include "adc.h"
#include <stdint.h>

#define ADC_CHANNEL_NUM 4

typedef struct
{
    uint16_t *adc_value;
    uint8_t channel;
    // uint16_t (*get_value)(void); // 可选的 getter 封装，便于模块调用
} ADC_Instance;

void ADC_Start(void);
ADC_Instance *ADCRegister(uint8_t channel);
float ADC_GetVoltage(ADC_Instance *instance);
