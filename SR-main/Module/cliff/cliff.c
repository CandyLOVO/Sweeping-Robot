#include "cliff.h"
#include "adc.h"
#include "string.h"
#include "bsp_adc.h"

#define SAFE_DISTANCE 0.001f // Safe distance between cliff and ground

static ADC_Instance *left_back_adc, *left_front_adc;
static ADC_Instance *right_back_adc, *right_front_adc;

static Cliff_Instance cliff_data; // 不再使用 malloc，直接静态结构体

Cliff_Instance *Cliff_Init(void)
{
    // 注册 ADC 通道
    left_back_adc   = ADCRegister(ADC_CHANNEL_0);
    left_front_adc  = ADCRegister(ADC_CHANNEL_1);
    right_front_adc = ADCRegister(ADC_CHANNEL_2);
    right_back_adc  = ADCRegister(ADC_CHANNEL_3);

    // 初始化为 0
    memset(&cliff_data, 0, sizeof(Cliff_Instance));
    return &cliff_data;
}

void Cliff_Update(void)
{
    // 读取 ADC 值
    cliff_data.voltages[0] = ADC_GetVoltage(left_back_adc);
    cliff_data.voltages[1] = ADC_GetVoltage(left_front_adc);
    cliff_data.voltages[2] = ADC_GetVoltage(right_front_adc);
    cliff_data.voltages[3] = ADC_GetVoltage(right_back_adc);

    // 检测悬崖
    for (int i = 0; i < 4; i++) {
        if (cliff_data.voltages[i] < SAFE_DISTANCE) {
            cliff_data.detected[i] = true;
        } else {
            cliff_data.detected[i] = false;
        }
    }
}
