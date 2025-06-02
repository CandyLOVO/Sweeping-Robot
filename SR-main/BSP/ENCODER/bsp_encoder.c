#include "bsp_encoder.h"
#include "memory.h"
#include "stdlib.h"
static uint8_t idx = 0;
static TIM_Encoder_Instance *encoderInstances[MAX_ENCODER_NUM];

TIM_Encoder_Instance *TIM_Encoder_Register(TIM_Encoder_Config *config)
{
    if (idx >= MAX_ENCODER_NUM) return NULL;
    TIM_Encoder_Instance *encoder = (TIM_Encoder_Instance *)malloc(sizeof(TIM_Encoder_Instance));
    if (encoder == NULL) return NULL;
    memset(encoder, 0, sizeof(TIM_Encoder_Instance));
    encoder->htim           = config->htim;
    encoderInstances[idx++] = encoder;
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);

    return encoder;
}