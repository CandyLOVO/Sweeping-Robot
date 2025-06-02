#pragma once

#include "stdlib.h"
#include "bsp_adc.h"
#include "stdbool.h"

typedef struct {
    float voltages[4]; // 假设有四个ADC通道
    bool detected[4];  // 每个通道是否检测到悬崖
    // 0: LB, 1: LF, 2: RF, 3: RB
} Cliff_Instance;

Cliff_Instance *Cliff_Init(void);
void Cliff_Update(void);