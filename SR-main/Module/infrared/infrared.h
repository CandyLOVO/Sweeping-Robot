#ifndef INFRARED_H
#define INFRARED_H

#include "stdint.h"
#include "bsp_gpio.h"

#define IR_MAX_NUM 4 // 最多支持 4 个红外传感器实例

typedef struct IR_Instance {
    GPIO_Instance *gpio;
    uint32_t last_tick;                      // 去抖动时间戳
    void (*on_detect)(struct IR_Instance *); // 中断触发回调（物体检测）
} IR_Instance;

typedef struct {
    GPIO_Init_Config_s gpio_config;
    void (*on_detect)(IR_Instance *ir); // 注册时绑定回调
} IR_Config_s;

IR_Instance *IRRegister(IR_Config_s *config);
void IRUnregister(IR_Instance *ir);

#endif // INFRARED_H
