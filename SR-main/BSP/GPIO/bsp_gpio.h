#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "gpio.h"
#include "stdint.h"

#define GPIO_INSTANCE_MAX_NUM 10 // 最大支持的 GPIO 实例数量

// The External Interrupt Mode, Make sure as the same as CubeMX Config
typedef enum {
    GPIO_EXTI_MODE_RISING,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING,
    GPIO_EXTI_MODE_NONE,
} GPIO_EXTI_MODE_e;

/**
 * @brief GPIO 实例结构体（运行时注册实例）
 */
typedef struct GPIO_Instance {
    GPIO_TypeDef *GPIOx;                                 // GPIO 端口
    GPIO_PinState pin_state;                             // 当前引脚电平
    GPIO_EXTI_MODE_e exti_mode;                          // 外部中断触发模式
    uint16_t GPIO_Pin;                                   // HAL 宏定义的 GPIO_PIN_0...GPIO_PIN_15
    void (*gpio_model_callback)(struct GPIO_Instance *); // 外部模块提供的回调函数
    void *id;                                            // 绑定的模块实例（如 key、led 等）
} GPIO_Instance;

/**
 * @brief GPIO 初始化配置结构体
 */
typedef struct {
    GPIO_TypeDef *GPIOx;
    GPIO_PinState pin_state;
    GPIO_EXTI_MODE_e exti_mode;
    uint16_t GPIO_Pin;
    void (*gpio_model_callback)(GPIO_Instance *);
    void *id;
} GPIO_Init_Config_s;

// 注册 GPIO 实例
GPIO_Instance *GPIORegister(GPIO_Init_Config_s *GPIO_config);

// 注销 GPIO 实例（建议补充实现）
void GPIOUnregister(GPIO_Instance *instance);

// GPIO 控制 API
void GPIOToggel(GPIO_Instance *_instance);
GPIO_PinState GPIOSet(GPIO_Instance *_instance);
GPIO_PinState GPIOReset(GPIO_Instance *_instance);
GPIO_PinState GPIORead(GPIO_Instance *_instance);

#endif // BSP_GPIO_H
