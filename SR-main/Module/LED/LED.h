/**
 * @file led.h
 * @brief LED 控制模块头文件（面向实例，风格统一）
 * @version 0.1
 * @date 2025-04-10
 */

#ifndef LED_H
#define LED_H

#include "stdint.h"
#include "bsp_gpio.h"

#define LED_MAX_NUM 10 // 支持的最大LED实例数

typedef enum {
    LED_OFF = 0, // 关闭
    LED_ON  = 1, // 打开
} LED_State_e;

/**
 * @brief LED实例结构体
 */
typedef struct LED_Instance {
    GPIO_Instance *gpio;                      // 绑定的GPIO实例
    LED_State_e state;                        // 当前LED状态（LED_OFF=灭，LED_ON=亮）
    void (*on_change)(struct LED_Instance *); // 状态变更时的用户回调（可选）
} LED_Instance;

/**
 * @brief LED初始化配置结构体
 */
typedef struct {
    GPIO_Init_Config_s gpio_config;       // GPIO初始化配置
    LED_State_e initial_state;            // 初始状态（LED_OFF=灭，LED_ON=亮）
    void (*on_change)(LED_Instance *led); // 状态变更回调（可选）
} LED_Config_s;

/**
 * @brief 注册一个LED实例
 * @param config 初始化配置
 * @return LED_Instance* 返回实例指针，失败返回NULL
 */
LED_Instance *LEDRegister(LED_Config_s *config);

/**
 * @brief 注销LED实例，释放资源
 * @param led LED实例
 */
void LEDUnregister(LED_Instance *led);

/**
 * @brief 点亮LED
 * @param led LED实例
 */
void LEDOn(LED_Instance *led);

/**
 * @brief 熄灭LED
 * @param led LED实例
 */
void LEDOff(LED_Instance *led);

/**
 * @brief 切换LED状态
 * @param led LED实例
 */
void LEDToggle(LED_Instance *led);

/**
 * @brief 获取当前LED状态
 * @param led LED实例
 * @return uint8_t 当前状态（0=灭，1=亮）
 */
uint8_t LEDGetState(LED_Instance *led);

#endif // LED_H
