/**
 * @file key.h
 * @author
 * @brief 按键模块（支持回调、计数、去抖动）
 * @version 0.2
 * @date 2025-04-10
 */

#ifndef KEY_H
#define KEY_H

#include "stdint.h"
#include "bsp_gpio.h"

#define KEY_MAX_NUM     3  // 最大支持按键数量
#define KEY_DEBOUNCE_MS 20 // 默认去抖时间（ms）

/**
 * @brief 按键实例结构体
 */
typedef struct KEY_Instance {
    GPIO_Instance *gpio;                     // GPIO实例
    uint16_t count;                          // 按键按下计数
    uint32_t last_tick;                      // 上一次触发时间（用于去抖）
    void (*on_press)(struct KEY_Instance *); // 用户自定义按下时的回调函数
} KEY_Instance;

/**
 * @brief 按键初始化配置结构体
 */
typedef struct {
    GPIO_Init_Config_s gpio_config;      // GPIO 配置（包含中断模式等）
    void (*on_press)(KEY_Instance *key); // 初始化时绑定的按下回调（可选）
} KEY_Config_s;

/**
 * @brief 注册一个按键，返回 KEY 实例指针
 * @param config 按键初始化配置
 * @return KEY_Instance*
 */
KEY_Instance *KEYRegister(KEY_Config_s *config);

/**
 * @brief 注销一个按键，释放资源
 * @param key 按键实例
 */
void KEYUnregister(KEY_Instance *key);

#endif // KEY_H
