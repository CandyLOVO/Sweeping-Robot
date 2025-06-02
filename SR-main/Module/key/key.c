#include "key.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t idx                              = 0;
static KEY_Instance *key_instances[KEY_MAX_NUM] = {NULL}; // 按键实例列表

/**
 * @brief 按键中断触发时的处理函数（由 GPIO 层回调）
 * @param gpio 引发中断的 GPIO 实例
 */
static void KEY_HandleInterrupt(GPIO_Instance *gpio)
{
    KEY_Instance *key = (KEY_Instance *)gpio->id;
    if (key == NULL) return;

    uint64_t now_us = DWT_GetTimeline_us();

    // 检测状态变化 去抖

    if ((now_us - key->last_tick) > KEY_DEBOUNCE_MS * 1000) {
        key->count++;
        key->last_tick = now_us;
        if (key->on_press) key->on_press(key); // 执行用户注册的回调
    }
}

/**
 * @brief 注册一个按键
 * @param config 配置结构体
 * @return KEY_Instance* 按键实例指针
 */
KEY_Instance *KEYRegister(KEY_Config_s *config)
{
    if (idx >= KEY_MAX_NUM) return NULL;

    KEY_Instance *key = (KEY_Instance *)malloc(sizeof(KEY_Instance));
    if (key == NULL) return NULL;
    memset(key, 0, sizeof(KEY_Instance));

    config->gpio_config.gpio_model_callback = KEY_HandleInterrupt; // 绑定回调
    config->gpio_config.id                  = key;

    key->gpio       = GPIORegister(&config->gpio_config);
    key->count      = 0;
    key->last_tick  = DWT_GetTimeline_us();
    key->on_press   = config->on_press; // 可选绑定用户回调

    key_instances[idx++] = key;
    return key;
}

/**
 * @brief 注销按键实例，释放资源
 * @param key 按键实例
 */
void KEYUnregister(KEY_Instance *key)
{
    for (int i = 0; i < idx; ++i) {
        if (key_instances[i] == key) {
            key_instances[i] = NULL;
            free(key);
            break;
        }
    }
}
