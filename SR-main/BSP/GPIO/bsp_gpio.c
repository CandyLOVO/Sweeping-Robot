#include "bsp_gpio.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t idx                                          = 0;
static GPIO_Instance *gpio_instances[GPIO_INSTANCE_MAX_NUM] = {NULL};

/**
 * @brief 注册 GPIO 实例并保存到 gpio_instances 数组
 * @param GPIO_config GPIO 初始化配置
 * @return GPIO_Instance* 返回实例指针，失败返回 NULL
 */
GPIO_Instance *GPIORegister(GPIO_Init_Config_s *GPIO_config)
{
    if (idx >= GPIO_INSTANCE_MAX_NUM) return NULL;

    GPIO_Instance *gpio = (GPIO_Instance *)malloc(sizeof(GPIO_Instance));
    if (!gpio) return NULL;

    memset(gpio, 0, sizeof(GPIO_Instance));

    gpio->GPIOx               = GPIO_config->GPIOx;
    gpio->GPIO_Pin            = GPIO_config->GPIO_Pin;
    gpio->pin_state           = GPIO_config->pin_state;
    gpio->exti_mode           = GPIO_config->exti_mode;
    gpio->id                  = GPIO_config->id;
    gpio->gpio_model_callback = GPIO_config->gpio_model_callback;

    gpio_instances[idx++] = gpio;
    if (gpio->pin_state == GPIO_PIN_SET) {
        HAL_GPIO_WritePin(gpio->GPIOx, gpio->GPIO_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(gpio->GPIOx, gpio->GPIO_Pin, GPIO_PIN_RESET);
    }
    return gpio;
}

/**
 * @brief 注销并释放一个 GPIO 实例
 * @param instance 要释放的 GPIO 实例
 */
void GPIOUnregister(GPIO_Instance *instance)
{
    if (!instance) return;
    for (int i = 0; i < GPIO_INSTANCE_MAX_NUM; ++i) {
        if (gpio_instances[i] == instance) {
            gpio_instances[i] = NULL;
            free(instance);
            break;
        }
    }
}

/**
 * @brief HAL层中断回调，在中断发生时调用
 * @param GPIO_Pin 被触发的引脚编号（HAL 定义宏）
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    GPIO_Instance *gpio = NULL;
    for (uint8_t i = 0; i < GPIO_INSTANCE_MAX_NUM; ++i) {
        gpio = gpio_instances[i];
        if (gpio && gpio->GPIO_Pin == GPIO_Pin && gpio->gpio_model_callback) {
            gpio->gpio_model_callback(gpio);
            return;
        }
    }
}

/**
 * @brief 切换 GPIO 电平
 * @param instance GPIO 实例
 */
void GPIOToggel(GPIO_Instance *instance)
{
    HAL_GPIO_TogglePin(instance->GPIOx, instance->GPIO_Pin);
}

/**
 * @brief 设置 GPIO 为高电平
 * @param instance GPIO 实例
 */
GPIO_PinState GPIOSet(GPIO_Instance *instance)
{
    HAL_GPIO_WritePin(instance->GPIOx, instance->GPIO_Pin, GPIO_PIN_SET);
    return GPIO_PIN_SET;
}

/**
 * @brief 设置 GPIO 为低电平
 * @param instance GPIO 实例
 */
GPIO_PinState GPIOReset(GPIO_Instance *instance)
{
    instance->pin_state = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(instance->GPIOx, instance->GPIO_Pin, GPIO_PIN_RESET);
    return GPIO_PIN_RESET;
}

/**
 * @brief 读取 GPIO 电平状态
 * @param instance GPIO 实例
 * @return GPIO_PinState 当前引脚状态
 */
GPIO_PinState GPIORead(GPIO_Instance *instance)
{
    instance->pin_state = HAL_GPIO_ReadPin(instance->GPIOx, instance->GPIO_Pin);
    return instance->pin_state;
}
