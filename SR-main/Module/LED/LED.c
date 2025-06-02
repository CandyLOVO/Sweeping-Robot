#include "LED.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t led_idx                          = 0;
static LED_Instance *led_instances[LED_MAX_NUM] = {NULL};

/**
 * @brief 注册一个 LED 实例
 * @param config 配置结构体
 * @return LED_Instance* 返回实例指针
 */
LED_Instance *LEDRegister(LED_Config_s *config)
{
    if (led_idx >= LED_MAX_NUM) return NULL;

    LED_Instance *led = (LED_Instance *)malloc(sizeof(LED_Instance));
    if (!led) return NULL;

    memset(led, 0, sizeof(LED_Instance));

    // 绑定 GPIO
    config->gpio_config.id = led;
    led->gpio              = GPIORegister(&config->gpio_config);

    // 设置初始状态
    if (config->initial_state == LED_ON) {
        GPIOReset(led->gpio);
        led->state = LED_ON;
    } else {
        GPIOSet(led->gpio);
        led->state = LED_OFF;
    }

    led->on_change = config->on_change;

    led_instances[led_idx++] = led;
    return led;
}

/**
 * @brief 注销 LED 实例
 * @param led LED实例
 */
void LEDUnregister(LED_Instance *led)
{
    for (int i = 0; i < LED_MAX_NUM; ++i) {
        if (led_instances[i] == led) {
            led_instances[i] = NULL;
            GPIOUnregister(led->gpio); // 如果需要清除GPIO
            free(led);
            break;
        }
    }
}

/**
 * @brief 点亮 LED
 * @param led LED 实例
 */
void LEDOn(LED_Instance *led)
{
    if (!led || led->state == LED_ON) return;

    GPIOReset(led->gpio);
    led->state = LED_ON;

    if (led->on_change)
        led->on_change(led);
}

/**
 * @brief 熄灭 LED
 * @param led LED 实例
 */
void LEDOff(LED_Instance *led)
{
    if (!led || led->state == LED_OFF) return;

    GPIOSet(led->gpio);
    led->state = LED_OFF;

    if (led->on_change)
        led->on_change(led);
}

/**
 * @brief 切换 LED 状态
 * @param led LED 实例
 */
void LEDToggle(LED_Instance *led)
{
    if (!led) return;

    GPIOToggel(led->gpio);
    led->state = !led->state;

    if (led->on_change)
        led->on_change(led);
}

/**
 * @brief 获取当前 LED 状态
 * @param led LED 实例
 * @return 0=灭，1=亮
 */
uint8_t LEDGetState(LED_Instance *led)
{
    if (!led) return LED_OFF;
    return led->state;
}
