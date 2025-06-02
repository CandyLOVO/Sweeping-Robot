#include "infrared.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t ir_idx                        = 0;
static IR_Instance *ir_instances[IR_MAX_NUM] = {NULL};

/**
 * @brief 红外传感器中断回调处理（由 GPIO 模块统一回调）
 */
static void IR_HandleInterrupt(GPIO_Instance *gpio)
{
    IR_Instance *ir = (IR_Instance *)gpio->id;
    if (ir == NULL) return;

    uint64_t now = DWT_GetTimeline_us();

    // 简单去抖（例如红外遮挡震动等）
    if ((now - ir->last_tick) > 5000) { // 5ms 抖动过滤
        ir->last_tick = now;
        if (ir->on_detect) ir->on_detect(ir); // 调用用户回调
    }
}

/**
 * @brief 注册红外传感器
 */
IR_Instance *IRRegister(IR_Config_s *config)
{
    if (ir_idx >= IR_MAX_NUM) return NULL;

    IR_Instance *ir = (IR_Instance *)malloc(sizeof(IR_Instance));
    if (!ir) return NULL;
    memset(ir, 0, sizeof(IR_Instance));

    config->gpio_config.gpio_model_callback = IR_HandleInterrupt;
    config->gpio_config.id                  = ir;

    ir->gpio      = GPIORegister(&config->gpio_config);
    ir->last_tick = DWT_GetTimeline_us();
    ir->on_detect = config->on_detect;

    ir_instances[ir_idx++] = ir;
    return ir;
}

/**
 * @brief 注销红外传感器
 */
void IRUnregister(IR_Instance *ir)
{
    for (int i = 0; i < IR_MAX_NUM; ++i) {
        if (ir_instances[i] == ir) {
            ir_instances[i] = NULL;
            free(ir);
            break;
        }
    }
}
