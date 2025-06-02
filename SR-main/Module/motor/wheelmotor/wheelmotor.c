#include "wheelmotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include "tim.h"
#include "daemon.h"
#include <math.h>     // 计算三角函数时用到
#include "cmsis_os.h" // FreeRTOS头文件

static uint8_t idx                                                    = 0;
static WheelMotor_Instance *wheelmotor_instances[WHEEL_MOTOR_MAX_NUM] = {NULL}; // 电机实例列表

static void WheelMotorLossCallback(void *owner);
static void DecodeWheelMotor(WheelMotor_Instance *motor);

WheelMotor_Instance *WheelMotorInit(WheelMotor_Init_Config_s *config)
{
    if (idx >= WHEEL_MOTOR_MAX_NUM)
        return NULL;

    WheelMotor_Instance *motor = (WheelMotor_Instance *)malloc(sizeof(WheelMotor_Instance));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(WheelMotor_Instance));

    motor->motor_settings = config->controller_setting_init_config;

    PIDInit(&motor->motor_controller.speed_PID_forward, &config->controller_param_init_config.speed_PID_forward);
    PIDInit(&motor->motor_controller.speed_PID_reverse, &config->controller_param_init_config.speed_PID_reverse); // 你可以传入另一套参数
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;
    
    // 初始化PWM和编码器
    motor->pwm     = PWMRegister(&config->pwm_init_config);
    motor->encoder = TIM_Encoder_Register(&config->encoder_init_config);

    config->gpio_init_config.exti_mode           = EXTI_MODE_NONE; // Updated to use the single GPIO config
    config->gpio_init_config.gpio_model_callback = NULL;
    motor->gpio_1                                           = GPIORegister(&config->gpio_init_config);

    // 初始化测量数据
    motor->measurement.encoder_total_count = 0;
    motor->measurement.total_angle         = 0;
    motor->measurement.total_round         = 0;
    motor->measurement.speed_aps           = 0;
    motor->measurement.linear_speed        = 0;
    motor->measurement.direction           = 0; // 初始设定为正转

    Daemon_Init_Config_s daemon_config = {
        .callback     = WheelMotorLossCallback,
        .owner_id     = motor,
        .reload_count = 2, // 20ms未收到数据则判定丢失
    };
    motor->daemon = DaemonRegister(&daemon_config);
    WheelMotorEnable(motor); // 设置电机启动标志

    wheelmotor_instances[idx++] = motor; // 保存到实例数组
    return motor;
}

void WheelMotorControl(void)
{
    for (size_t i = 0; i < idx; i++) {
        if (wheelmotor_instances[i] != NULL) {
            DecodeWheelMotor(wheelmotor_instances[i]);
        }
    }

    WheelMotor_Instance *motor;
    WheelMotor_Control_Setting_s *motor_setting;
    WheelMotor_Controller_s *motor_controller;
    WheelMotor_Measurement_s *measurement;

    for (size_t i = 0; i < idx; i++) {
        motor            = wheelmotor_instances[i];
        motor_setting    = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measurement      = &motor->measurement;

        if (motor == NULL) continue;

        if (motor->stop_flag == MOTOR_STOP) {
            motor->gpio_1->pin_state = GPIOReset(motor->gpio_1);
            PWMSetDutyRatio(motor->pwm, 0.0f);
        } else if (motor->stop_flag == MOTOR_ENABLE) {

            float ref_speed = motor_controller->pid_ref;
            float measured_speed;

            if (motor_setting->speed_feedback_source == OTHER_FEED && motor_controller->other_speed_feedback_ptr != NULL) {
                measured_speed = *motor_controller->other_speed_feedback_ptr;
            } else {
                measured_speed = measurement->speed_aps;
            }

            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD && motor_controller->speed_feedforward_ptr != NULL) {
                ref_speed += *motor_controller->speed_feedforward_ptr;
            }
            
            float pid_out;
            if (ref_speed >= 0) {
                pid_out = PIDCalculate(&motor_controller->speed_PID_forward, measured_speed, ref_speed);
            } else {
                pid_out = PIDCalculate(&motor_controller->speed_PID_reverse, measured_speed, ref_speed);
            }

            if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
                pid_out = -pid_out;
            }

            if (pid_out < 0) {
                motor->gpio_1->pin_state = GPIOSet(motor->gpio_1); // 反转
            } else {
                motor->gpio_1->pin_state = GPIOReset(motor->gpio_1);
            }

            float duty_ratio = fabsf(pid_out) / 750.0f;
            if (duty_ratio > 1.0f) duty_ratio = 1.0f;
            if (duty_ratio < 0.0f) duty_ratio = 0.0f;

            motor_controller->pid_out       = pid_out;

            motor->pwm_out = duty_ratio;
            PWMSetDutyRatio(motor->pwm, duty_ratio);
        }
    }
}

/**
 * @brief 设置电机启动标志
 *
 * @param motor 电机实例指针
 */
void WheelMotorEnable(WheelMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_ENABLE;
}

/**
 * @brief 停止电机：直接清零发送的电流值
 *
 * @param motor 电机实例指针
 */
void WheelMotorStop(WheelMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

/**
 * @brief 设置电机参考值
 *
 * @param motor 电机实例指针
 * @param ref 参考值
 */
void WheelMotorSetRef(WheelMotor_Instance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

/**
 * @brief 电机通信丢失回调函数，停止电机
 */
static void WheelMotorLossCallback(void *owner)
{
    WheelMotor_Instance *motor = (WheelMotor_Instance *)owner;
    motor->stop_flag           = MOTOR_STOP;
}

static void DecodeWheelMotor(WheelMotor_Instance *motor)
{
    // 静态变量记录前一次编码器值（按实例编号区分）
    static uint16_t last_encoder[WHEEL_MOTOR_MAX_NUM] = {0};
    uint8_t id                                        = 0;
    for (; id < WHEEL_MOTOR_MAX_NUM; ++id) {
        if (wheelmotor_instances[id] == motor)
            break;
    }

    // 当前时间
    uint32_t now_ms     = osKernelGetTickCount();
    uint32_t dt_ms      = now_ms - motor->last_tick_ms;
    motor->last_tick_ms = now_ms;

    // 转为秒
    motor->dt = dt_ms / 1000.0f;
    if (motor->dt <= 0.0f) motor->dt = 0.001f; // 防止除以0

    int8_t reverse_flag = 1;
    if (motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
        reverse_flag = -1;
    }

    // 当前编码器读数
    int16_t curr_encoder = __HAL_TIM_GET_COUNTER(motor->encoder->htim);
    curr_encoder *= reverse_flag;
    int16_t delta    = (int16_t)(curr_encoder - last_encoder[id]);
    last_encoder[id] = curr_encoder;

    motor->measurement.encoder = curr_encoder;
    motor->measurement.encoder_total_count += delta;

    // 角度变化（度）
    // 原始角速度
    float angle_increment = delta * DEGREE_PER_COUNT;
    float raw_speed       = angle_increment / motor->dt;

    // 跳变抑制（限幅）
    float last_speed  = motor->measurement.speed_aps;
    float delta_speed = raw_speed - last_speed;
    if (fabsf(delta_speed) > APS_JUMP_THRESHOLD) {
        raw_speed = last_speed + (delta_speed > 0 ? APS_JUMP_THRESHOLD : -APS_JUMP_THRESHOLD);
    }

    // 根据 delta 符号选择滤波系数（运动方向）
    float lpf_alpha = (delta >= 0) ? APS_LPF_ALPHA_FORWARD : APS_LPF_ALPHA_REVERSE;

    // 一阶滤波
    float lpf_speed = LowPassFilter(last_speed, raw_speed, lpf_alpha);
     // motor->measurement.speed_aps = lpf_speed;
    // 滑动平均
    motor->measurement.speed_aps = SlidingAverageFilter(
        motor->measurement.aps_buffer,
        APS_BUFFER_SIZE,
        lpf_speed,
        &motor->measurement.aps_index); // Updated to use motor->measurement.aps_index

    // 同步线速度
    motor->measurement.linear_speed = (motor->measurement.speed_aps / 360.0f) * WHEEL_CIRCUMFERENCE_M; // Updated to use motor->measurement.linear_speed

    // 累计角度与圈数
    motor->measurement.total_angle += angle_increment;
    motor->measurement.total_round = (int32_t)(motor->measurement.total_angle / 360.0f);

    // 方向判断（0 正转，1 反转）
    motor->measurement.direction = (delta >= 0) ? 0 : 1;

    // 重载守护进程
    DaemonReload(motor->daemon);
}

/**
 * @brief 获取电机累计编码器值（长距离计数）
 *
 * @param motor 电机实例指针
 * @return int32_t 当前累计编码器值
 */
int32_t WheelMotorGetTotalEncoder(WheelMotor_Instance *motor)
{
    return motor->measurement.encoder_total_count;
}