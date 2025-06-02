#include "chassis.h"
#include "robot_def.h"

#include "message_center.h"
#include "wheelmotor.h"
#include "JY901S.h"
#include "general_def.h"
#include "chassis_speed_kf.h"
#include "controller.h"
#include "stdbool.h"

#include "bsp_dwt.h"
#include "arm_math.h"

#define WHEEL_BASE 0.24f // 轮距，单位：米

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;       // 从 robot_cmd 接收到的控制指令  
static Chassis_Upload_Data_s chassis_upload_data; // 上传到 robot_cmd 的反馈数据  

static Publisher_t *chassis_upload_pub;                                          // 上传主题的发布者  
static Subscriber_t *chassis_cmd_sub;                                            // 指令主题的订阅者  
static JY901S_attitude_t *attitude = NULL;                                       // 姿态传感器变量声明（缺失补充）  
static WheelMotor_Instance *motor_l, *motor_r;                                   // 左右轮电机实例  
static float chassis_vx, chassis_wz;                                             // 机器人线速度和角速度  
static float wheel_l_ref, wheel_r_ref;                                           // 左右轮的目标速度参考值  
static float real_vx, real_wz;                                                   // 机器人实际线速度和角速度  
static float wheel_l_speed, wheel_r_speed, wheel_l_speed_aps, wheel_r_speed_aps; // 左右轮速度及其角速度  
static ChassisSpeedKF_t gSpeedKF;                                                // 速度卡尔曼滤波器实例  
static uint32_t dwt_last = 0;                                                    // 上一次速度估计的时间戳  

static PID_Instance yaw_pid;
static float target_yaw, yaw_error;
static bool yaw_lock = false;

static void EstimateSpeed(void);

void ChassisInit()
{
    attitude = INS_Init(); // Initialize the JY901S sensor
    // TODO: Initialize the chassis motors and sensors here
    // Motor_Init_Config_s motor_config = {
    //     .motor_type                   = WheelMotor,
    // };
    WheelMotor_Init_Config_s wheelmotor_config_r = {
        .controller_param_init_config = {
            .speed_PID_forward = {
                .Kp = 1.4f,
                .Ki = 0.01f,
                .Kd = 0.00002f,
                // .Derivative_LPF_RC = 0.025,
                .IntegralLimit = 250.f,
                .MaxOut        = 750.0f, // max aps
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
            .speed_PID_reverse = {
                .Kp                = 0.045f,
                .Ki                = 0.0f,
                .Kd                = 0.0001f,
                .IntegralLimit     = 50.f,
                .Derivative_LPF_RC = 0.002f,
                .MaxOut            = 750.0f,
                .DeadBand          = 10.f,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
            }},
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED, //选择电机作为电机控制的速度反馈 wheelmotor.c中
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .pwm_init_config = {
            .htim      = &htim1,
            .channel   = TIM_CHANNEL_1,
            .period    = 0.02f, // 20ms
            .dutyratio = 0.f,   // 50% duty cycle
            .callback  = NULL,  // No callback for now
            .id        = NULL,  // No ID for now
            .is_N      = 1,
        },
        .encoder_init_config = {
            .htim = &htim5,
        },
        .gpio_init_config = {
            .GPIOx     = GPIOE,
            .GPIO_Pin  = GPIO_PIN_7,
            .pin_state = GPIO_PIN_RESET,
        },
    };

    motor_r = WheelMotorInit(&wheelmotor_config_r); // Initialize the left wheel motor

    WheelMotor_Init_Config_s wheelmotor_config_l = {
        .controller_param_init_config = {
            .speed_PID_forward = {
                .Kp = 1.5f,
                .Ki = 0.025f,
                .Kd = 0.00002f,
                // .Derivative_LPF_RC = 0.025,
                .IntegralLimit = 300.f,
                .MaxOut        = 750.0f, // max aps
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
            .speed_PID_reverse = {
                .Kp                = 0.04f,
                .Ki                = 0.0f,
                .Kd                = 0.0001f,
                .IntegralLimit     = 30.f,
                .Derivative_LPF_RC = 0.002f,
                .MaxOut            = 750.0f,
                .DeadBand          = 15.f,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
            }},
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .pwm_init_config = {
            .htim      = &htim1,
            .channel   = TIM_CHANNEL_3,
            .period    = 0.02f, // 20ms
            .dutyratio = 0.f,   // 50% duty cycle
            .callback  = NULL,  // No callback for now
            .id        = NULL,  // No ID for now
            .is_N      = 0,
        },
        .encoder_init_config = {
            .htim = &htim2,
        },
        .gpio_init_config = {
            .GPIOx     = GPIOE,
            .GPIO_Pin  = GPIO_PIN_12,
            .pin_state = GPIO_PIN_RESET,
        },
    };

    motor_l = WheelMotorInit(&wheelmotor_config_l); // Initialize the right wheel motor

    ChassisSpeedKF_Init(&gSpeedKF,
                        0.1f,  // q_vx
                        0.1f,  // q_wz
                        0.05f, // r_vx
                        0.05f, // r_wzEnc
                        0.02f  // r_wzImu
    );

    PID_Init_Config_s yaw_pid_config = {
        .Kp            = 0.1f,
        .Ki            = 0.001f,
        .Kd            = 0.0005f,
        .MaxOut        = 1.5f,
        .IntegralLimit = 2.0f,
        .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .DeadBand      = 0.1f // 允许小范围偏航不纠正（单位：度）
    };
    PIDInit(&yaw_pid, &yaw_pid_config);

    chassis_cmd_sub    = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));       // Subscribe to the command topic
    chassis_upload_pub = PubRegister("chassis_fetch", sizeof(Chassis_Upload_Data_s)); // Register the upload topic
}

// The core function of the chassis task
void ChassisTask(void)
{
    SubGetMessage(chassis_cmd_sub, &chassis_cmd_recv); // Get the command from the robot_cmd

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        // TODO: Motor stop
        WheelMotorStop(motor_l);
        WheelMotorStop(motor_r);
    } else {
        // TODO: Motor Enable
        WheelMotorEnable(motor_l);
        WheelMotorEnable(motor_r);
    }

    chassis_vx = chassis_cmd_recv.vx; // Get the forward speed from the command
    chassis_wz = chassis_cmd_recv.wz; // Get the angular speed from the command

    // TODO: Control the motors based on the chassis_vx and chassis_wz values
    // TODO: Get the real speed and battery level from the motors and sensors

    // 1. 线速度 -> 左右轮线速度

    if (chassis_vx > 0.01f && fabsf(chassis_wz) < 0.01f) {
        if (!yaw_lock) {
            target_yaw = attitude->YawTotalAngle; // 锁定初始角度
            yaw_lock   = true;
        }

        yaw_error               = target_yaw - attitude->YawTotalAngle;
        float wz_yaw_correction = PIDCalculate(&yaw_pid, 0.0f, yaw_error); // 目标角度为0误差
        chassis_wz              = -wz_yaw_correction;
    } else {
        yaw_lock = false; // 有旋转指令，不锁定角度 or RobotStop reset the yaw traget
    }

    // 用陀螺仪实际角速度进行 PID 修正

    float v_l = chassis_vx - (chassis_wz * WHEEL_BASE / 2.0f);
    float v_r = chassis_vx + (chassis_wz * WHEEL_BASE / 2.0f);

    // 2. 线速度 -> 角速度（rad/s）-> deg/s
    wheel_l_ref = (v_l / WHEEL_RADIUS) * RAD_2_DEGREE;
    wheel_r_ref = -(v_r / WHEEL_RADIUS) * RAD_2_DEGREE;

    WheelMotorSetRef(motor_l, wheel_l_ref); // Set the reference speed for the left wheel
    WheelMotorSetRef(motor_r, wheel_r_ref); // Set the reference speed for the right wheel

    // 3. 估算速度
    EstimateSpeed(); // Estimate the speed of the robot

    PubPushMessage(chassis_upload_pub, (void *)&chassis_upload_data); // Publish the upload data
}

static void EstimateSpeed(void)
{
    // 1) 得到编码器的线速度、角速度 (左右轮速度合成)
    wheel_l_speed     = motor_l->measurement.linear_speed;
    wheel_r_speed     = -motor_r->measurement.linear_speed;
    wheel_l_speed_aps = motor_l->measurement.speed_aps;
    wheel_r_speed_aps = -motor_r->measurement.speed_aps;
    float vEnc        = 0.5f * (wheel_l_speed + wheel_r_speed);
    float wEnc        = (wheel_r_speed - wheel_l_speed) / WHEEL_BASE;

    // 2) 从 IMU 获取陀螺仪Z轴角速度 (若是 JY901S_attitude_t, 取 Gyro[2] 即可)
    //    视情况确认陀螺仪三轴排列
    float wImu = attitude->Gyro[2] * DEGREE_2_RAD; // 例如Z轴
    // ...

    // 3) 计算本次 dt
    float dt = DWT_GetDeltaT(&dwt_last);

    // 4) 调用 KF 更新
    ChassisSpeedKF_Update(&gSpeedKF, vEnc, wEnc, wImu, dt);

    // 5) 读取滤波结果
    float vx_f, wz_f;
    ChassisSpeedKF_GetEstimate(&gSpeedKF, &vx_f, &wz_f);

    // 6) 输出到您需要的位置 (例如 chassis_upload_data 或调试打印)
    // ...
    chassis_upload_data.real_vx = vx_f;
    chassis_upload_data.real_wz = wz_f;
    chassis_upload_data.dt      = dt;
    real_vx                     = vx_f;
    real_wz                     = wz_f;
}