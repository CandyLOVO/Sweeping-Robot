#include "JY901S.h"
#include "JY901S_typedef.h"
#include "bsp_usart.h"
#include "stm32f4xx_it.h"
#include "general_def.h"

/* ========== 1. 轴映射配置 ========== */
/*
 * 目标: 让 IMU 输出最终落到机器人 base_link 坐标
 *       base_link 约定:  X 前,  Y 左,  Z 上  (右手系)
 *
 * 填表说明:
 *   AXIS_MAP[i]  : 传感器原始数据中，被映射到目标轴 i 的“源轴索引”
 *   AXIS_SIGN[i] : +1 保留正向，-1 翻转符号
 *
 * 例子: 你的板子正面朝上、USB 口朝前,
 *       但 JY901S 自身是  X→前, Y→右, Z→下
 *       想要得到 base_link (X前,Y左,Z上) 时：
 *         base_X ← +sensor_X  (索引 0, 正号)
 *         base_Y ← -sensor_Y  (索引 1, 反号)
 *         base_Z ← -sensor_Z  (索引 2, 反号)
 */
static const int8_t AXIS_MAP[3] = {0, 1, 2};             // 改这里可调换轴
static const float AXIS_SIGN[3] = {+1.0f, -1.0f, +1.0f}; // 新：只翻 Y，保留 Z 正

/* ========== 2. 通用向量映射宏 ========== */
#define MAP_AXIS(dst, src)                   \
    do {                                     \
        for (int _i = 0; _i < 3; ++_i) {     \
            (dst)[_i] = AXIS_SIGN[_i] *      \
                        (src)[AXIS_MAP[_i]]; \
        }                                    \
    } while (0)

/* ========== 3. 四元数坐标系旋转 ========== */
/* 先把原始四元数转成 (roll,pitch,yaw)，按映射表换轴后再转回四元数 */
#include <math.h>
static void quat_axis_remap(const float q_in[4], float q_out[4])
{
    /* 3‑1 解算欧拉角（右手系，ZYX）*/
    float w = q_in[0], x = q_in[1], y = q_in[2], z = q_in[3];
    float sinr_cosp = 2.f * (w * x + y * z);
    float cosr_cosp = 1.f - 2.f * (x * x + y * y);
    float roll      = atan2f(sinr_cosp, cosr_cosp);

    float sinp  = 2.f * (w * y - z * x);
    float pitch = fabsf(sinp) >= 1.f ? copysignf(M_PI_2, sinp) : asinf(sinp);

    float siny_cosp = 2.f * (w * z + x * y);
    float cosy_cosp = 1.f - 2.f * (y * y + z * z);
    float yaw       = atan2f(siny_cosp, cosy_cosp);

    /* 3‑2 根据 AXIS_MAP / SIGN 调整欧拉角 */
    float rpy_sensor[3] = {roll, pitch, yaw};
    float rpy_base[3];
    MAP_AXIS(rpy_base, rpy_sensor); // roll→X, pitch→Y, yaw→Z

    /* 3‑3 重新转回四元数 (ZYX) */
    float cr = cosf(rpy_base[0] * 0.5f);
    float sr = sinf(rpy_base[0] * 0.5f);
    float cp = cosf(rpy_base[1] * 0.5f);
    float sp = sinf(rpy_base[1] * 0.5f);
    float cy = cosf(rpy_base[2] * 0.5f);
    float sy = sinf(rpy_base[2] * 0.5f);

    q_out[0] = cr * cp * cy + sr * sp * sy; // w
    q_out[1] = sr * cp * cy - cr * sp * sy; // x
    q_out[2] = cr * sp * cy + sr * cp * sy; // y
    q_out[3] = cr * cp * sy - sr * sp * cy; // z
}

#define JY901S_FRAME_LEN  11
#define JY901S_FRAME_HEAD 0x55

static JY901S_Instance *INS = NULL;
static struct SAcc stcAcc;
static struct SGyro stcGyro;
static struct SAngle stcAngle;
static struct SMag stcMag;
static struct SQ stcQ;

extern DMA_HandleTypeDef hdma_usart2_rx;

static void JY901S_Parsedata(void);

JY901S_attitude_t *INS_Init(void)
{
    if (INS == NULL) {
        INS = (JY901S_Instance *)malloc(sizeof(JY901S_Instance));
        memset(INS, 0, sizeof(JY901S_Instance));
        INS->is_init             = 1;
        INS->attitude.yaw_inited = 0;
    } else {
        return &INS->attitude;
    }

    // 1. 初始化串口
    USART_Init_Config_s usart_config = {
        .usart_handle    = &huart2,
        .recv_buff_size  = RXBUFFER_LEN,
        .module_callback = JY901S_Parsedata,
    };
    INS->usart_instance     = USARTRegister(&usart_config);
    INS->rx_data.frame_head = JY901S_FRAME_HEAD;
    return &INS->attitude;
}

static float JY901S_UnwrapYaw(JY901S_attitude_t *att, float current_yaw)
{
    /* 第一次进来：把当前航向当作 0°，不计入累计角 */
    if (!att->yaw_inited) {
        att->_last_yaw  = current_yaw;
        att->_yaw_total = 0.0f;
        att->yaw_inited = 1;
        return 0.0f;
    }

    float delta = current_yaw - att->_last_yaw;

    if (delta > 180.0f)
        delta -= 360.0f;
    else if (delta < -180.0f)
        delta += 360.0f;

    att->_yaw_total += delta;
    att->_last_yaw = current_yaw;
    return att->_yaw_total;
}

static void JY901S_Parsedata(void)
{
    uint32_t temp;
    temp                = hdma_usart2_rx.Instance->NDTR;
    INS->rx_data.rx_len = RXBUFFER_LEN - temp;
    if (INS->rx_data.rx_len < RXBUFFER_LEN) return;
    memcpy(INS->rx_data.rx_buffer, INS->usart_instance->recv_buff, RXBUFFER_LEN);
    for (uint8_t i = 0; i < 9; i++) {
        if (INS->rx_data.rx_buffer[i * JY901S_FRAME_LEN] != INS->rx_data.frame_head) return;
        switch (INS->rx_data.rx_buffer[i * JY901S_FRAME_LEN + 1]) {
            case 0x51:
                memcpy(&stcAcc, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                /* —— 原始：INS->attitude.Accel[j] = ... —— */
                float acc_raw[3];
                for (int j = 0; j < 3; ++j) acc_raw[j] = (float)stcAcc.a[j] / 32768.f * 16.f;
                MAP_AXIS(INS->attitude.Accel, acc_raw);
                break;
            case 0x52:
                memcpy(&stcGyro, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                float gyro_raw[3];
                for (int j = 0; j < 3; ++j) gyro_raw[j] = (float)stcGyro.w[j] / 32768.f * 2000.f;
                MAP_AXIS(INS->attitude.Gyro, gyro_raw);
                break;
            case 0x53:
                memcpy(&stcAngle, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                float eul_raw[3];
                /* JY901S 文档：Roll-Yaw‑Pitch，一定要注意！ */
                eul_raw[0] = (float)stcAngle.Angle[1] / 32768.f * 180.f; // roll
                eul_raw[1] = (float)stcAngle.Angle[0] / 32768.f * 180.f; // yaw
                eul_raw[2] = (float)stcAngle.Angle[2] / 32768.f * 180.f; // pitch
                MAP_AXIS(INS->attitude.Angle, eul_raw);

                INS->attitude.Roll_Angle  = INS->attitude.Angle[0];
                INS->attitude.Yaw_Angle   = INS->attitude.Angle[2];
                INS->attitude.Pitch_Angle = INS->attitude.Angle[1];
                INS->attitude.YawTotalAngle =
                    JY901S_UnwrapYaw(&INS->attitude, INS->attitude.Yaw_Angle);
                break;
            case 0x54:
                memcpy(&stcMag, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                for (uint8_t j = 0; j < 3; j++) { INS->attitude.Mag[j] = (float)stcMag.h[j]; }
                break;
            case 0x59:
                memcpy(&stcQ, &INS->rx_data.rx_buffer[2 + i * JY901S_FRAME_LEN], 8);
                float q_raw[4];
                for (int j = 0; j < 4; ++j) q_raw[j] = (float)stcQ.q[j] / 32768.f;
                float q_base[4];
                quat_axis_remap(q_raw, q_base);
                for (int j = 0; j < 4; ++j) INS->attitude.Quaternion[j] = q_base[j];
                break;
                // TODO: Parse the rest of imu data
            default:
                break;
        }
    }
}