/**
 * @file    chassis_speed_kf.h
 * @brief   对机器人线速度 (vx) 和角速度 (wz) 进行卡尔曼滤波估计（支持添加陀螺仪wz观测）
 */

#ifndef __CHASSIS_SPEED_KF_H
#define __CHASSIS_SPEED_KF_H

#include "kalman_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief KF 对象
 *
 * 状态维度: 2  ( x = [ vx, wz ] )
 * 测量维度: 3  ( z = [ vxEnc, wzEnc, wzImu ] )
 */
typedef struct
{
    KalmanFilter_t kf; // 复用已有的 KalmanFilter_t

    float dt;      // 采样周期 (秒)
    float xhat[2]; // 估计量 [vx, wz]

    // 配置：过程噪声、测量噪声等
    float q_vx;    // vx 的过程噪声系数
    float q_wz;    // wz 的过程噪声系数
    float r_vx;    // 编码器测得的 vx 的测量噪声
    float r_wzEnc; // 编码器测得的 wz 的测量噪声
    float r_wzImu; // IMU 陀螺仪测得的 wz 的测量噪声
} ChassisSpeedKF_t;

/**
 * @brief  初始化底盘速度 KF
 * @param  kfObj       KF 对象
 * @param  q_vx        vx 的过程噪声
 * @param  q_wz        wz 的过程噪声
 * @param  r_vx        编码器测 vx 的测量噪声
 * @param  r_wzEnc     编码器测 wz 的测量噪声
 * @param  r_wzImu     陀螺仪测 wz 的测量噪声
 */
void ChassisSpeedKF_Init(ChassisSpeedKF_t *kfObj,
                         float q_vx,
                         float q_wz,
                         float r_vx,
                         float r_wzEnc,
                         float r_wzImu);

/**
 * @brief  每次循环调用：更新 KF
 * @param  kfObj        KF 对象
 * @param  measured_vx  编码器测量得到的线速度
 * @param  measured_wzE 编码器测量得到的角速度
 * @param  measured_wzI IMU 陀螺仪测得的角速度
 * @param  dt           本次与上次的时间间隔（秒）
 */
void ChassisSpeedKF_Update(ChassisSpeedKF_t *kfObj,
                           float measured_vx,
                           float measured_wzE,
                           float measured_wzI,
                           float dt);

/**
 * @brief  获取当前的滤波估计
 * @param  kfObj   KF 对象
 * @param  vxOut   输出线速度
 * @param  wzOut   输出角速度
 */
static inline void ChassisSpeedKF_GetEstimate(ChassisSpeedKF_t *kfObj, float *vxOut, float *wzOut)
{
    if (vxOut) *vxOut = kfObj->xhat[0];
    if (wzOut) *wzOut = kfObj->xhat[1];
}

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_SPEED_KF_H */
