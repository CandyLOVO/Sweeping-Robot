//对机器人线速度 (vx) 与角速度 (wz) 进行卡尔曼滤波

#include "speed_kf.h"
#include <string.h>

void ChassisSpeedKF_Init(ChassisSpeedKF_t *kfObj,
                         float q_vx,
                         float q_wz,
                         float r_vx,
                         float r_wzEnc,
                         float r_wzImu)
{
    //初始化 KF：xhatSize=2, uSize=0, zSize=3
    Kalman_Filter_Init(&kfObj->kf, 2, 0, 3);

    //不使用自动调整(UseAutoAdjustment=0)，我们手动设 H、R
    kfObj->kf.UseAutoAdjustment = 0;

    //状态转移矩阵 F (2×2)，先设单位阵 (假设短时内 vx,wz 不发生显著耦合)
    float F[4] = {1, 0,
                  0, 1};
    memcpy(kfObj->kf.F_data, F, sizeof(F));

    float H[6] = {
        1.f, 0.f,
        0.f, 1.f,
        0.f, 1.f};
    memcpy(kfObj->kf.H_data, H, sizeof(H));

    //初始 P (2×2)，可适度设置大些
    float P[4] = {1.0f, 0.0f,
                  0.0f, 1.0f};
    memcpy(kfObj->kf.P_data, P, sizeof(P));

    //记录下用户的噪声参数
    kfObj->q_vx    = q_vx;
    kfObj->q_wz    = q_wz;
    kfObj->r_vx    = r_vx;
    kfObj->r_wzEnc = r_wzEnc;
    kfObj->r_wzImu = r_wzImu;

    //填充初始 Q (2×2) 与 R (3×3)
    memset(kfObj->kf.Q_data, 0, 4 * sizeof(float));
    kfObj->kf.Q_data[0] = q_vx; // Q(0,0)
    kfObj->kf.Q_data[3] = q_wz; // Q(1,1)

    //diag(r_vx, r_wzEnc, r_wzImu)
    memset(kfObj->kf.R_data, 0, 9 * sizeof(float));
    kfObj->kf.R_data[0] = r_vx;    // R(0,0)
    kfObj->kf.R_data[4] = r_wzEnc; // R(1,1)
    kfObj->kf.R_data[8] = r_wzImu; // R(2,2)

    //MeasurementMap若不使用自动调整，其实用处不大，但可辅助调试
    kfObj->kf.MeasurementMap[0] = 1; // z0-> x[0]
    kfObj->kf.MeasurementMap[1] = 2; // z1-> x[1]
    kfObj->kf.MeasurementMap[2] = 2; // z2-> x[1] (IMU)

    // 初值
    kfObj->xhat[0] = 0.0f;
    kfObj->xhat[1] = 0.0f;
    kfObj->dt      = 0.0f;
}

/**
 * @brief 每次循环调用：更新滤波
 */
void ChassisSpeedKF_Update(ChassisSpeedKF_t *kfObj,
                           float measured_vx,
                           float measured_wzE, // encoder
                           float measured_wzI, // IMU
                           float dt)
{
    kfObj->dt = dt;

    //动态更新 Q, R 等(若需要根据dt修正)
    kfObj->kf.Q_data[0] = kfObj->q_vx * dt;
    kfObj->kf.Q_data[3] = kfObj->q_wz * dt;

    //填写测量向量 z (3×1)
    kfObj->kf.MeasuredVector[0] = measured_vx;
    kfObj->kf.MeasuredVector[1] = measured_wzE;
    kfObj->kf.MeasuredVector[2] = measured_wzI;

    //卡尔曼更新
    float *filtered = Kalman_Filter_Update(&kfObj->kf);

    //拿到后验估计 xhat(k)
    kfObj->xhat[0] = filtered[0]; // vx
    kfObj->xhat[1] = filtered[1]; // wz
}
