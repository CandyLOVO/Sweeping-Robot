#pragma once
#include <stdbool.h>

typedef struct {
    float x;   // m
    float y;   // m
    float yaw; // deg
} Pose2D_t;

typedef struct {
    float vx; // m/s
    float wz; // rad/s
} VelocityCmd_t;

/* 初始化，设定初始位姿 (一般 0) */
void GotoCtrl_Init(const Pose2D_t *init_pose);

/* 更新目标点 */
void GotoCtrl_SetGoal(float x, float y);

/* 运行控制器：输入当前观测 & dt，输出底盘速度指令 */
VelocityCmd_t GotoCtrl_Step(float real_vx, float real_wz,
                            float imu_yaw_deg, float dt_s);

/* 查询是否到点 */
bool GotoCtrl_IsArrived(void);
