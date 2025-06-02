#include "goto_controller.h"
#include "controller.h"
#include <math.h>

#define DEG2RAD        0.0174532925f
#define RAD2DEG        57.2957795f
#define CLAMP(x, l, h) (((x) < (l)) ? (l) : ((x) > (h) ? (h) : (x)))

//状态枚举
typedef enum {
    TURN_TO,
    DRIVE_TO,
    ARRIVED
} GoToState_e;

//内部状态
static Pose2D_t odom     = {0}; // 实时位置估计
static Pose2D_t goal     = {0}; // 当前目标
static GoToState_e state = ARRIVED;

// 参数
static const float yaw_eps  = 10.0f; // 转向结束阈值 (deg)
static const float dist_eps = 0.03f; // 到点距离阈值 (m)

//PID 控制器
static PID_Instance dist_pid;
static PID_Instance yaw_pid;

//工具函数
static inline float angle_diff_deg(float a, float b)
{
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static void odom_update(float vx, float dt_s, float imu_yaw_deg)
{
    float yaw_rad = odom.yaw * DEG2RAD;
    odom.x += vx * cosf(yaw_rad) * dt_s;
    odom.y += vx * sinf(yaw_rad) * dt_s;
    odom.yaw = imu_yaw_deg;
}

//接口函数
void GotoCtrl_Init(const Pose2D_t *init_pose)
{
    odom  = *init_pose;
    state = ARRIVED;

    static PID_Init_Config_s dist_cfg = {
        .Kp            = 0.8f,
        .Ki            = 0.001f,
        .Kd            = 0.0f,
        .MaxOut        = 0.15f, // 线速度上限 (m/s)
        .IntegralLimit = 0.2f,
        .DeadBand      = 0.0f,
        .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit};

    static PID_Init_Config_s yaw_cfg = {
        .Kp            = 0.1f,
        .Ki            = 0.0002f,
        .Kd            = 0.0008f,
        .MaxOut        = 1.0f, // 角速度上限 (rad/s)
        .IntegralLimit = 0.5f,
        .DeadBand      = 0.5f, // 航向死区 (deg)
        .Improve       = PID_Trapezoid_Intergral | PID_DerivativeFilter |
                   PID_Integral_Limit | PID_Derivative_On_Measurement};

    PIDInit(&dist_pid, &dist_cfg);
    PIDInit(&yaw_pid, &yaw_cfg);
}

void GotoCtrl_SetGoal(float x, float y)
{
    goal.x = x;
    goal.y = y;
    // state  = TURN_TO;
    state  = DRIVE_TO;
}

VelocityCmd_t GotoCtrl_Step(float real_vx, float real_wz,
                            float imu_yaw_deg, float dt_s)
{
    VelocityCmd_t cmd = {0};

    // 里程计更新
    odom_update(real_vx, dt_s, imu_yaw_deg);

    //误差计算
    float dx      = goal.x - odom.x;
    float dy      = goal.y - odom.y;
    float dist    = sqrtf(dx * dx + dy * dy);
    float tgt_yaw = atan2f(dy, dx) * RAD2DEG;
    float yaw_err = angle_diff_deg(tgt_yaw, odom.yaw);

    switch (state) {
        case TURN_TO: {
            float wz_cmd = PIDCalculate(&yaw_pid, 0.0f, yaw_err);
            cmd.wz       = -wz_cmd;

            if (fabsf(yaw_err) < yaw_eps &&
                fabsf(wz_cmd) < 0.05f)
                break;
        }

        case DRIVE_TO: {
            float vx_cmd = PIDCalculate(&dist_pid, 0.0f, dist);
            float wz_cmd = PIDCalculate(&yaw_pid, 0.0f, yaw_err);

            float yaw_scale = cosf(fabsf(yaw_err) * DEG2RAD); // 大偏角自动减速
            vx_cmd *= yaw_scale;

            cmd.vx = vx_cmd;
            cmd.wz = -wz_cmd;

            if (dist < dist_eps && fabsf(vx_cmd) < 0.05f)
                state = ARRIVED;
            break;
        }

        case ARRIVED:
            cmd.vx = 0.0f;
            cmd.wz = 0.0f;
            break;
    }

    return cmd;
}

bool GotoCtrl_IsArrived(void)
{
    return state == ARRIVED;
}
