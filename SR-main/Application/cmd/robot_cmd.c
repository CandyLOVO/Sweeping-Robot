#include "robot_cmd.h"
#include "robot_def.h"
#include "goto_controller.h"

#include "key.h"
#include "message_center.h"
#include "miniPC_process.h"
#include "JY901S.h"
#include "cliff.h"
#include "general_def.h"

#include "math.h"
#include <string.h>
#include "bsp_dwt.h"

static Robot_Status_e robot_status = ROBOT_STOP;    // 机器人的状态  
static Obstacle_State_e obs_state  = OBSTACLE_NONE; // 障碍物的状态  

static KEY_Instance *key_l, *key_r; // 左右按键实例  
static Vision_Recv_s *vision_ctrl;  // 视觉接收数据  

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给机器人底盘的控制指令  
static Chassis_Upload_Data_s chassis_fetch_data; // 从机器人底盘获取的数据  
static Sensor_Upload_Data_s sensor_fetch_data;   // 从传感器上传的数据  
static Publisher_t *chassis_cmd_pub;             // 控制指令主题的发布者  
static Publisher_t *robot_state_pub;             // 机器人状态的发布者  
static Subscriber_t *chassis_fetch_sub;          // 控制指令主题的订阅者  
static Subscriber_t *sensor_sub;                  // 传感器数据的订阅者  

static int cliff_trigger_idx     = -1; // 触发的是哪个悬崖传感器  
static float rotate_target_angle = 0.0f; // 目标旋转角度  
static float yaw_start           = 0.0f; // 起始偏航角  
static float yaw_error_cliif     = 0.0f; // 悬崖引起的偏航误差  
static float norm; // 四元数的模长校验  

static char data_pc_start[256]; // PC数据起始缓存区  

static JY901S_attitude_t *attitude_cmd = NULL; // 姿态传感器指令数据  

static Pose2D_t init_pose = {0};               //机器人上电时的初始位置 (0,0,0°)
static void RobotStop(void);                   // 停止机器人的函数  
static void RobotEnableSet(KEY_Instance *key); // 设置机器人使能模式的函数  
static void RobotModeSet(KEY_Instance *key);   // 设置机器人工作模式的函数  
static void ObstacleAvoidance(void);            // 障碍物避让功能函数  
static void RobotGoTo(void);                     // 机器人前往指定点的函数  

static void VisionControl(void);                 // 视觉控制函数  

//机器人核心控制任务初始化,会被RobotInit()调用
void RobotCMDInit(void)
{
    attitude_cmd = INS_Init(); // Initialize the JY901S sensor

    // Initialize the key instances
    KEY_Config_s key_config = {
        .gpio_config = {
            .GPIOx     = KEY_L_GPIO_Port,
            .GPIO_Pin  = KEY_L_Pin,
            .pin_state = GPIO_PIN_SET,
            .exti_mode = GPIO_EXTI_MODE_FALLING,
        },
        .on_press = NULL, //现在不返回
    };
    key_l                           = KEYRegister(&key_config);
    key_config.gpio_config.GPIOx    = KEY_R_GPIO_Port;
    key_config.gpio_config.GPIO_Pin = KEY_R_Pin;
    key_r                           = KEYRegister(&key_config);

    vision_ctrl = VisionInit(&huart1); // 初始化视觉控制
    GotoCtrl_Init(&init_pose); //初始化控制器

    // 注册机器人底盘的控制指令主题  
    chassis_cmd_pub   = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));  
    chassis_fetch_sub = SubRegister("chassis_fetch", sizeof(Chassis_Upload_Data_s));  
    robot_state_pub   = PubRegister("robot_state", sizeof(Robot_Status_e));  
    // 注册传感器的命令主题  
    sensor_sub   = SubRegister("sensor_fetch", sizeof(Sensor_Upload_Data_s)); // 注册悬崖数据主题  
    robot_status = ROBOT_READY;                                               // 设置机器人状态为准备就绪  

    // strcpy(data_pc_start, "AT+RST\r\n\r\n");  
    // HAL_UART_Transmit(&huart3, (uint8_t *)data_pc_start, strlen(data_pc_start), HAL_MAX_DELAY);  

    // DWT_Delay(5);

    // strcpy(data_pc_start, "AT+CWMODE=1\r\n\r\n");
    // HAL_UART_Transmit(&huart3, (uint8_t *)data_pc_start, strlen(data_pc_start), HAL_MAX_DELAY);

    // DWT_Delay(5);

    // strcpy(data_pc_start, "AT+CIPSTART=\"TCP\",\"192.168.215.165\",8080\r\n\r\n");
    // HAL_UART_Transmit(&huart3, (uint8_t *)data_pc_start, strlen(data_pc_start), HAL_MAX_DELAY);
    // strcpy(data_pc_start, "AT+CIPMODE=0\r\n\r\n");
    // HAL_UART_Transmit(&huart3, (uint8_t *)data_pc_start, strlen(data_pc_start), HAL_MAX_DELAY);
}

void RobotCMDTask(void)
{
    // Get the modules status
    SubGetMessage(chassis_fetch_sub, &chassis_fetch_data); //从机器人底盘获取数据
    SubGetMessage(sensor_sub, &sensor_fetch_data);         //从传感器获取数据

    // strcpy(data_pc_start, "AT+CIPSEND=1\r\n\r\n");
    // HAL_UART_Transmit(&huart3, (uint8_t *)data_pc_start, strlen(data_pc_start), HAL_MAX_DELAY);
    // strcpy(data_pc_start, "1");
    // HAL_UART_Transmit(&huart3, (uint8_t *)data_pc_start, strlen(data_pc_start), HAL_MAX_DELAY);
    // // data_pc = 1;
    // // HAL_UART_Transmit(&huart3, (uint8_t *)data_pc, strlen(data_pc), HAL_MAX_DELAY);

    float dt = chassis_fetch_data.dt;
    //导航模式 CHASSIS_GOTO_POINT
    if (chassis_cmd_send.chassis_mode == CHASSIS_GOTO_POINT &&
        robot_status == ROBOT_READY) // 只有启动车才导航
    {
        VelocityCmd_t out = GotoCtrl_Step(chassis_fetch_data.real_vx,
                                          chassis_fetch_data.real_wz,
                                          attitude_cmd->YawTotalAngle,
                                          dt);

        chassis_cmd_send.vx = out.vx;
        chassis_cmd_send.wz = out.wz;

        //到点即停
        if (GotoCtrl_IsArrived()) {
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        }
    }
    RobotEnableSet(key_l); //左按键控制扫地机运行模式

// TODO: 从PC获取指令并设置vx和wz的值

PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send); // 发布指令到机器人底盘
PubPushMessage(robot_state_pub, (void *)&robot_status);     // 发布机器人状态到PC

// TODO: 向PC发布数据
float vx, wz, ax, ay, az;
vx = chassis_fetch_data.real_vx;
wz = chassis_fetch_data.real_wz;
float q[4] = {attitude_cmd->Quaternion[0],
              +attitude_cmd->Quaternion[1],
              +attitude_cmd->Quaternion[2],
              +attitude_cmd->Quaternion[3]};

ax = attitude_cmd->Accel[0];
ay = attitude_cmd->Accel[1];
az = attitude_cmd->Accel[2];

norm = sqrtf(q[0] * q[0] + q[1] * q[1] +
             +q[2] * q[2] + q[3] * q[3]); // 检查四元数的模长
VisionValueSet(vx, wz, q, ax, ay, az);    // 发送数据到PC

VisionSend(); // 发送数据到PC
}

static uint8_t goal_executed = 0; // 0 = 还没跑过 GOTO, 1 = 已执行

static void RobotEnableSet(KEY_Instance *key)
{
    // 根据按键次数判断模式
    switch (key->count % 2) {
        // 按键次数为偶数时停止机器人
        case 0:
            RobotStop();
            goal_executed = 0;
            break;
        // 按键次数为奇数时启动机器人
        default:
            robot_status = ROBOT_READY; // 设置机器人状态为准备运行

            //以下三种控制方式只选择一种
            RobotModeSet(key_r);  // 右按键控制，偶数次：悬崖传感器及其响应；奇数次：上位机控制
            
            // if (goal_executed == 0) 
            // {
            //     RobotGoTo(); 
            // }

            // VisionControl(); // 上位机控制
            break;
    }
}


__unused static void RobotModeSet(KEY_Instance *key_r)
{

    switch (key_r->count % 2) {
        case 0: /* NORMAL / 避障 */
        //调试模式 CHASSIS_NORMAL
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            ObstacleAvoidance(); //悬崖传感器触发响应
            break;
        case 1: /* 手动直行示例 */
            chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
            VisionControl(); //上位机控制
            break;
    }
}

/**
 * @brief Only run once, can not be used with RobotModeSet
 *
 */
__unused static void RobotGoTo(void)
{
    chassis_cmd_send.chassis_mode = CHASSIS_GOTO_POINT;
    GotoCtrl_SetGoal(1, 0); // Set the goal
    goal_executed = 1;
}

static void RobotStop(void)
{
    robot_status                  = ROBOT_STOP;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.vx           = 0.0f;
    chassis_cmd_send.wz           = 0.0f;
}

static void ObstacleAvoidance(void)
{
    float current_yaw = attitude_cmd->YawTotalAngle; // Get the current yaw angle
    switch (obs_state) {
        case OBSTACLE_NONE: //正常行驶
            //检测是否触发悬崖
            //轮询扫描四个悬崖传感器
            for (int i = 0; i < 4; ++i) {
                if (sensor_fetch_data.cliff_detected[i]) {
                    obs_state         = OBSTACLE_DETECTED; //检测到悬崖
                    cliff_trigger_idx = i;
                    break;
                }
            }
            chassis_cmd_send.vx = 0.15f; //前进速度0.15
            chassis_cmd_send.wz = -0.0f; //旋转角速度0
            break;
        case OBSTACLE_DETECTED: //检测到悬崖
            // 设置转向目标角度
            yaw_start           = current_yaw;
            rotate_target_angle = yaw_start + 180.0f;
            // TODO: Add the direction of the turn
            obs_state = OBSTACLE_TURNING; //原地旋转
            break;
        case OBSTACLE_TURNING: //原地旋转
            // 旋转中（固定角速度）
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.wz = -0.7f; //固定角速度0.7 rad/s原地旋转

            yaw_error_cliif = rotate_target_angle - current_yaw;

            if (fabsf(yaw_error_cliif) < 1.0f) { //偏航角误差小于1°认为“旋转完成”
                obs_state = OBSTACLE_RECOVERY; //恢复前进
            }
            break;
        case OBSTACLE_RECOVERY: //恢复前进
            chassis_cmd_send.vx = 0.15f; //设置前进速度
            chassis_cmd_send.wz = 0.0f; //设置前进角速度
            obs_state           = OBSTACLE_NONE; //正常行驶
            break;
    }
}

static void VisionControl(void)
{
    if (fabsf(vision_ctrl->vx) < 0.01 && fabsf(vision_ctrl->wz) < 0.01) {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    } else {
        chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    }
    //根据视觉控制设置底盘指令
    chassis_cmd_send.vx = vision_ctrl->vx;
    chassis_cmd_send.wz = vision_ctrl->wz;
}