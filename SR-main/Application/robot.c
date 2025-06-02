#include "robot.h"
#include "robot_task.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "chassis.h"

#include "bsp_init.h"
#include "LED.h"

//初始化机器人的硬件和外设
void Robot_Init(void)
{
    // 初始化机器人硬件和外设
    // 此函数应在启动机器人任务前调用
    // 确保所有硬件准备就绪
    __disable_irq(); // 禁用中断以确保原子操作

    // 初始化板级支持包
    BSPInit();

    // 初始化应用层
    RobotCMDInit();      // 初始化机器人命令处理
    ChassisInit();       // 初始化底盘控制

    // 初始化FreeRTOS任务
    OSTaskInit();

    __enable_irq(); // 初始化完成后启用中断
}

//机器人主控制任务入口
void RobotTask(void)
{
    RobotCMDTask();      // 处理命令解析任务
    ChassisTask();       // 底盘运动控制任务
}