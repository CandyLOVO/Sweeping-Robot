/**
 * @file cmd.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief Command definitions for robot control，Usage for communication with PC.
 * @version 0.1
 * @date 2025-04-10
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include "stdint.h"
#include "main.h"

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit(void);

/**
 * @brief 机器人核心控制任务,会被RobotTask()调用
 *
 */
void RobotCMDTask(void);