#pragma once
#include "stdint.h"
#include "main.h"

//机器人核心控制任务初始化,会被RobotInit()调用
void RobotCMDInit(void);

//机器人核心控制任务,会被RobotTask()调用
void RobotCMDTask(void);