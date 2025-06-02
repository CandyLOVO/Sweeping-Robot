#pragma once

//初始化机器人所有硬件模块和外设
void Robot_Init(void);

//负责处理实时控制逻辑、传感器数据融合
void RobotTask(void);