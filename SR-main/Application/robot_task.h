//FreeRTOS 系统多任务管理接口定义、初始化配置
#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "robot.h"
#include "led_task.h"
#include "motor_task.h"
#include "daemon.h"
#include "JY901S.h"
#include "sensor.h"

osThreadId_t RobotTaskHandle;
osThreadId_t LEDTaskHandle;
osThreadId_t MotorTaskHandle;
osThreadId_t SensorTaskHandle;
osThreadId_t DaemonTaskHandle;

// 机器人主任务线程配置（高于正常优先级）
const osThreadDef_t RobotTask_attributes = {
    .name = "RobotTask",
    .pthread = StartRobotTask,    // 线程入口函数
    .tpriority = osPriorityAboveNormal,
    .instances = 1,
    .stacksize = 1024 * 4         // 堆栈大小（字节）
};

// LED控制任务线程配置（正常优先级）
const osThreadDef_t LEDTask_attributes = {
    .name = "LEDTask",
    .pthread = StartLEDTask,      // 线程入口函数
    .tpriority = osPriorityNormal,
    .instances = 1,
    .stacksize = 128 * 4          // 堆栈大小（字节）
};

// 电机控制任务线程配置（正常优先级）
const osThreadDef_t MotorTask_attributes = {
    .name = "MotorTask",
    .pthread = StartMotorTask,    // 线程入口函数
    .tpriority = osPriorityNormal,
    .instances = 1,
    .stacksize = 1024 * 4         // 堆栈大小（字节）
};

// 传感器采集任务线程配置（正常优先级）
const osThreadDef_t SensorTask_attributes = {
    .name = "SensorTask",
    .pthread = StartSensorTask,   // 线程入口函数
    .tpriority = osPriorityNormal,
    .instances = 1,
    .stacksize = 128 * 4          // 堆栈大小（字节）
};

// 守护进程任务线程配置（正常优先级）
const osThreadDef_t DaemonTask_attributes = {
    .name = "DaemonTask",
    .pthread = StartDAEMONTASK,   // 线程入口函数
    .tpriority = osPriorityNormal,
    .instances = 1,
    .stacksize = 128 * 4          // 堆栈大小（字节）
};

void RobotTask(void *argument);
void LEDTask(void *argument);
void MotorTask(void *argument);
void SensorTask(void *argument);
void DaemonTask(void *argument);

void OSTaskInit(void)
{
    RobotTaskHandle  = osThreadNew(RobotTask, NULL, &RobotTask_attributes);
    LEDTaskHandle    = osThreadNew(LEDTask, NULL, &LEDTask_attributes);
    MotorTaskHandle  = osThreadNew(MotorTask, NULL, &MotorTask_attributes);
    SensorTaskHandle = osThreadNew(SensorTask, NULL, &SensorTask_attributes);
    DaemonTaskHandle = osThreadNew(DaemonTask, NULL, &DaemonTask_attributes);
}

//机器人的整体控制及与上位机的通信处理
void RobotTask(void *argument)
{
    /* USER CODE BEGIN RobotTask */
    INS_Init(); // Initialize the JY901S instance here if needed
    /* Infinite loop */
    for (;;) {
        RobotTask(); // Call the robot task function here
        osDelay(5);
    }
    /* USER CODE END RobotTask */
}

//LED状态控制及命令处理
void LEDTask(void *argument)
{
    /* USER CODE BEGIN LEDTask */
    LEDInit(); // Initialize the LED instances here if needed
    /* Infinite loop */
    for (;;) {
        LEDTask(); // Call the LED task function here
        osDelay(10);
    }
    /* USER CODE END LEDTask */
}

//电机驱动及编码器数据读取
void MotorTask(void *argument)
{

    /* USER CODE BEGIN MotorTask */
    /* Infinite loop */
    for (;;) {
        MotorControlTask();
        osDelay(2);
    }
    /* USER CODE END MotorTask */
}

//传感器控制及数据读取
void SensorTask(void *argument)
{
    /* USER CODE BEGIN SensorTask */
    SensorInit(); // Initialize the sensor instances here if needed
    /* Infinite loop */
    for (;;) {
        SensorTask();
        osDelay(2);
    }
    /* USER CODE END SensorTask */
}

// 守护线程任务,100Hz,相当于看门狗
void DaemonTask(void *argument)
{
    static float daemon_dt __attribute__((unused)); // for cancel warning
    static float daemon_start;
    for (;;) {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        osDelay(10);
    }
}
