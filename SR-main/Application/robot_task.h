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

osThreadId_t StartRobotTaskHandle;
osThreadId_t StartLEDTaskHandle;
osThreadId_t StartMotorTaskHandle;
osThreadId_t StartSensorTaskHandle;
osThreadId_t StartDaemonTaskHandle;

// 机器人主任务线程,高于正常优先级
const osThreadAttr_t StartRobotTask_attributes = {
    .name       = "StartRobotTask",
    .stack_size = 1024 * 4, // 乘以4转换为字节（4KB堆栈）
    .priority   = (osPriority_t)osPriorityAboveNormal,
};

// LED控制任务线程,正常优先级
const osThreadAttr_t StartLEDTask_attributes = {
    .name       = "StartLEDTask",
    .stack_size = 128 * 4, // 乘以4转换为字节（0.5KB堆栈）
    .priority   = (osPriority_t)osPriorityNormal,
};

// 电机控制任务线程,正常优先级
const osThreadAttr_t StartMotorTask_attributes = {
    .name       = "StartMotorTask",
    .stack_size = 1024 * 4, // 乘以4转换为字节（4KB堆栈）
    .priority   = (osPriority_t)osPriorityNormal,
};

// 传感器采集任务线程,正常优先级
const osThreadAttr_t StartSensorTask_attributes = {
    .name       = "StartSensorTask",
    .stack_size = 128 * 4, // 乘以4转换为字节（0.5KB堆栈）
    .priority   = (osPriority_t)osPriorityNormal,
};

// 守护进程任务线程,正常优先级
const osThreadAttr_t StartDAEMONTASK_attributes = {
    .name       = "StartDAEMONTASK",
    .stack_size = 128 * 4, // 乘以4转换为字节（0.5KB堆栈）
    .priority   = (osPriority_t)osPriorityNormal,
};

void StartRobotTask(void *argument);
void StartLEDTask(void *argument);
void StartMotorTask(void *argument);
void StartSensorTask(void *argument);
void StartDAEMONTASK(void *argument);

/**
 * @brief Intialize the FreeRTOS tasks, all tasks should be created here.
 *
 */
void OSTaskInit(void)
{
    StartRobotTaskHandle  = osThreadNew(StartRobotTask, NULL, &StartRobotTask_attributes);
    StartLEDTaskHandle    = osThreadNew(StartLEDTask, NULL, &StartLEDTask_attributes);
    StartMotorTaskHandle  = osThreadNew(StartMotorTask, NULL, &StartMotorTask_attributes);
    StartSensorTaskHandle = osThreadNew(StartSensorTask, NULL, &StartSensorTask_attributes);
    StartDaemonTaskHandle = osThreadNew(StartDAEMONTASK, NULL, &StartDAEMONTASK_attributes);
}

//机器人的整体控制及与上位机的通信处理
void StartRobotTask(void *argument)
{
    /* USER CODE BEGIN StartRobotTask */
    INS_Init(); // Initialize the JY901S instance here if needed
    /* Infinite loop */
    for (;;) {
        RobotTask(); // Call the robot task function here
        osDelay(5);
    }
    /* USER CODE END StartRobotTask */
}

//LED状态控制及命令处理
void StartLEDTask(void *argument)
{
    /* USER CODE BEGIN StartLEDTask */
    LEDInit(); // Initialize the LED instances here if needed
    /* Infinite loop */
    for (;;) {
        LEDTask(); // Call the LED task function here
        osDelay(10);
    }
    /* USER CODE END StartLEDTask */
}

//电机驱动及编码器数据读取
void StartMotorTask(void *argument)
{

    /* USER CODE BEGIN StartMotorTask */
    /* Infinite loop */
    for (;;) {
        MotorControlTask();
        osDelay(2);
    }
    /* USER CODE END StartMotorTask */
}

//传感器控制及数据读取
void StartSensorTask(void *argument)
{
    /* USER CODE BEGIN StartSensorTask */
    SensorInit(); // Initialize the sensor instances here if needed
    /* Infinite loop */
    for (;;) {
        SensorTask();
        osDelay(2);
    }
    /* USER CODE END StartSensorTask */
}

// 守护线程任务,100Hz,相当于看门狗
void StartDAEMONTASK(void *argument)
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
