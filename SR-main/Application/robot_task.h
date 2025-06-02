/**
 * @file robot_task.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief The Task for the robot's freeRTOS
 * @version 0.1
 * @date 2025-04-04
 *
 * @copyright Copyright (c) 2025
 *
 */

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

const osThreadAttr_t StartRobotTask_attributes = {
    .name       = "StartRobotTask",
    .stack_size = 1024 * 4, // Multiply by 4 to convert to bytes
    .priority   = (osPriority_t)osPriorityAboveNormal,
};
const osThreadAttr_t StartLEDTask_attributes = {
    .name       = "StartLEDTask",
    .stack_size = 128 * 4, // Multiply by 4 to convert to bytes
    .priority   = (osPriority_t)osPriorityNormal,
};
const osThreadAttr_t StartMotorTask_attributes = {
    .name       = "StartMotorTask",
    .stack_size = 1024 * 4, // Multiply by 4 to convert to bytes
    .priority   = (osPriority_t)osPriorityNormal,
};
const osThreadAttr_t StartSensorTask_attributes = {
    .name       = "StartSensorTask",
    .stack_size = 128 * 4, // Multiply by 4 to convert to bytes
    .priority   = (osPriority_t)osPriorityNormal,
};
const osThreadAttr_t StartDAEMONTASK_attributes = {
    .name       = "StartDAEMONTASK",
    .stack_size = 128 * 4, // Multiply by 4 to convert to bytes
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

/**
 * @brief The main task for the robot,
 * this task will be used to control the robot and handle the messages from PC.
 *
 * @param argument
 */
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

/**
 * @brief The task for the LED,
 * this task will be used to control the LED and handle the messages from cmd.
 *
 * @param argument
 */
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

/**
 * @brief The task for the motor,
 * this task will be used to control the motor and read the encoder.
 *
 * @param argument
 */
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

/**
 * @brief The task for the sensor,
 * this task will be used to control the sensor and read the data.
 *
 * @param argument
 */
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

/**
 * @brief 守护线程任务,100Hz,相当于看门狗
 *
 */
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
