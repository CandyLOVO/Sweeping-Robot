/**
 * @file robot.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief Header file for robot Initialization and task management
 * @version 0.1
 * @date 2025-04-04
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

/**
 * @brief Initialize the robot's hardware and peripherals.
 *
 */
void Robot_Init(void);

/**
 * @brief The task entry point for the robot.
 *
 */
void RobotTask(void);