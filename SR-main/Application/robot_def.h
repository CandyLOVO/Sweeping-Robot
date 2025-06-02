#pragma once

#include "stdint.h"
#include "stdbool.h"

// #define VISION_USE_VCP // 是否使用虚拟串口

#define VISION_USE_UART // 是否使用硬件串口

#pragma pack(1) // Code alignment for structure packing

// The state of the robot
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// The mode of the chassis
typedef enum {
    CHASSIS_ZERO_FORCE = 0, // stop mode
    CHASSIS_NORMAL,
    CHASSIS_GOTO_POINT,     // the mode for going to a specific point
} chassis_mode_e;

// the mode of the cleaning machine
typedef enum {
    CLEANING_IDLE = 0, // idle mode
    CLEANING_ACTIVE    // active cleaning mode
} cleaning_mode_e;

typedef enum {
    OBSTACLE_NONE,
    OBSTACLE_DETECTED,
    OBSTACLE_TURNING,
    OBSTACLE_RECOVERY
} Obstacle_State_e;

/* ----------------CMD Application public the control data---------------- */
typedef struct {
    // control part
    float vx; // the forward speed of the robot m/s
    float wz; // the angular speed of the robot rad/s
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s;

/* ----------------The feedback data---------------- */
typedef struct
{
    // the real speed of the robot
    float real_vx;       // the real speed of the robot m/s
    float real_wz;       // the real angular speed of the robot rad/s
    float battery_level; // the battery level of the robot in percentage
    float dt;            // the time interval of the robot in seconds
} Chassis_Upload_Data_s;

typedef struct
{
    // the feedback data of the sensors
    bool cliff_detected[4]; // 每个通道是否检测到悬崖
} Sensor_Upload_Data_s;

typedef struct {
    float w, x, y, z;
} Quaternion_f;

#pragma pack() // 取消压缩