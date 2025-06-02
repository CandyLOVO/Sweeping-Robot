#pragma once

#include "REG.h"
#include "wit_c_sdk.h"
#include "stdint.h"
#include "bsp_usart.h"

#define RXBUFFER_LEN 66

typedef struct {
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    // 还需要增加角速度数据
    float Mag[3];        // 磁场
    float Quaternion[4]; // 四元数
    float Angle[3];      // 欧拉角
    float Roll_Angle;
    float Pitch_Angle;
    float Yaw_Angle;
    float YawTotalAngle;

    float _last_yaw;
    float _yaw_total;
    uint8_t yaw_inited;

} JY901S_attitude_t;

typedef struct {
    uint8_t rx_len;
    uint8_t frame_head; // 帧头
    uint8_t rx_buffer[RXBUFFER_LEN];
} JY901S_RX_DATA_t;

typedef struct {
    uint8_t is_init;
    USART_Instance *usart_instance;
    JY901S_RX_DATA_t rx_data;
    JY901S_attitude_t attitude;
} JY901S_Instance;

JY901S_attitude_t *INS_Init(void);