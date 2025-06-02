#include "miniPC_process.h"
#include "string.h"
#include "robot_def.h"
#include "crc_ref.h"
#include "daemon.h"
#include <math.h>

static Vision_Instance *vision_instance; // 用于和视觉通信的串口实例
static uint8_t *vis_recv_buff __attribute__((unused));
static Daemon_Instance *vision_daemon_instance;
// 全局变量区
extern uint16_t CRC_INIT;

/**
 * @brief 处理视觉传入的数据
 *
 * @param recv
 * @param rx_buff
 */
static void RecvProcess(Vision_Recv_s *recv, uint8_t *rx_buff)
{
    /* 使用memcpy接收浮点型小数 */

    memcpy(&recv->checksum, &rx_buff[sizeof(Vision_Recv_s) - 2], sizeof(uint16_t));
    /* CRC校验 */
    if (Verify_CRC16_Check_Sum(rx_buff, sizeof(Vision_Recv_s))) {
        memcpy(&recv->vx, &rx_buff[1], sizeof(float));
        memcpy(&recv->wz, &rx_buff[5], sizeof(float));
    } else {
        // @todo CRC校验失败,可以加入蜂鸣器或者led等提示
    }
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
    // memset(vision_instance->recv_data + 1, 0, sizeof(Vision_Recv_s) - 1);
#ifdef VISION_USE_UART
    USARTServiceInit(vision_instance->usart);
#endif // !VISION_USE_UART
}

void VisionValueSet(float vx, float wz, float *q, float ax, float ay, float az)
{
    vision_instance->send_data->real_vx = vx;
    vision_instance->send_data->real_wz = wz;
    memcpy(vision_instance->send_data->q, q, sizeof(float) * 4);
    vision_instance->send_data->ax = ax;
    vision_instance->send_data->ay = ay;
    vision_instance->send_data->az = az;
}

/**
 * @brief 发送数据处理函数
 *
 * @param send 待发送数据
 * @param tx_buff 发送缓冲区
 *
 */
static void SendProcess(Vision_Send_s *send, uint8_t *tx_buff)
{
    tx_buff[0] = send->header;

    memcpy(&tx_buff[1], &send->real_vx, sizeof(float)); // 1‑4
    memcpy(&tx_buff[5], &send->real_wz, sizeof(float)); // 5‑8
    memcpy(&tx_buff[9], &send->q, sizeof(send->q));     // 9‑24

    /* ===== 新增三行 ===== */
    memcpy(&tx_buff[25], &send->ax, sizeof(float)); // 25‑28
    memcpy(&tx_buff[29], &send->ay, sizeof(float)); // 29‑32
    memcpy(&tx_buff[33], &send->az, sizeof(float)); // 33‑36
    /* ==================== */

    /* 校验位覆盖 0‑36 共 37 字节 */
    send->checksum = Get_CRC16_Check_Sum(tx_buff, sizeof(Vision_Send_s) - 3u, CRC_INIT);
    memcpy(&tx_buff[sizeof(Vision_Send_s) - 3u], &send->checksum, 2); // 37‑38
    memcpy(&tx_buff[sizeof(Vision_Send_s) - 1u], &send->tail, 1);     // 39
}

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config)
{
    Vision_Recv_s *recv_data = (Vision_Recv_s *)malloc(sizeof(Vision_Recv_s));
    memset(recv_data, 0, sizeof(Vision_Recv_s));

    recv_data->header = recv_config->header;

    return recv_data;
}

/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config)
{
    Vision_Send_s *send_data = (Vision_Send_s *)malloc(sizeof(Vision_Send_s));
    memset(send_data, 0, sizeof(Vision_Send_s));

    send_data->header = send_config->header;
    send_data->tail   = send_config->tail;
    return send_data;
}

#ifdef VISION_USE_UART

/**
 * @brief 回调函数，确认帧头后用于解析视觉数据
 *
 */
static void DecodeVision()
{
    if (vision_instance->usart->recv_buff[0] == vision_instance->recv_data->header) {
        // 读取视觉数据
        RecvProcess(vision_instance->recv_data, vision_instance->usart->recv_buff);
    }
}

/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *video_usart_handle)
{
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size  = sizeof(Vision_Recv_s);
    conf.usart_handle    = video_usart_handle;

    vision_instance->usart                = USARTRegister(&conf);
    Vision_Recv_Init_Config_s recv_config = {
        .header = VISION_RECV_HEADER,
    };

    vision_instance->recv_data            = VisionRecvRegister(&recv_config);
    Vision_Send_Init_Config_s send_config = {
        .header = VISION_SEND_HEADER,
        .tail   = VISION_SEND_TAIL,
    };
    vision_instance->send_data = VisionSendRegister(&send_config);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return vision_instance->recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    static uint8_t send_buff[sizeof(Vision_Send_s)];
    SendProcess(vision_instance->send_data, send_buff);
    USARTSend(vision_instance->usart, send_buff, sizeof(Vision_Send_s), USART_TRANSFER_BLOCKING);
}
#endif

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
/**
 * @brief 回调函数，确认帧头后用于解析视觉数据
 *
 */
static void DecodeVision(uint16_t var)
{
    UNUSED(var); // 仅为了消除警告
    if (vis_recv_buff[0] == vision_instance->recv_data->header) {
        // 读取视觉数据
        /* 接收校验位 */
        memcpy(&vision_instance->recv_data->checksum, &vis_recv_buff[VISION_RECV_SIZE - 2], 2);
        if (vision_instance->recv_data->checksum == Get_CRC16_Check_Sum(vis_recv_buff, VISION_RECV_SIZE - 2, CRC_INIT)) {
            DaemonReload(vision_daemon_instance);
            RecvProcess(vision_instance->recv_data, vis_recv_buff);
        } else {
            memset(vision_instance->recv_data, 0, sizeof(Vision_Recv_s));
        }
    }
}
/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *video_usart_handle)
{
    UNUSED(video_usart_handle); // 仅为了消除警告
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));
    Vision_Recv_Init_Config_s recv_config = {
        .header = VISION_RECV_HEADER,
    };

    USB_Init_Config_s conf     = {.rx_cbk = DecodeVision};
    vis_recv_buff              = USBInit(conf);
    recv_config.header         = VISION_RECV_HEADER;
    vision_instance->recv_data = VisionRecvRegister(&recv_config);

    Vision_Send_Init_Config_s send_config = {
        .header       = VISION_SEND_HEADER,
        .detect_color = VISION_DETECT_COLOR_RED,
        .tail         = VISION_SEND_TAIL,
    };
    vision_instance->send_data = VisionSendRegister(&send_config);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
    return vision_instance->recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(vision_instance->send_data, send_buff);
    USBTransmit(send_buff, VISION_SEND_SIZE);
}

#endif