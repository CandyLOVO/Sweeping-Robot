//传感器检测任务即SensorTask运行
#include "robot_def.h"
#include "robot_sensor.h"
#include "bsp_gpio.h"
#include "infrared.h"
#include "cliff.h"
#include "message_center.h"
#include "string.h"

static IR_Instance *ir_l, *ir_r;                  // 左右红外传感器实例指针
static Cliff_Instance *cliff_instance = NULL;   // 悬崖传感器实例（静态变量）
static Sensor_Upload_Data_s sensor_upload_data; // 传感器上传数据结构体
static Publisher_t *sensor_pub = NULL;          // 传感器数据发布者句柄

// 红外传感器回调函数
static void Infrared_Callback(IR_Instance *ir)
{

}
void SensorInit(void)
{
    // 定义红外传感器配置
    IR_Config_s ir_cofig;

    ir_cofig.gpio_config.GPIOx = GPIOE;
    ir_cofig.gpio_config.GPIO_Pin = GPIO_PIN_5;
    ir_cofig.gpio_config.exti_mode = GPIO_EXTI_MODE_FALLING;
    ir_cofig.gpio_config.pin_state = GPIO_PIN_SET;
    ir_cofig.on_detect = Infrared_Callback;

    ir_l = IRRegister(&ir_cofig);
    ir_cofig.gpio_config.GPIOx = GPIOB;
    ir_cofig.gpio_config.GPIO_Pin = GPIO_PIN_8;
    ir_r = IRRegister(&ir_cofig);
    // 初始化悬崖传感器
    cliff_instance = Cliff_Init();
    sensor_pub = PubRegister("sensor_fetch", sizeof(Sensor_Upload_Data_s));
    // 启动ADC采样
    ADC_Start();
}

void SensorTask(void)
{
    Cliff_Update(); // 每次刷新电压

    // 更新悬崖数据
    memcpy(sensor_upload_data.cliff_detected, cliff_instance->detected, sizeof(cliff_instance->detected));
    PubPushMessage(sensor_pub, (void *)&sensor_upload_data); // 发布传感器数据
}