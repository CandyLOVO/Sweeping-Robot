#include "robot_def.h"

#include "sensor.h"
#include "bsp_gpio.h"
#include "infrared.h"
#include "cliff.h"
#include "message_center.h"
#include "string.h"

static IR_Instance *ir_l, *ir_r;
static Cliff_Instance *cliff_instance = NULL;   // Static instance of Cliff_Instance
static Sensor_Upload_Data_s sensor_upload_data; // The data to upload from the sensors
static Publisher_t *sensor_pub = NULL;          // Publisher for sensor data

static void Infrared_Callback(IR_Instance *ir)
{
}
void SensorInit(void)
{
    IR_Config_s ir_cofig = {
        .gpio_config = {
            .GPIOx     = GPIOE,
            .GPIO_Pin  = GPIO_PIN_5,
            .exti_mode = GPIO_EXTI_MODE_FALLING,
            .pin_state = GPIO_PIN_SET,
        },
        .on_detect = Infrared_Callback, // 注册回调函数
    };
    ir_l = IRRegister(&ir_cofig);

    ir_cofig.gpio_config.GPIOx    = GPIOB;
    ir_cofig.gpio_config.GPIO_Pin = GPIO_PIN_8;
    ir_r                          = IRRegister(&ir_cofig);

    cliff_instance = Cliff_Init();

    sensor_pub = PubRegister("sensor_fetch", sizeof(Sensor_Upload_Data_s)); // 注册悬崖数据发布器

    ADC_Start(); // 启动ADC
}

void SensorTask(void)
{
    Cliff_Update(); // 每次刷新电压

    // 更新悬崖数据
    memcpy(sensor_upload_data.cliff_detected, cliff_instance->detected, sizeof(cliff_instance->detected));
    PubPushMessage(sensor_pub, (void *)&sensor_upload_data); // 发布传感器数据
}