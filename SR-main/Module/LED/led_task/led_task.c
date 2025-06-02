#include "led_task.h"
#include "LED.h"
#include "message_center.h"

#include "robot_def.h"

static LED_Instance *led_r, *led_l;
static Chassis_Ctrl_Cmd_s cmd_recv;     // The command to receive from the robot_cmd
static Robot_Status_e robot_status_led; // The status of the robot
static Subscriber_t *led_subscriber_chassis = NULL;
static Subscriber_t *led_subscriber_robot   = NULL;

void LEDInit(void)
{
    // Initialize the LED instances here if needed
    // For example, you can register the LEDs and set their initial states
    LED_Config_s led_config = {
        .gpio_config = {
            .GPIOx     = LED0_GPIO_Port,
            .GPIO_Pin  = LED0_Pin,
            .pin_state = GPIO_PIN_SET,
            .exti_mode = GPIO_EXTI_MODE_NONE,
        },
        .initial_state = LED_OFF,
        .on_change     = NULL, // No callback for now
    };
    led_r = LEDRegister(&led_config);

    led_config.gpio_config.GPIOx    = LED1_GPIO_Port;
    led_config.gpio_config.GPIO_Pin = LED1_Pin;

    led_l = LEDRegister(&led_config);
    // Register the LED task with the message center
    led_subscriber_chassis = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    led_subscriber_robot   = SubRegister("robot_state", sizeof(Robot_Status_e));
}

void LEDTask(void)
{
    SubGetMessage(led_subscriber_chassis, &cmd_recv);
    SubGetMessage(led_subscriber_robot, &robot_status_led);

    switch (cmd_recv.chassis_mode) {
        case CHASSIS_NORMAL:
        case CHASSIS_GOTO_POINT:
            LEDOn(led_l);
            break;
        case CHASSIS_ZERO_FORCE:
            LEDOff(led_l);
            break;
        default:
            break;
    }

    switch (robot_status_led) {
        case ROBOT_READY:
            LEDOn(led_r);
            break;
        default:
            LEDOff(led_r);
            break;
    }
}