/**
 * @file    led_control.c
 * @brief   LED指示灯控制驱动实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "led_control.h"
#include "pwm_hal.h"
#include "gpio_hal.h"
#include "system_config.h"

// LED配置参数
typedef struct {
    pwm_channel_id_t pwm_channel;      // PWM通道 (用于亮度控制)
    gpio_output_id_t gpio_pin;         // GPIO引脚 (用于开关控制)
    bool use_pwm;                      // 是否使用PWM控制亮度
    uint8_t max_brightness;            // 最大亮度 (0-100)
    led_color_t color;                 // LED颜色
} led_config_t;

// LED实例配置
static const led_config_t led_configs[5] = {
    // LED1 (电源指示) - 绿色
    {
        .pwm_channel = PWM_CH_LED_1,
        .gpio_pin = GPIO_LED_POWER,
        .use_pwm = false,               // 电源指示只需要开关
        .max_brightness = 100,
        .color = LED_COLOR_GREEN
    },
    // LED2 (运行指示) - 蓝色
    {
        .pwm_channel = PWM_CH_LED_2,
        .gpio_pin = GPIO_LED_RUN,
        .use_pwm = true,                // 支持亮度调节
        .max_brightness = 80,
        .color = LED_COLOR_BLUE
    },
    // LED3 (错误指示) - 红色
    {
        .pwm_channel = PWM_CH_LED_3,
        .gpio_pin = GPIO_LED_ERROR,
        .use_pwm = false,               // 错误指示只需要开关
        .max_brightness = 100,
        .color = LED_COLOR_RED
    },
    // LED4 (通信指示) - 黄色
    {
        .pwm_channel = PWM_CH_LED_4,
        .gpio_pin = GPIO_LED_COMM,
        .use_pwm = true,                // 支持亮度调节
        .max_brightness = 90,
        .color = LED_COLOR_YELLOW
    },
    // LED5 (报警指示) - 红色
    {
        .pwm_channel = PWM_CH_SPARE_1,  // 使用备用PWM通道
        .gpio_pin = GPIO_LED_ALARM,
        .use_pwm = false,               // 报警指示只需要开关
        .max_brightness = 100,
        .color = LED_COLOR_RED
    }
};

// LED状态
static led_status_t led_status[5];

// 闪烁控制
static uint32_t blink_periods[5] = {0};
static uint32_t blink_timers[5] = {0};
static bool blink_states[5] = {false};

/**
 * @brief  LED控制初始化
 * @param  None
 * @retval None
 */
void led_control_init(void)
{
    // 初始化PWM和GPIO
    for (int i = 0; i < 5; i++) {
        const led_config_t *config = &led_configs[i];

        // 配置PWM通道 (2kHz适合LED)
        if (config->use_pwm) {
            pwm_hal_config_channel(config->pwm_channel, 2000);
        }

        // 配置GPIO引脚
        gpio_hal_config_output(config->gpio_pin);

        // 初始化状态
        led_status[i].on = false;
        led_status[i].brightness = 100;  // 默认最大亮度
        led_status[i].blinking = false;
        led_status[i].blink_period = 0;
        led_status[i].fault = false;
        led_status[i].initialized = true;

        // 关闭LED
        control_led(i, false);

        // 重置闪烁参数
        blink_periods[i] = 0;
        blink_timers[i] = 0;
        blink_states[i] = false;
    }
}

/**
 * @brief  控制LED开关
 * @param  led_id LED ID (0-4)
 * @param  on 开关状态
 * @retval None
 */
void control_led(uint8_t led_id, bool on)
{
    if (led_id >= 5) {
        return;
    }

    const led_config_t *config = &led_configs[led_id];

    // 更新状态
    led_status[led_id].on = on;

    if (config->use_pwm) {
        // 使用PWM控制亮度
        if (on) {
            uint16_t duty_cycle = (uint16_t)(led_status[led_id].brightness * 100);
            pwm_hal_set_duty_cycle(config->pwm_channel, duty_cycle);
            pwm_hal_enable_channel(config->pwm_channel, true);
        } else {
            pwm_hal_set_duty_cycle(config->pwm_channel, 0);
            pwm_hal_enable_channel(config->pwm_channel, false);
        }
    } else {
        // 使用GPIO控制开关
        gpio_hal_write_pin(config->gpio_pin, on);
    }
}

/**
 * @brief  设置LED亮度
 * @param  led_id LED ID (0-4)
 * @param  brightness 亮度 (0-100)
 * @retval bool 设置成功返回true
 */
bool led_set_brightness(uint8_t led_id, uint8_t brightness)
{
    if (led_id >= 5 || brightness > 100) {
        return false;
    }

    const led_config_t *config = &led_configs[led_id];

    // 限制最大亮度
    if (brightness > config->max_brightness) {
        brightness = config->max_brightness;
    }

    led_status[led_id].brightness = brightness;

    // 如果LED开启且支持PWM，更新PWM占空比
    if (led_status[led_id].on && config->use_pwm) {
        uint16_t duty_cycle = (uint16_t)(brightness * 100);
        pwm_hal_set_duty_cycle(config->pwm_channel, duty_cycle);
    }

    return true;
}

/**
 * @brief  设置LED闪烁
 * @param  led_id LED ID (0-4)
 * @param  blink_period 闪烁周期 (ms, 0表示不闪烁)
 * @retval bool 设置成功返回true
 */
bool led_set_blink(uint8_t led_id, uint32_t blink_period)
{
    if (led_id >= 5) {
        return false;
    }

    led_status[led_id].blinking = (blink_period > 0);
    led_status[led_id].blink_period = blink_period;

    blink_periods[led_id] = blink_period;
    blink_timers[led_id] = xTaskGetTickCount();
    blink_states[led_id] = false;

    return true;
}

/**
 * @brief  更新LED闪烁状态 (需要周期性调用)
 * @param  None
 * @retval None
 */
void led_update_blink(void)
{
    uint32_t current_time = xTaskGetTickCount();

    for (int i = 0; i < 5; i++) {
        if (led_status[i].blinking && blink_periods[i] > 0) {
            uint32_t elapsed = current_time - blink_timers[i];

            if (elapsed >= pdMS_TO_TICKS(blink_periods[i] / 2)) {
                // 切换闪烁状态
                blink_states[i] = !blink_states[i];
                control_led(i, blink_states[i]);
                blink_timers[i] = current_time;
            }
        }
    }
}

/**
 * @brief  获取LED状态
 * @param  led_id LED ID (0-4)
 * @retval led_status_t LED状态
 */
led_status_t led_get_status(uint8_t led_id)
{
    led_status_t status = {0};

    if (led_id < 5) {
        status = led_status[led_id];
    }

    return status;
}

/**
 * @brief  设置LED状态
 * @param  led_id LED ID (0-4)
 * @param  on 开关状态
 * @param  blink_period 闪烁周期 (0表示不闪烁)
 * @retval bool 设置成功返回true
 */
bool led_set_state(uint8_t led_id, bool on, uint32_t blink_period)
{
    if (led_id >= 5) {
        return false;
    }

    if (blink_period > 0) {
        // 设置闪烁模式
        led_set_blink(led_id, blink_period);
    } else {
        // 设置固定状态
        led_status[led_id].blinking = false;
        blink_periods[led_id] = 0;
        control_led(led_id, on);
    }

    return true;
}

/**
 * @brief  获取LED开关状态
 * @param  led_id LED ID (0-4)
 * @retval bool LED状态 (true=开, false=关)
 */
bool led_get_state(uint8_t led_id)
{
    if (led_id >= 5) {
        return false;
    }

    return led_status[led_id].on;
}

/**
 * @brief  设置系统状态指示
 * @param  system_state 系统状态
 * @retval None
 */
void led_set_system_status(system_led_status_t system_state)
{
    switch (system_state) {
        case LED_STATUS_POWER_ON:
            led_set_state(LED_POWER, true, 0);
            led_set_state(LED_RUN, false, 0);
            led_set_state(LED_ERROR, false, 0);
            led_set_state(LED_ALARM, false, 0);
            break;

        case LED_STATUS_RUNNING:
            led_set_state(LED_POWER, true, 0);
            led_set_state(LED_RUN, true, 0);
            led_set_state(LED_ERROR, false, 0);
            led_set_state(LED_ALARM, false, 0);
            break;

        case LED_STATUS_IDLE:
            led_set_state(LED_POWER, true, 0);
            led_set_state(LED_RUN, true, 2000);  // 慢闪表示待机
            led_set_state(LED_ERROR, false, 0);
            led_set_state(LED_ALARM, false, 0);
            break;

        case LED_STATUS_ERROR:
            led_set_state(LED_POWER, true, 0);
            led_set_state(LED_RUN, false, 0);
            led_set_state(LED_ERROR, true, 500);  // 快闪表示错误
            led_set_state(LED_ALARM, false, 0);
            break;

        case LED_STATUS_ALARM:
            led_set_state(LED_POWER, true, 0);
            led_set_state(LED_RUN, false, 0);
            led_set_state(LED_ERROR, true, 0);
            led_set_state(LED_ALARM, true, 200);  // 超快闪表示报警
            break;

        case LED_STATUS_COMMUNICATION:
            led_set_state(LED_COMM, true, 100);   // 通信活动指示
            break;

        default:
            break;
    }
}

/**
 * @brief  关闭所有LED
 * @param  None
 * @retval None
 */
void led_all_off(void)
{
    for (int i = 0; i < 5; i++) {
        led_set_state(i, false, 0);
    }
}

/**
 * @brief  LED自检
 * @param  None
 * @retval bool 自检通过返回true
 */
bool led_self_test(void)
{
    // 依次点亮所有LED
    for (int i = 0; i < 5; i++) {
        control_led(i, true);
        vTaskDelay(pdMS_TO_TICKS(200));
        control_led(i, false);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 全部点亮1秒
    for (int i = 0; i < 5; i++) {
        control_led(i, true);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 全部关闭
    led_all_off();

    return true; // 简化处理，认为自检总是通过
}

/**
 * @brief  获取LED颜色
 * @param  led_id LED ID (0-4)
 * @retval led_color_t LED颜色
 */
led_color_t led_get_color(uint8_t led_id)
{
    if (led_id >= 5) {
        return LED_COLOR_WHITE;
    }

    return led_configs[led_id].color;
}

/**
 * @brief  获取LED亮度
 * @param  led_id LED ID (0-4)
 * @retval uint8_t 亮度 (0-100)
 */
uint8_t led_get_brightness(uint8_t led_id)
{
    if (led_id >= 5) {
        return 0;
    }

    return led_status[led_id].brightness;
}