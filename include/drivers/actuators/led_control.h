/**
 * @file    led_control.h
 * @brief   LED指示灯控制驱动头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// LED ID定义
typedef enum {
    LED_POWER = 0,      // 电源指示LED
    LED_RUN = 1,        // 运行指示LED
    LED_ERROR = 2,      // 错误指示LED
    LED_COMM = 3,       // 通信指示LED
    LED_ALARM = 4       // 报警指示LED
} led_id_t;

// LED颜色定义
typedef enum {
    LED_COLOR_RED = 0,
    LED_COLOR_GREEN = 1,
    LED_COLOR_BLUE = 2,
    LED_COLOR_YELLOW = 3,
    LED_COLOR_WHITE = 4
} led_color_t;

// 系统LED状态定义
typedef enum {
    LED_STATUS_POWER_ON = 0,
    LED_STATUS_RUNNING = 1,
    LED_STATUS_IDLE = 2,
    LED_STATUS_ERROR = 3,
    LED_STATUS_ALARM = 4,
    LED_STATUS_COMMUNICATION = 5
} system_led_status_t;

// LED状态结构
typedef struct {
    bool on;                // 开关状态
    uint8_t brightness;     // 亮度 (0-100)
    bool blinking;          // 闪烁状态
    uint32_t blink_period;  // 闪烁周期 (ms)
    bool fault;            // 故障状态
    bool initialized;      // 初始化状态
} led_status_t;

// 函数声明
void led_control_init(void);
void control_led(uint8_t led_id, bool on);
bool led_set_brightness(uint8_t led_id, uint8_t brightness);
bool led_set_blink(uint8_t led_id, uint32_t blink_period);
void led_update_blink(void);
led_status_t led_get_status(uint8_t led_id);
bool led_set_state(uint8_t led_id, bool on, uint32_t blink_period);
bool led_get_state(uint8_t led_id);
void led_set_system_status(system_led_status_t system_state);
void led_all_off(void);
bool led_self_test(void);
led_color_t led_get_color(uint8_t led_id);
uint8_t led_get_brightness(uint8_t led_id);

#endif /* __LED_CONTROL_H */