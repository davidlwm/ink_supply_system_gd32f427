/**
 * @file    hmi_task.h
 * @brief   人机界面任务头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __HMI_TASK_H
#define __HMI_TASK_H

#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

// HMI结果定义
typedef enum {
    HMI_SUCCESS = 0,
    HMI_ERROR_INVALID_PARAMETER,
    HMI_ERROR_DISPLAY_FAULT,
    HMI_ERROR_KEYPAD_FAULT,
    HMI_ERROR_TIMEOUT,
    HMI_ERROR_MEMORY_ALLOCATION
} hmi_result_t;

// 显示页面定义
typedef enum {
    HMI_PAGE_MAIN = 0,
    HMI_PAGE_SENSOR_STATUS,
    HMI_PAGE_ACTUATOR_STATUS,
    HMI_PAGE_CONTROL_PARAMS,
    HMI_PAGE_SYSTEM_INFO,
    HMI_PAGE_FAULT_LOG,
    HMI_PAGE_SETTINGS,
    HMI_PAGE_CALIBRATION
} hmi_page_t;

// 按键定义
typedef enum {
    KEY_UP = 0,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_ENTER,
    KEY_ESC,
    KEY_MENU,
    KEY_F1,
    KEY_F2,
    KEY_F3
} key_code_t;

// LED指示灯定义
typedef enum {
    LED_POWER = 0,
    LED_RUN,
    LED_ERROR,
    LED_COMM,
    LED_ALARM
} led_indicator_t;

// 蜂鸣器模式定义
typedef enum {
    BUZZER_OFF = 0,
    BUZZER_SINGLE_BEEP,
    BUZZER_DOUBLE_BEEP,
    BUZZER_LONG_BEEP,
    BUZZER_ALARM
} buzzer_mode_t;

// HMI状态结构
typedef struct {
    hmi_page_t current_page;
    bool display_enabled;
    bool keypad_enabled;
    uint8_t brightness;
    uint32_t last_activity_time;
    bool screensaver_active;
} hmi_status_t;

// 系统显示信息结构
typedef struct {
    float temperatures[3];
    float pressures[2];
    float liquid_levels[2];
    bool fault_status[16];
    uint8_t system_state;
    float system_efficiency;
    uint32_t uptime;
} system_display_info_t;

// 任务函数
void hmi_task(void *pvParameters);
hmi_result_t hmi_manager_init(void);

// 显示控制函数
hmi_result_t hmi_display_page(hmi_page_t page);
hmi_result_t hmi_update_display(void);
hmi_result_t hmi_set_brightness(uint8_t brightness);
hmi_result_t hmi_enable_screensaver(bool enable);

// 按键处理函数
hmi_result_t hmi_process_key(key_code_t key);
bool hmi_is_key_pressed(key_code_t key);

// LED和蜂鸣器控制
hmi_result_t hmi_set_led(led_indicator_t led, bool on);
hmi_result_t hmi_set_led_blink(led_indicator_t led, uint32_t period_ms);
hmi_result_t hmi_set_buzzer(buzzer_mode_t mode);

// 信息更新函数
hmi_result_t hmi_update_system_info(const system_display_info_t *info);
hmi_result_t hmi_show_message(const char *message, uint32_t duration_ms);
hmi_result_t hmi_show_error(uint16_t error_code, const char *description);

// 内部处理函数
static void hmi_task_init(void);
static void hmi_display_main_page(void);
static void hmi_display_sensor_page(void);
static void hmi_display_actuator_page(void);
static void hmi_display_system_page(void);
static void hmi_handle_navigation(key_code_t key);
static void hmi_update_indicators(void);
static void hmi_check_screensaver(void);

// 硬件驱动接口
extern void lcd_init(void);
extern void lcd_clear(void);
extern void lcd_print(uint8_t row, uint8_t col, const char *text);
extern void lcd_set_brightness(uint8_t brightness);
extern void keypad_init(void);
extern bool keypad_get_key(key_code_t *key);
extern void led_driver_init(void);
extern void led_driver_set(uint8_t led_id, bool on);
extern void buzzer_init(void);
extern void buzzer_beep(buzzer_mode_t mode);

#endif /* __HMI_TASK_H */