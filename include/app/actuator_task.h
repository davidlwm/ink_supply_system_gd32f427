/**
 * @file    actuator_task.h
 * @brief   执行器任务头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __ACTUATOR_TASK_H
#define __ACTUATOR_TASK_H

#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

// 执行器类型定义
typedef enum {
    ACTUATOR_TYPE_HEATER = 0,
    ACTUATOR_TYPE_PUMP = 1,
    ACTUATOR_TYPE_VALVE = 2,
    ACTUATOR_TYPE_LED = 3
} actuator_type_t;

// 故障代码定义 (执行器部分)
#define FAULT_HEATER_1_SAFETY           0x2001
#define FAULT_HEATER_2_SAFETY           0x2002
#define FAULT_HEATER_3_SAFETY           0x2003
#define FAULT_PUMP_1_SAFETY             0x2011
#define FAULT_PUMP_2_SAFETY             0x2012
#define FAULT_VALVE_1_SAFETY            0x2021
#define FAULT_VALVE_2_SAFETY            0x2022
#define FAULT_VALVE_3_SAFETY            0x2023
#define FAULT_VALVE_4_SAFETY            0x2024
#define FAULT_VALVE_5_SAFETY            0x2025
#define FAULT_VALVE_6_SAFETY            0x2026
#define FAULT_VALVE_7_SAFETY            0x2027
#define FAULT_VALVE_8_SAFETY            0x2028

// LED状态定义
#define LED_STATE_OFF                   0
#define LED_STATE_ON                    1
#define LED_BLINK_SLOW                  1000        // 1秒周期
#define LED_BLINK_FAST                  500         // 0.5秒周期
#define LED_BLINK_ULTRA                 200         // 0.2秒周期

// 任务函数声明
void actuator_task(void *pvParameters);
void actuator_manager_init(void);

// 公共接口函数
bool set_heater_power(uint8_t heater_id, float power_percent);
bool set_pump_speed(uint8_t pump_id, uint16_t speed_rpm);
bool set_valve_state(uint8_t valve_id, bool open);
bool set_led_state(uint8_t led_id, bool on, uint32_t blink_period_ms);

/* 控制任务兼容接口 */
bool actuator_set_heater_power(uint8_t heater_id, float power_percent);
bool actuator_set_pump_speed(uint8_t pump_id, uint16_t speed_rpm);
bool actuator_set_valve_state(uint8_t valve_id, bool state);

// 状态查询函数
float get_heater_power(uint8_t heater_id);
uint16_t get_pump_speed(uint8_t pump_id);
bool get_valve_state(uint8_t valve_id);
bool get_actuator_fault_status(uint8_t actuator_type, uint8_t actuator_id);

// 内部函数声明
static void actuator_task_init(void);
static void actuator_self_test(void);
static void update_led_outputs(void);
static void report_actuator_performance(uint32_t task_counter);

// 执行器驱动函数 (需要在对应的驱动文件中实现)
void control_heater_mra23d3(uint8_t heater_id, float power_percent);
void control_pump_speed_mpb025bbb(uint8_t pump_id, uint16_t target_rpm);
void control_valve(uint8_t valve_id, bool open);
void control_led(uint8_t led_id, bool on);

// 执行器初始化函数
void mra23d3_heater_init(void);
void mpb025bbb_pump_init(void);
void valve_control_init(void);
void led_control_init(void);

// 保护系统函数
void actuator_protection_init(void);
bool actuator_safety_check(uint8_t actuator_type, uint8_t actuator_id);

#endif /* __ACTUATOR_TASK_H */