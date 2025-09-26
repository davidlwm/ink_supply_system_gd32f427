/**
 * @file    safety_task.h
 * @brief   安全任务头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __SAFETY_TASK_H
#define __SAFETY_TASK_H

#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

// 安全结果定义
typedef enum {
    SAFETY_SUCCESS = 0,
    SAFETY_ERROR_INVALID_PARAMETER,
    SAFETY_ERROR_SENSOR_FAULT,
    SAFETY_ERROR_ACTUATOR_FAULT,
    SAFETY_ERROR_TEMPERATURE_OVER_LIMIT,
    SAFETY_ERROR_PRESSURE_OVER_LIMIT,
    SAFETY_ERROR_EMERGENCY_STOP,
    SAFETY_ERROR_WATCHDOG_TIMEOUT
} safety_result_t;

// 安全等级定义
typedef enum {
    SAFETY_LEVEL_NORMAL = 0,
    SAFETY_LEVEL_WARNING,
    SAFETY_LEVEL_ALARM,
    SAFETY_LEVEL_EMERGENCY,
    SAFETY_LEVEL_CRITICAL
} safety_level_t;

// 安全检查项目定义
typedef enum {
    SAFETY_CHECK_TEMPERATURE = 0,
    SAFETY_CHECK_PRESSURE,
    SAFETY_CHECK_LIQUID_LEVEL,
    SAFETY_CHECK_EMERGENCY_STOP,
    SAFETY_CHECK_SENSOR_HEALTH,
    SAFETY_CHECK_ACTUATOR_HEALTH,
    SAFETY_CHECK_COMMUNICATION,
    SAFETY_CHECK_POWER_SUPPLY
} safety_check_item_t;

// 安全限制参数
typedef struct {
    float max_temperature[3];      // 最大温度限制
    float min_temperature[3];      // 最小温度限制
    float max_pressure[2];         // 最大压力限制
    float min_pressure[2];         // 最小压力限制
    float max_liquid_level[2];     // 最大液位限制
    float min_liquid_level[2];     // 最小液位限制
    uint32_t sensor_timeout_ms;    // 传感器超时时间
    uint32_t actuator_timeout_ms;  // 执行器超时时间
} safety_limits_t;

// 安全状态结构
typedef struct {
    bool emergency_stop;           // 急停状态
    bool system_fault;            // 系统故障
    safety_level_t safety_level;  // 安全等级
    uint16_t fault_code;          // 故障代码
    uint32_t fault_count;         // 故障计数
    uint32_t last_check_time;     // 上次检查时间
} safety_status_t;

// 故障记录结构
typedef struct {
    uint16_t fault_code;
    safety_level_t level;
    uint32_t timestamp;
    bool active;
    uint32_t occurrence_count;
} safety_fault_record_t;

// 任务函数
void safety_task(void *pvParameters);
safety_result_t safety_manager_init(void);

// 安全检查函数
safety_result_t safety_check_all_systems(void);
safety_result_t safety_check_temperatures(void);
safety_result_t safety_check_pressures(void);
safety_result_t safety_check_liquid_levels(void);
safety_result_t safety_check_emergency_stop(void);
safety_result_t safety_check_sensor_health(void);
safety_result_t safety_check_actuator_health(void);

// 故障处理函数
safety_result_t safety_handle_fault(uint16_t fault_code, safety_level_t level);
safety_result_t safety_clear_fault(uint16_t fault_code);
safety_result_t safety_emergency_shutdown(void);
safety_result_t safety_system_recovery(void);

// 状态查询函数
safety_result_t safety_get_system_status(safety_status_t *status);
safety_result_t safety_get_fault_records(safety_fault_record_t *records, uint8_t max_count);
bool safety_is_system_safe(void);
bool safety_is_emergency_stop_active(void);

/* 控制任务兼容接口 */
typedef safety_status_t system_status_t;

// 安全限制设置
safety_result_t safety_set_limits(const safety_limits_t *limits);
safety_result_t safety_get_limits(safety_limits_t *limits);

// 内部处理函数
static void safety_task_init(void);
static void safety_monitor_cycle(void);
static void safety_update_fault_records(uint16_t fault_code, safety_level_t level);
static void safety_trigger_emergency_actions(void);
static bool safety_validate_sensor_data(void);
static bool safety_validate_actuator_status(void);

// 安全故障代码定义
#define SAFETY_FAULT_TEMPERATURE_HIGH_1     0x3001
#define SAFETY_FAULT_TEMPERATURE_HIGH_2     0x3002
#define SAFETY_FAULT_TEMPERATURE_HIGH_3     0x3003
#define SAFETY_FAULT_TEMPERATURE_LOW_1      0x3004
#define SAFETY_FAULT_TEMPERATURE_LOW_2      0x3005
#define SAFETY_FAULT_TEMPERATURE_LOW_3      0x3006
#define SAFETY_FAULT_PRESSURE_HIGH_1        0x3011
#define SAFETY_FAULT_PRESSURE_HIGH_2        0x3012
#define SAFETY_FAULT_PRESSURE_LOW_1         0x3013
#define SAFETY_FAULT_PRESSURE_LOW_2         0x3014
#define SAFETY_FAULT_LEVEL_HIGH_1           0x3021
#define SAFETY_FAULT_LEVEL_HIGH_2           0x3022
#define SAFETY_FAULT_LEVEL_LOW_1            0x3023
#define SAFETY_FAULT_LEVEL_LOW_2            0x3024
#define SAFETY_FAULT_EMERGENCY_STOP         0x3031
#define SAFETY_FAULT_SENSOR_TIMEOUT         0x3041
#define SAFETY_FAULT_ACTUATOR_TIMEOUT       0x3042
#define SAFETY_FAULT_COMMUNICATION_LOST     0x3051
#define SAFETY_FAULT_POWER_SUPPLY           0x3061

// 外部接口函数声明
extern safety_result_t sensor_get_all_data(void *data);
extern safety_result_t actuator_get_status(void *status);
extern bool emergency_stop_button_pressed(void);

#endif /* __SAFETY_TASK_H */