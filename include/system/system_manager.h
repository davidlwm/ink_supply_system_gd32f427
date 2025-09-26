/**
 * @file    system_manager.h
 * @brief   系统管理器头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __SYSTEM_MANAGER_H
#define __SYSTEM_MANAGER_H

#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

// 系统初始化结果定义
typedef enum {
    SYSTEM_INIT_SUCCESS = 0,
    SYSTEM_INIT_ERROR_MEMORY,
    SYSTEM_INIT_ERROR_PERIPHERAL,
    SYSTEM_INIT_ERROR_TASK_CREATE,
    SYSTEM_INIT_ERROR_INVALID_STATE
} system_init_result_t;

// 系统状态定义
typedef enum {
    SYSTEM_STATE_INIT = 0,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_RUNNING,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_EMERGENCY_STOP
} system_state_t;

// 系统运行模式定义
typedef enum {
    SYSTEM_MODE_MANUAL = 0,
    SYSTEM_MODE_AUTOMATIC,
    SYSTEM_MODE_CALIBRATION,
    SYSTEM_MODE_TEST
} system_mode_t;

// 系统管理器函数
system_init_result_t system_manager_init(void);
void system_early_init(void);

// 模块初始化函数
void sensor_manager_init(void);
void actuator_manager_init(void);
void comm_manager_init(void);
void hmi_manager_init(void);
void safety_manager_init(void);
void config_manager_init(void);

// 系统状态管理
void system_set_state(system_state_t new_state);
system_state_t system_get_state(void);
void system_set_mode(system_mode_t new_mode);
system_mode_t system_get_mode(void);

// 系统监控和诊断
void system_health_monitor(void);
uint32_t system_get_uptime(void);
float system_get_cpu_usage(void);

#endif /* __SYSTEM_MANAGER_H */