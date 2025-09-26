/**
 * @file    error_handler.h
 * @brief   错误处理器头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

// 错误级别定义
typedef enum {
    ERROR_LEVEL_INFO = 0,
    ERROR_LEVEL_WARNING,
    ERROR_LEVEL_ERROR,
    ERROR_LEVEL_CRITICAL,
    ERROR_LEVEL_FATAL
} error_level_t;

// 系统错误代码定义
#define ERROR_BOARD_INIT_FAILED         0x0001
#define ERROR_SYSTEM_INIT_FAILED        0x0002
#define ERROR_SCHEDULER_FAILED          0x0003
#define ERROR_TASK_CREATE_FAILED        0x0004
#define ERROR_MEMORY_ALLOCATION         0x0005
#define ERROR_PERIPHERAL_FAULT          0x0006
#define ERROR_COMMUNICATION_LOST        0x0007
#define ERROR_SENSOR_FAULT              0x0008
#define ERROR_ACTUATOR_FAULT            0x0009
#define ERROR_SAFETY_VIOLATION          0x000A
#define ERROR_WATCHDOG_TIMEOUT          0x000B
#define ERROR_STACK_OVERFLOW            0x000C

// 错误信息结构
typedef struct {
    uint16_t error_code;
    error_level_t level;
    uint32_t timestamp;
    uint32_t line_number;
    const char *file_name;
    const char *description;
} error_info_t;

// 错误处理函数
void error_handler(uint16_t error_code);
void error_handler_with_info(uint16_t error_code, const char *file, uint32_t line);
void error_log(error_level_t level, uint16_t error_code, const char *description);

// 错误日志管理
void error_log_init(void);
bool error_log_add(const error_info_t *error);
bool error_log_get_last(error_info_t *error);
uint32_t error_log_get_count(void);
void error_log_clear(void);

// 系统复位和恢复
void system_soft_reset(void);
void system_emergency_stop(void);
bool system_recovery_attempt(uint16_t error_code);

// 错误处理宏
#define ERROR_HANDLER() error_handler_with_info(0, __FILE__, __LINE__)
#define ERROR_HANDLER_CODE(code) error_handler_with_info(code, __FILE__, __LINE__)

#endif /* __ERROR_HANDLER_H */