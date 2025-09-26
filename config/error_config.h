/**
 * @file    error_config.h
 * @brief   系统错误配置文件 - 统一错误处理和日志管理
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __ERROR_CONFIG_H__
#define __ERROR_CONFIG_H__

#include "gd32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------- 错误级别定义 ---------- */

typedef enum {
    ERROR_LEVEL_INFO = 0,        // 信息级别
    ERROR_LEVEL_WARNING,         // 警告级别
    ERROR_LEVEL_ERROR,           // 错误级别
    ERROR_LEVEL_CRITICAL,        // 关键错误级别
    ERROR_LEVEL_FATAL            // 致命错误级别
} error_level_t;

/* ---------- 错误类别定义 ---------- */

typedef enum {
    ERROR_CATEGORY_SYSTEM = 0x1000,      // 系统错误
    ERROR_CATEGORY_SENSOR = 0x2000,      // 传感器错误
    ERROR_CATEGORY_ACTUATOR = 0x3000,    // 执行器错误
    ERROR_CATEGORY_CONTROL = 0x4000,     // 控制错误
    ERROR_CATEGORY_COMMUNICATION = 0x5000, // 通信错误
    ERROR_CATEGORY_SAFETY = 0x6000,      // 安全错误
    ERROR_CATEGORY_HMI = 0x7000,         // HMI错误
    ERROR_CATEGORY_CONFIG = 0x8000       // 配置错误
} error_category_t;

/* ---------- 系统错误代码定义 ---------- */

// 系统级错误 (0x1000-0x1FFF)
#define ERROR_SYSTEM_INIT_FAILED           (ERROR_CATEGORY_SYSTEM + 0x01)
#define ERROR_SYSTEM_CLOCK_FAILED          (ERROR_CATEGORY_SYSTEM + 0x02)
#define ERROR_SYSTEM_MEMORY_FAULT          (ERROR_CATEGORY_SYSTEM + 0x03)
#define ERROR_SYSTEM_WATCHDOG_TIMEOUT      (ERROR_CATEGORY_SYSTEM + 0x04)
#define ERROR_SYSTEM_TASK_CREATE_FAILED    (ERROR_CATEGORY_SYSTEM + 0x05)
#define ERROR_SYSTEM_TASK_STACK_OVERFLOW   (ERROR_CATEGORY_SYSTEM + 0x06)
#define ERROR_SYSTEM_HEAP_ALLOCATION_FAILED (ERROR_CATEGORY_SYSTEM + 0x07)
#define ERROR_SYSTEM_CRITICAL_TASK_FAILED  (ERROR_CATEGORY_SYSTEM + 0x08)

// 传感器错误 (0x2000-0x2FFF)
#define ERROR_SENSOR_LIQUID_LEVEL_1_FAULT  (ERROR_CATEGORY_SENSOR + 0x01)
#define ERROR_SENSOR_LIQUID_LEVEL_2_FAULT  (ERROR_CATEGORY_SENSOR + 0x02)
#define ERROR_SENSOR_PRESSURE_1_FAULT      (ERROR_CATEGORY_SENSOR + 0x03)
#define ERROR_SENSOR_PRESSURE_2_FAULT      (ERROR_CATEGORY_SENSOR + 0x04)
#define ERROR_SENSOR_TEMPERATURE_1_FAULT   (ERROR_CATEGORY_SENSOR + 0x05)
#define ERROR_SENSOR_TEMPERATURE_2_FAULT   (ERROR_CATEGORY_SENSOR + 0x06)
#define ERROR_SENSOR_TEMPERATURE_3_FAULT   (ERROR_CATEGORY_SENSOR + 0x07)
#define ERROR_SENSOR_ADC_FAULT             (ERROR_CATEGORY_SENSOR + 0x10)
#define ERROR_SENSOR_CALIBRATION_FAULT     (ERROR_CATEGORY_SENSOR + 0x11)
#define ERROR_SENSOR_OUT_OF_RANGE          (ERROR_CATEGORY_SENSOR + 0x12)
#define ERROR_SENSOR_COMMUNICATION_TIMEOUT (ERROR_CATEGORY_SENSOR + 0x13)

// 执行器错误 (0x3000-0x3FFF)
#define ERROR_ACTUATOR_HEATER_1_FAULT      (ERROR_CATEGORY_ACTUATOR + 0x01)
#define ERROR_ACTUATOR_HEATER_2_FAULT      (ERROR_CATEGORY_ACTUATOR + 0x02)
#define ERROR_ACTUATOR_HEATER_3_FAULT      (ERROR_CATEGORY_ACTUATOR + 0x03)
#define ERROR_ACTUATOR_PUMP_1_FAULT        (ERROR_CATEGORY_ACTUATOR + 0x04)
#define ERROR_ACTUATOR_PUMP_2_FAULT        (ERROR_CATEGORY_ACTUATOR + 0x05)
#define ERROR_ACTUATOR_VALVE_FAULT         (ERROR_CATEGORY_ACTUATOR + 0x06)
#define ERROR_ACTUATOR_PWM_FAULT           (ERROR_CATEGORY_ACTUATOR + 0x10)
#define ERROR_ACTUATOR_OVERCURRENT         (ERROR_CATEGORY_ACTUATOR + 0x11)
#define ERROR_ACTUATOR_OVERTEMPERATURE     (ERROR_CATEGORY_ACTUATOR + 0x12)
#define ERROR_ACTUATOR_POWER_SUPPLY_FAULT  (ERROR_CATEGORY_ACTUATOR + 0x13)

// 控制错误 (0x4000-0x4FFF)
#define ERROR_CONTROL_PID_FAULT            (ERROR_CATEGORY_CONTROL + 0x01)
#define ERROR_CONTROL_ALGORITHM_FAULT      (ERROR_CATEGORY_CONTROL + 0x02)
#define ERROR_CONTROL_PARAMETER_FAULT      (ERROR_CATEGORY_CONTROL + 0x03)
#define ERROR_CONTROL_SETPOINT_FAULT       (ERROR_CATEGORY_CONTROL + 0x04)
#define ERROR_CONTROL_OUTPUT_LIMIT_FAULT   (ERROR_CATEGORY_CONTROL + 0x05)
#define ERROR_CONTROL_STABILITY_FAULT      (ERROR_CATEGORY_CONTROL + 0x06)

// 通信错误 (0x5000-0x5FFF)
#define ERROR_COMM_ETHERCAT_INIT_FAILED    (ERROR_CATEGORY_COMMUNICATION + 0x01)
#define ERROR_COMM_ETHERCAT_CONNECTION_LOST (ERROR_CATEGORY_COMMUNICATION + 0x02)
#define ERROR_COMM_ETHERCAT_SYNC_FAULT     (ERROR_CATEGORY_COMMUNICATION + 0x03)
#define ERROR_COMM_TCP_INIT_FAILED         (ERROR_CATEGORY_COMMUNICATION + 0x10)
#define ERROR_COMM_TCP_CONNECTION_LOST     (ERROR_CATEGORY_COMMUNICATION + 0x11)
#define ERROR_COMM_TCP_DATA_FAULT          (ERROR_CATEGORY_COMMUNICATION + 0x12)
#define ERROR_COMM_ETHERNET_PHY_FAULT      (ERROR_CATEGORY_COMMUNICATION + 0x20)
#define ERROR_COMM_PROTOCOL_FAULT          (ERROR_CATEGORY_COMMUNICATION + 0x21)

// 安全错误 (0x6000-0x6FFF)
#define ERROR_SAFETY_EMERGENCY_STOP        (ERROR_CATEGORY_SAFETY + 0x01)
#define ERROR_SAFETY_TEMPERATURE_LIMIT     (ERROR_CATEGORY_SAFETY + 0x02)
#define ERROR_SAFETY_PRESSURE_LIMIT        (ERROR_CATEGORY_SAFETY + 0x03)
#define ERROR_SAFETY_LEVEL_LIMIT           (ERROR_CATEGORY_SAFETY + 0x04)
#define ERROR_SAFETY_INTERLOCK_FAULT       (ERROR_CATEGORY_SAFETY + 0x05)
#define ERROR_SAFETY_REDUNDANCY_FAULT      (ERROR_CATEGORY_SAFETY + 0x06)
#define ERROR_SAFETY_MONITORING_FAULT      (ERROR_CATEGORY_SAFETY + 0x07)

// HMI错误 (0x7000-0x7FFF)
#define ERROR_HMI_DISPLAY_FAULT            (ERROR_CATEGORY_HMI + 0x01)
#define ERROR_HMI_KEYPAD_FAULT             (ERROR_CATEGORY_HMI + 0x02)
#define ERROR_HMI_LED_FAULT                (ERROR_CATEGORY_HMI + 0x03)
#define ERROR_HMI_BUZZER_FAULT             (ERROR_CATEGORY_HMI + 0x04)
#define ERROR_HMI_COMMUNICATION_FAULT      (ERROR_CATEGORY_HMI + 0x05)

// 配置错误 (0x8000-0x8FFF)
#define ERROR_CONFIG_FLASH_WRITE_FAILED    (ERROR_CATEGORY_CONFIG + 0x01)
#define ERROR_CONFIG_FLASH_READ_FAILED     (ERROR_CATEGORY_CONFIG + 0x02)
#define ERROR_CONFIG_CHECKSUM_FAULT        (ERROR_CATEGORY_CONFIG + 0x03)
#define ERROR_CONFIG_VERSION_MISMATCH      (ERROR_CATEGORY_CONFIG + 0x04)
#define ERROR_CONFIG_PARAMETER_FAULT       (ERROR_CATEGORY_CONFIG + 0x05)
#define ERROR_CONFIG_BACKUP_FAULT          (ERROR_CATEGORY_CONFIG + 0x06)

/* ---------- 错误记录结构定义 ---------- */

typedef struct {
    uint16_t error_code;         // 错误代码
    error_level_t level;         // 错误级别
    uint32_t timestamp;          // 时间戳 (系统tick)
    uint32_t count;              // 发生次数
    bool active;                 // 是否当前活跃
    char description[64];        // 错误描述
} error_record_t;

/* ---------- 错误处理配置 ---------- */

/**
 * MAX_ERROR_RECORDS: 最大错误记录数量
 */
#define MAX_ERROR_RECORDS           100

/**
 * ERROR_LOG_ENABLE: 错误日志使能
 */
#define ERROR_LOG_ENABLE            1

/**
 * ERROR_FLASH_BACKUP_ENABLE: 错误Flash备份使能
 */
#define ERROR_FLASH_BACKUP_ENABLE   1

/**
 * ERROR_AUTO_RECOVERY_ENABLE: 错误自动恢复使能
 */
#define ERROR_AUTO_RECOVERY_ENABLE  1

/**
 * ERROR_WATCHDOG_ENABLE: 错误看门狗使能
 */
#define ERROR_WATCHDOG_ENABLE       1

/**
 * CRITICAL_ERROR_SHUTDOWN_ENABLE: 关键错误系统关闭使能
 */
#define CRITICAL_ERROR_SHUTDOWN_ENABLE 1

/* ---------- 错误响应配置 ---------- */

/**
 * 错误响应动作类型
 */
typedef enum {
    ERROR_ACTION_NONE = 0,       // 无动作
    ERROR_ACTION_LOG_ONLY,       // 仅记录日志
    ERROR_ACTION_WARNING,        // 发出警告
    ERROR_ACTION_ALARM,          // 发出报警
    ERROR_ACTION_SHUTDOWN,       // 系统关闭
    ERROR_ACTION_EMERGENCY_STOP, // 紧急停止
    ERROR_ACTION_RESET           // 系统重启
} error_action_t;

/**
 * 错误响应配置结构
 */
typedef struct {
    uint16_t error_code;         // 错误代码
    error_action_t action;       // 响应动作
    uint32_t delay_ms;           // 动作延时 (毫秒)
    uint32_t retry_count;        // 重试次数
    bool auto_recovery;          // 自动恢复使能
} error_response_config_t;

/* ---------- 错误恢复配置 ---------- */

/**
 * ERROR_RECOVERY_TIMEOUT_MS: 错误恢复超时时间
 */
#define ERROR_RECOVERY_TIMEOUT_MS   30000   // 30秒

/**
 * ERROR_RETRY_MAX_COUNT: 最大重试次数
 */
#define ERROR_RETRY_MAX_COUNT       3

/**
 * ERROR_RECOVERY_INTERVAL_MS: 错误恢复检查间隔
 */
#define ERROR_RECOVERY_INTERVAL_MS  1000    // 1秒

/* ---------- 错误Flash存储配置 ---------- */

/**
 * ERROR_FLASH_BASE_ADDR: 错误Flash存储基地址
 */
#define ERROR_FLASH_BASE_ADDR       0x08070000

/**
 * ERROR_FLASH_SIZE: 错误Flash存储大小
 */
#define ERROR_FLASH_SIZE            0x10000     // 64KB

/**
 * ERROR_FLASH_RECORD_SIZE: 单个错误记录大小
 */
#define ERROR_FLASH_RECORD_SIZE     128

/**
 * ERROR_FLASH_MAX_RECORDS: Flash最大错误记录数
 */
#define ERROR_FLASH_MAX_RECORDS     (ERROR_FLASH_SIZE / ERROR_FLASH_RECORD_SIZE)

/* ---------- 调试配置 ---------- */

/**
 * ERROR_DEBUG_ENABLE: 错误调试使能
 */
#ifdef DEBUG
#define ERROR_DEBUG_ENABLE          1
#else
#define ERROR_DEBUG_ENABLE          0
#endif

/**
 * ERROR_DEBUG_UART: 错误调试串口
 */
#define ERROR_DEBUG_UART            USART0

/**
 * ERROR_TRACE_ENABLE: 错误跟踪使能
 */
#define ERROR_TRACE_ENABLE          ERROR_DEBUG_ENABLE

/* ---------- 函数声明 ---------- */

/**
 * 错误处理系统初始化
 */
void error_system_init(void);

/**
 * 记录错误
 */
void error_record(uint16_t error_code, error_level_t level, const char* description);

/**
 * 清除错误
 */
void error_clear(uint16_t error_code);

/**
 * 获取错误记录
 */
error_record_t* error_get_records(uint16_t* count);

/**
 * 错误恢复处理
 */
void error_recovery_process(void);

/**
 * 检查是否有活跃错误
 */
bool error_has_active_errors(void);

/**
 * 获取最高级别的活跃错误
 */
error_level_t error_get_highest_level(void);

/* ---------- 错误宏定义 ---------- */

/**
 * 错误记录宏
 */
#define ERROR_RECORD(code, level, desc) \
    error_record((code), (level), (desc))

/**
 * 信息记录宏
 */
#define ERROR_INFO(code, desc) \
    ERROR_RECORD((code), ERROR_LEVEL_INFO, (desc))

/**
 * 警告记录宏
 */
#define ERROR_WARNING(code, desc) \
    ERROR_RECORD((code), ERROR_LEVEL_WARNING, (desc))

/**
 * 错误记录宏
 */
#define ERROR_ERROR(code, desc) \
    ERROR_RECORD((code), ERROR_LEVEL_ERROR, (desc))

/**
 * 关键错误记录宏
 */
#define ERROR_CRITICAL(code, desc) \
    ERROR_RECORD((code), ERROR_LEVEL_CRITICAL, (desc))

/**
 * 致命错误记录宏
 */
#define ERROR_FATAL(code, desc) \
    ERROR_RECORD((code), ERROR_LEVEL_FATAL, (desc))

#endif /* __ERROR_CONFIG_H__ */