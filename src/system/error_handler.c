/**
 * @file    error_handler.c
 * @brief   错误处理器实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "error_handler.h"
#include "system_manager.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// 标准库
#include <string.h>
#include <stdio.h>

// 错误日志配置
#define MAX_ERROR_LOG_ENTRIES   32
#define ERROR_LOG_ENTRY_SIZE    sizeof(error_info_t)

// 错误日志存储
static error_info_t g_error_log[MAX_ERROR_LOG_ENTRIES];
static uint32_t g_log_write_index = 0;
static uint32_t g_log_count = 0;
static SemaphoreHandle_t g_error_log_mutex = NULL;

// 系统错误统计
static uint32_t g_total_error_count = 0;
static uint32_t g_critical_error_count = 0;
static uint32_t g_last_error_time = 0;

// 错误描述表
static const char* error_descriptions[] = {
    [ERROR_BOARD_INIT_FAILED] = "Board initialization failed",
    [ERROR_SYSTEM_INIT_FAILED] = "System initialization failed",
    [ERROR_SCHEDULER_FAILED] = "Task scheduler failed",
    [ERROR_TASK_CREATE_FAILED] = "Task creation failed",
    [ERROR_MEMORY_ALLOCATION] = "Memory allocation failed",
    [ERROR_PERIPHERAL_FAULT] = "Peripheral fault detected",
    [ERROR_COMMUNICATION_LOST] = "Communication lost",
    [ERROR_SENSOR_FAULT] = "Sensor fault detected",
    [ERROR_ACTUATOR_FAULT] = "Actuator fault detected",
    [ERROR_SAFETY_VIOLATION] = "Safety violation occurred",
    [ERROR_WATCHDOG_TIMEOUT] = "Watchdog timeout",
    [ERROR_STACK_OVERFLOW] = "Stack overflow detected"
};

/**
 * @brief  错误日志初始化
 * @param  None
 * @retval None
 */
void error_log_init(void)
{
    // 创建错误日志互斥锁
    g_error_log_mutex = xSemaphoreCreateMutex();
    if (g_error_log_mutex == NULL) {
        // 无法创建互斥锁，系统将无法正常记录错误
        return;
    }

    // 清空错误日志
    memset(g_error_log, 0, sizeof(g_error_log));
    g_log_write_index = 0;
    g_log_count = 0;
    g_total_error_count = 0;
    g_critical_error_count = 0;
    g_last_error_time = 0;

    // 记录错误处理系统初始化完成
    error_log(ERROR_LEVEL_INFO, 0, "Error handling system initialized");
}

/**
 * @brief  错误处理函数
 * @param  error_code 错误代码
 * @retval None
 */
void error_handler(uint16_t error_code)
{
    error_handler_with_info(error_code, "unknown", 0);
}

/**
 * @brief  带信息的错误处理函数
 * @param  error_code 错误代码
 * @param  file 文件名
 * @param  line 行号
 * @retval None
 */
void error_handler_with_info(uint16_t error_code, const char *file, uint32_t line)
{
    // 记录错误信息
    error_info_t error_info;
    error_info.error_code = error_code;
    error_info.level = ERROR_LEVEL_CRITICAL; // 默认为关键级别
    error_info.timestamp = xTaskGetTickCount();
    error_info.line_number = line;
    error_info.file_name = file;

    // 获取错误描述
    if (error_code < sizeof(error_descriptions) / sizeof(error_descriptions[0])) {
        error_info.description = error_descriptions[error_code];
    } else {
        error_info.description = "Unknown error";
    }

    // 记录到错误日志
    error_log_add(&error_info);

    // 根据错误类型采取相应措施
    switch (error_code) {
        case ERROR_BOARD_INIT_FAILED:
        case ERROR_SYSTEM_INIT_FAILED:
        case ERROR_SCHEDULER_FAILED:
            // 系统级错误，执行系统复位
            system_soft_reset();
            break;

        case ERROR_TASK_CREATE_FAILED:
        case ERROR_MEMORY_ALLOCATION:
            // 资源错误，尝试恢复
            if (!system_recovery_attempt(error_code)) {
                system_soft_reset();
            }
            break;

        case ERROR_SAFETY_VIOLATION:
        case ERROR_WATCHDOG_TIMEOUT:
            // 安全错误，立即紧急停止
            system_emergency_stop();
            break;

        case ERROR_STACK_OVERFLOW:
            // 栈溢出，立即复位
            system_soft_reset();
            break;

        default:
            // 其他错误，记录并继续运行
            break;
    }

    // 更新系统状态
    system_set_state(SYSTEM_STATE_ERROR);

    // 如果在严重错误处理过程中，进入无限循环
    if (error_code == ERROR_SCHEDULER_FAILED ||
        error_code == ERROR_STACK_OVERFLOW) {
        while (1) {
            // 等待看门狗复位
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * @brief  记录错误日志
 * @param  level 错误级别
 * @param  error_code 错误代码
 * @param  description 错误描述
 * @retval None
 */
void error_log(error_level_t level, uint16_t error_code, const char *description)
{
    error_info_t error_info;
    error_info.error_code = error_code;
    error_info.level = level;
    error_info.timestamp = xTaskGetTickCount();
    error_info.line_number = 0;
    error_info.file_name = "";
    error_info.description = description;

    error_log_add(&error_info);
}

/**
 * @brief  添加错误日志条目
 * @param  error 错误信息指针
 * @retval bool 添加结果
 */
bool error_log_add(const error_info_t *error)
{
    if (error == NULL || g_error_log_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(g_error_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    // 复制错误信息到日志缓冲区
    memcpy(&g_error_log[g_log_write_index], error, sizeof(error_info_t));

    // 更新写入索引（循环缓冲区）
    g_log_write_index = (g_log_write_index + 1) % MAX_ERROR_LOG_ENTRIES;

    // 更新计数器
    if (g_log_count < MAX_ERROR_LOG_ENTRIES) {
        g_log_count++;
    }

    // 更新统计信息
    g_total_error_count++;
    g_last_error_time = error->timestamp;

    if (error->level >= ERROR_LEVEL_CRITICAL) {
        g_critical_error_count++;
    }

    xSemaphoreGive(g_error_log_mutex);

    // 根据错误级别执行相应的通知
    switch (error->level) {
        case ERROR_LEVEL_CRITICAL:
        case ERROR_LEVEL_FATAL:
            // 关键和致命错误：立即通知用户
            // 这里可以触发蜂鸣器、LED等
            break;

        case ERROR_LEVEL_ERROR:
            // 一般错误：记录并可能通知用户
            break;

        case ERROR_LEVEL_WARNING:
            // 警告：仅记录
            break;

        case ERROR_LEVEL_INFO:
            // 信息：仅记录
            break;

        default:
            break;
    }

    return true;
}

/**
 * @brief  获取最后一个错误
 * @param  error 错误信息指针
 * @retval bool 获取结果
 */
bool error_log_get_last(error_info_t *error)
{
    if (error == NULL || g_error_log_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(g_error_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    if (g_log_count == 0) {
        xSemaphoreGive(g_error_log_mutex);
        return false;
    }

    // 计算最后一个错误的索引
    uint32_t last_index = (g_log_write_index - 1 + MAX_ERROR_LOG_ENTRIES) % MAX_ERROR_LOG_ENTRIES;

    // 复制错误信息
    memcpy(error, &g_error_log[last_index], sizeof(error_info_t));

    xSemaphoreGive(g_error_log_mutex);
    return true;
}

/**
 * @brief  获取错误日志计数
 * @param  None
 * @retval uint32_t 错误计数
 */
uint32_t error_log_get_count(void)
{
    uint32_t count = 0;

    if (g_error_log_mutex != NULL) {
        if (xSemaphoreTake(g_error_log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            count = g_log_count;
            xSemaphoreGive(g_error_log_mutex);
        }
    }

    return count;
}

/**
 * @brief  清空错误日志
 * @param  None
 * @retval None
 */
void error_log_clear(void)
{
    if (g_error_log_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(g_error_log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(g_error_log, 0, sizeof(g_error_log));
        g_log_write_index = 0;
        g_log_count = 0;
        g_total_error_count = 0;
        g_critical_error_count = 0;

        xSemaphoreGive(g_error_log_mutex);
    }
}

/**
 * @brief  获取错误日志条目
 * @param  index 索引
 * @param  error 错误信息指针
 * @retval bool 获取结果
 */
bool error_log_get_entry(uint32_t index, error_info_t *error)
{
    if (error == NULL || g_error_log_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(g_error_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    if (index >= g_log_count) {
        xSemaphoreGive(g_error_log_mutex);
        return false;
    }

    // 计算实际索引（考虑循环缓冲区）
    uint32_t actual_index;
    if (g_log_count < MAX_ERROR_LOG_ENTRIES) {
        actual_index = index;
    } else {
        actual_index = (g_log_write_index + index) % MAX_ERROR_LOG_ENTRIES;
    }

    // 复制错误信息
    memcpy(error, &g_error_log[actual_index], sizeof(error_info_t));

    xSemaphoreGive(g_error_log_mutex);
    return true;
}

/**
 * @brief  获取错误统计信息
 * @param  total_count 总错误数指针
 * @param  critical_count 关键错误数指针
 * @param  last_error_time 最后错误时间指针
 * @retval None
 */
void error_get_statistics(uint32_t *total_count, uint32_t *critical_count, uint32_t *last_error_time)
{
    if (g_error_log_mutex != NULL) {
        if (xSemaphoreTake(g_error_log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (total_count != NULL) {
                *total_count = g_total_error_count;
            }
            if (critical_count != NULL) {
                *critical_count = g_critical_error_count;
            }
            if (last_error_time != NULL) {
                *last_error_time = g_last_error_time;
            }
            xSemaphoreGive(g_error_log_mutex);
        }
    }
}

/**
 * @brief  格式化错误信息为字符串
 * @param  error 错误信息指针
 * @param  buffer 输出缓冲区
 * @param  buffer_size 缓冲区大小
 * @retval int 格式化的字符数
 */
int error_format_string(const error_info_t *error, char *buffer, size_t buffer_size)
{
    if (error == NULL || buffer == NULL || buffer_size == 0) {
        return 0;
    }

    const char* level_str[] = {
        "INFO", "WARN", "ERROR", "CRIT", "FATAL"
    };

    const char* level_name = (error->level < 5) ? level_str[error->level] : "UNKNOWN";

    return snprintf(buffer, buffer_size,
                   "[%lu] %s: 0x%04X - %s (%s:%lu)",
                   error->timestamp,
                   level_name,
                   error->error_code,
                   error->description ? error->description : "No description",
                   error->file_name ? error->file_name : "unknown",
                   error->line_number);
}

/**
 * @brief  检查是否有活跃的关键错误
 * @param  None
 * @retval bool 是否有关键错误
 */
bool error_has_critical_errors(void)
{
    return g_critical_error_count > 0;
}

/**
 * @brief  错误恢复尝试
 * @param  error_code 错误代码
 * @retval bool 恢复结果
 */
bool error_recovery_attempt(uint16_t error_code)
{
    // 记录恢复尝试
    char log_msg[64];
    snprintf(log_msg, sizeof(log_msg), "Recovery attempt for error 0x%04X", error_code);
    error_log(ERROR_LEVEL_INFO, 0, log_msg);

    // 调用系统管理器的恢复函数
    return system_recovery_attempt(error_code);
}

/**
 * @brief  获取错误级别字符串
 * @param  level 错误级别
 * @retval const char* 级别字符串
 */
const char* error_get_level_string(error_level_t level)
{
    switch (level) {
        case ERROR_LEVEL_INFO:     return "INFO";
        case ERROR_LEVEL_WARNING:  return "WARNING";
        case ERROR_LEVEL_ERROR:    return "ERROR";
        case ERROR_LEVEL_CRITICAL: return "CRITICAL";
        case ERROR_LEVEL_FATAL:    return "FATAL";
        default:                   return "UNKNOWN";
    }
}

/**
 * @brief  获取错误描述
 * @param  error_code 错误代码
 * @retval const char* 错误描述
 */
const char* error_get_description(uint16_t error_code)
{
    if (error_code < sizeof(error_descriptions) / sizeof(error_descriptions[0])) {
        return error_descriptions[error_code];
    }
    return "Unknown error";
}