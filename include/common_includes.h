/**
 * @file common_includes.h
 * @brief 供墨系统通用包含文件
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 统一管理所有常用头文件，符合8周v4标准
 */

#ifndef COMMON_INCLUDES_H
#define COMMON_INCLUDES_H

/* 标准库 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* GD32F4xx HAL库 */
#include "gd32f4xx.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"

/* 项目配置 */
#include "config/project_config.h"

/* 系统层 */
#include "system/error_handler.h"
#include "system/system_manager.h"

/* 中间件层 */
#include "middleware/middleware_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 通用宏定义 */
#ifndef UNUSED
#define UNUSED(x)                   ((void)(x))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array)           (sizeof(array) / sizeof(array[0]))
#endif

#ifndef MIN
#define MIN(a, b)                   ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)                   ((a) > (b) ? (a) : (b))
#endif

#ifndef CLAMP
#define CLAMP(value, min, max)      (MIN(MAX(value, min), max))
#endif

/* 时间转换宏 */
#define MS_TO_TICKS(ms)             pdMS_TO_TICKS(ms)
#define TICKS_TO_MS(ticks)          ((ticks) * portTICK_PERIOD_MS)
#define SECONDS_TO_TICKS(sec)       pdMS_TO_TICKS((sec) * 1000)

/* 内存对齐宏 */
#define ALIGN_4BYTE(size)           (((size) + 3) & ~3)
#define ALIGN_8BYTE(size)           (((size) + 7) & ~7)

/* 位操作宏 */
#define SET_BIT(reg, bit)           ((reg) |= (bit))
#define CLEAR_BIT(reg, bit)         ((reg) &= ~(bit))
#define READ_BIT(reg, bit)          ((reg) & (bit))
#define CLEAR_REG(reg)              ((reg) = 0x0)
#define WRITE_REG(reg, val)         ((reg) = (val))
#define READ_REG(reg)               ((reg))
#define MODIFY_REG(reg, clearmask, setmask) \
    WRITE_REG((reg), (((READ_REG(reg)) & (~(clearmask))) | (setmask)))

/* 调试宏 */
#if PROJECT_DEBUG_ENABLE
    #define DEBUG_PRINT(fmt, ...)   printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
    #define DEBUG_ASSERT(expr)      do { if (!(expr)) { printf("[ASSERT] %s:%d\r\n", __FILE__, __LINE__); while(1); } } while(0)
#else
    #define DEBUG_PRINT(fmt, ...)   do {} while(0)
    #define DEBUG_ASSERT(expr)      do {} while(0)
#endif

/* 日志宏 */
#define LOG_ERROR(fmt, ...)         printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...)       printf("[WARN]  " fmt "\r\n", ##__VA_ARGS__)

#if PROJECT_LOG_LEVEL >= 2
    #define LOG_INFO(fmt, ...)      printf("[INFO]  " fmt "\r\n", ##__VA_ARGS__)
#else
    #define LOG_INFO(fmt, ...)      do {} while(0)
#endif

#if PROJECT_LOG_LEVEL >= 3
    #define LOG_DEBUG(fmt, ...)     printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define LOG_DEBUG(fmt, ...)     do {} while(0)
#endif

/* 错误处理宏 */
#define CHECK_RESULT(expr) do { \
    if ((expr) != 0) { \
        LOG_ERROR("Operation failed at %s:%d", __FILE__, __LINE__); \
        return APP_ERROR; \
    } \
} while(0)

#define CHECK_POINTER(ptr) do { \
    if ((ptr) == NULL) { \
        LOG_ERROR("Null pointer at %s:%d", __FILE__, __LINE__); \
        return APP_INVALID_PARAM; \
    } \
} while(0)

/* 性能测量宏 */
#if PERFORMANCE_MONITOR_ENABLE
    #define PERF_START(var)         uint32_t var = xTaskGetTickCount()
    #define PERF_END(var, name)     LOG_DEBUG("%s took %lu ms", name, xTaskGetTickCount() - var)
#else
    #define PERF_START(var)         do {} while(0)
    #define PERF_END(var, name)     do {} while(0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* COMMON_INCLUDES_H */