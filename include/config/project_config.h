/**
 * @file project_config.h
 * @brief 供墨系统项目统一配置文件
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 基于8周v4标准的统一配置管理
 */

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 包含各层配置文件 */
#include "config/system_config.h"
#include "config/app_config.h"

/* 项目版本信息 */
#define PROJECT_VERSION_MAJOR       1
#define PROJECT_VERSION_MINOR       0
#define PROJECT_VERSION_PATCH       0
#define PROJECT_VERSION_BUILD       20241227

/* 项目标识 */
#define PROJECT_NAME                "InkSupplySystem"
#define PROJECT_DESCRIPTION         "供墨系统控制板卡 GD32F427"

/* 调试配置 */
#ifndef DEBUG
#define DEBUG                       0
#endif

#if DEBUG
    #define PROJECT_DEBUG_ENABLE    1
    #define PROJECT_LOG_LEVEL       3  /* 0=Error, 1=Warning, 2=Info, 3=Debug */
#else
    #define PROJECT_DEBUG_ENABLE    0
    #define PROJECT_LOG_LEVEL       1
#endif

/* 编译时配置验证 */
#if !defined(SYSTEM_CLOCK_FREQ)
    #error "SYSTEM_CLOCK_FREQ must be defined in system_config.h"
#endif

#if !defined(APP_TASK_COUNT_MAX)
    #error "APP_TASK_COUNT_MAX must be defined in app_config.h"
#endif

/* 全局功能开关 */
#define FEATURE_ETHERNET_ENABLE     1
#define FEATURE_ETHERCAT_ENABLE     1
#define FEATURE_FLASH_CONFIG_ENABLE 1
#define FEATURE_SAFETY_MONITOR_ENABLE 1
#define FEATURE_HMI_ENABLE          1

/* 内存配置 */
#define MEMORY_HEAP_SIZE            (32 * 1024)    /* 32KB堆大小 */
#define MEMORY_STACK_SIZE_DEFAULT   (2 * 1024)     /* 2KB默认栈大小 */

/* 通信配置 */
#define COMM_UART_COUNT             3
#define COMM_SPI_COUNT              2
#define COMM_I2C_COUNT              2

/* 传感器配置 */
#define SENSOR_COUNT_MAX            16
#define SENSOR_SAMPLE_RATE_DEFAULT  100  /* Hz */

/* 执行器配置 */
#define ACTUATOR_COUNT_MAX          8
#define ACTUATOR_PWM_FREQ_DEFAULT   1000 /* Hz */

/* 安全配置 */
#define SAFETY_WATCHDOG_TIMEOUT     5000 /* ms */
#define SAFETY_EMERGENCY_STOP_TIME  100  /* ms */

/* 性能监控配置 */
#define PERFORMANCE_MONITOR_ENABLE  1
#define PERFORMANCE_STATS_INTERVAL  1000 /* ms */

#ifdef __cplusplus
}
#endif

#endif /* PROJECT_CONFIG_H */