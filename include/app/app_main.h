/**
 * @file app_main.h
 * @brief 应用层主入口头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统应用层统一入口，管理所有应用任务和模块
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* 项目配置 */
#include "config/project_config.h"
#include "app/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 应用层版本信息 */
#define APP_VERSION_MAJOR       4
#define APP_VERSION_MINOR       0
#define APP_VERSION_PATCH       0
#define APP_BUILD_DATE          "2024-12-27"

/* 应用层错误码定义 */
typedef enum {
    APP_OK = 0,
    APP_ERROR = 1,
    APP_BUSY = 2,
    APP_TIMEOUT = 3,
    APP_INVALID_PARAM = 4,
    APP_NOT_INITIALIZED = 5,
    APP_TASK_CREATE_FAILED = 6,
    APP_RESOURCE_EXHAUSTED = 7
} app_result_t;

/* 应用运行状态 */
typedef enum {
    APP_STATE_STOPPED = 0,
    APP_STATE_STARTING = 1,
    APP_STATE_RUNNING = 2,
    APP_STATE_STOPPING = 3,
    APP_STATE_ERROR = 4
} app_state_t;

/* 任务优先级定义 (基于8周v4标准) */
#define APP_PRIORITY_CRITICAL   7   /* 安全监控任务 */
#define APP_PRIORITY_HIGH       6   /* 控制任务 */
#define APP_PRIORITY_NORMAL     5   /* 传感器任务 */
#define APP_PRIORITY_LOW        4   /* HMI、通信任务 */
#define APP_PRIORITY_BACKGROUND 3   /* 配置、统计任务 */

/* 任务栈大小定义 */
#define APP_STACK_SIZE_LARGE    2048
#define APP_STACK_SIZE_MEDIUM   1024
#define APP_STACK_SIZE_SMALL    512

/* 任务句柄结构体 */
typedef struct {
    TaskHandle_t safety_task;       /* 安全监控任务 */
    TaskHandle_t control_task;      /* 控制任务 */
    TaskHandle_t sensor_task;       /* 传感器任务 */
    TaskHandle_t actuator_task;     /* 执行器任务 */
    TaskHandle_t hmi_task;          /* 人机交互任务 */
    TaskHandle_t comm_task;         /* 通信任务 */
    TaskHandle_t config_task;       /* 配置管理任务 */
} app_task_handles_t;

/* 应用系统状态 */
typedef struct {
    app_state_t state;              /* 运行状态 */
    uint32_t uptime_seconds;        /* 运行时间(秒) */
    uint32_t cycle_count;           /* 运行周期计数 */
    uint32_t error_count;           /* 错误计数 */
    uint32_t free_heap_size;        /* 剩余堆内存 */
    uint32_t min_free_heap_size;    /* 历史最小剩余堆内存 */
    float cpu_usage_percent;        /* CPU使用率 */
} app_system_status_t;

/* 应用配置结构体 */
typedef struct {
    bool enable_watchdog;           /* 看门狗使能 */
    bool enable_stack_monitor;      /* 栈监控使能 */
    uint32_t statistics_period_ms;  /* 统计周期 */
    uint32_t heartbeat_period_ms;   /* 心跳周期 */
} app_config_t;

/* 主要接口函数 */
app_result_t app_main_init(const app_config_t* config);
app_result_t app_main_start(void);
app_result_t app_main_stop(void);
app_result_t app_main_deinit(void);

/* 状态查询接口 */
app_result_t app_get_system_status(app_system_status_t* status);
app_state_t app_get_state(void);
app_result_t app_get_task_handles(app_task_handles_t* handles);

/* 任务管理接口 */
app_result_t app_suspend_all_tasks(void);
app_result_t app_resume_all_tasks(void);
app_result_t app_restart_task(TaskHandle_t task_handle);

/* 系统监控接口 */
app_result_t app_update_statistics(void);
app_result_t app_check_stack_usage(void);
app_result_t app_check_heap_usage(void);
float app_get_cpu_usage(void);

/* 错误处理接口 */
app_result_t app_handle_error(const char* task_name, uint32_t error_code);
app_result_t app_clear_error_count(void);
uint32_t app_get_error_count(void);

/* 看门狗接口 */
app_result_t app_feed_watchdog(void);
app_result_t app_register_watchdog_task(TaskHandle_t task_handle);

/* 系统重启接口 */
app_result_t app_request_system_restart(void);
app_result_t app_emergency_shutdown(void);

/* 回调函数类型 */
typedef void (*app_error_callback_t)(const char* task_name, uint32_t error_code);
typedef void (*app_state_change_callback_t)(app_state_t old_state, app_state_t new_state);

/* 回调注册接口 */
app_result_t app_register_error_callback(app_error_callback_t callback);
app_result_t app_register_state_change_callback(app_state_change_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */