/**
 * @file tasks.h
 * @brief FreeRTOS任务管理头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 任务管理模块，提供任务监控、统计和诊断功能
 */

#ifndef TASKS_H
#define TASKS_H

#include "middleware_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =================================================================== */
/* 任务管理配置 */
/* =================================================================== */
#define TASK_MAX_MANAGED_TASKS      16      /* 最大管理任务数量 */
#define TASK_NAME_MAX_LEN           16      /* 任务名称最大长度 */
#define TASK_WATCHDOG_MULTIPLIER    3       /* 看门狗超时倍数 */
#define TASK_MIN_FREE_HEAP_SIZE     1024    /* 最小空闲堆大小 */
#define TASK_STACK_CHECK_THRESHOLD  85      /* 栈使用率阈值(%) */

/* =================================================================== */
/* 任务状态和优先级定义 */
/* =================================================================== */

/* 任务运行状态 */
typedef enum {
    TASK_STATE_RUNNING = 0,             /* 运行中 */
    TASK_STATE_READY = 1,               /* 就绪 */
    TASK_STATE_BLOCKED = 2,             /* 阻塞 */
    TASK_STATE_SUSPENDED = 3,           /* 挂起 */
    TASK_STATE_DELETED = 4,             /* 已删除 */
    TASK_STATE_INVALID = 5              /* 无效状态 */
} task_run_state_t;

/* 任务健康状态 */
typedef enum {
    TASK_HEALTH_GOOD = 0,               /* 健康 */
    TASK_HEALTH_WARNING = 1,            /* 警告 */
    TASK_HEALTH_ERROR = 2,              /* 错误 */
    TASK_HEALTH_CRITICAL = 3            /* 严重错误 */
} task_health_t;

/* 任务优先级类别 */
typedef enum {
    TASK_PRIORITY_IDLE = 0,             /* 空闲优先级 */
    TASK_PRIORITY_LOW = 1,              /* 低优先级 */
    TASK_PRIORITY_NORMAL = 2,           /* 普通优先级 */
    TASK_PRIORITY_HIGH = 3,             /* 高优先级 */
    TASK_PRIORITY_CRITICAL = 4,         /* 关键优先级 */
    TASK_PRIORITY_REALTIME = 5          /* 实时优先级 */
} task_priority_class_t;

/* =================================================================== */
/* 任务信息和统计结构 */
/* =================================================================== */

/* 任务性能统计 */
typedef struct {
    uint32_t run_count;                 /* 运行次数 */
    uint32_t total_runtime_us;          /* 总运行时间(微秒) */
    uint32_t max_runtime_us;            /* 最大运行时间(微秒) */
    uint32_t min_runtime_us;            /* 最小运行时间(微秒) */
    float avg_runtime_us;               /* 平均运行时间(微秒) */
    float cpu_usage_percent;            /* CPU使用率(%) */
    uint32_t stack_high_water_mark;     /* 栈高水位标记 */
    float stack_usage_percent;          /* 栈使用率(%) */
    uint32_t missed_deadlines;          /* 错过的截止时间 */
    uint32_t watchdog_violations;       /* 看门狗违规次数 */
} task_performance_stats_t;

/* 任务时间信息 */
typedef struct {
    uint32_t period_ms;                 /* 任务周期(毫秒) */
    uint32_t deadline_ms;               /* 截止时间(毫秒) */
    uint32_t last_run_time;             /* 上次运行时间 */
    uint32_t next_run_time;             /* 下次运行时间 */
    uint32_t created_time;              /* 创建时间 */
    uint32_t total_delay_ms;            /* 总延迟时间 */
} task_timing_info_t;

/* 任务配置信息 */
typedef struct {
    UBaseType_t priority;               /* FreeRTOS优先级 */
    task_priority_class_t priority_class; /* 优先级类别 */
    uint16_t stack_size;                /* 栈大小(字) */
    bool watchdog_enabled;              /* 看门狗使能 */
    uint32_t watchdog_timeout_ms;       /* 看门狗超时时间 */
    bool performance_monitoring;        /* 性能监控使能 */
} task_config_t;

/* 完整任务信息 */
typedef struct {
    /* 基础信息 */
    char name[TASK_NAME_MAX_LEN];       /* 任务名称 */
    uint8_t id;                         /* 任务ID */
    TaskHandle_t handle;                /* FreeRTOS任务句柄 */
    bool registered;                    /* 注册标志 */

    /* 配置信息 */
    task_config_t config;               /* 任务配置 */

    /* 状态信息 */
    task_run_state_t state;             /* 运行状态 */
    task_health_t health;               /* 健康状态 */
    bool enabled;                       /* 使能状态 */

    /* 时间信息 */
    task_timing_info_t timing;          /* 时间信息 */

    /* 性能统计 */
    task_performance_stats_t stats;     /* 性能统计 */

    /* 中间件统计 */
    mw_statistics_t mw_stats;           /* 中间件统计 */
} task_info_t;

/* =================================================================== */
/* 系统级统计和状态 */
/* =================================================================== */

/* 系统内存信息 */
typedef struct {
    uint32_t total_heap_size;           /* 总堆大小 */
    uint32_t free_heap_size;            /* 空闲堆大小 */
    uint32_t min_ever_free_heap_size;   /* 历史最小空闲堆 */
    uint32_t malloc_failed_count;       /* 内存分配失败次数 */
    float heap_fragmentation_percent;   /* 堆碎片率 */
} system_memory_info_t;

/* 系统负载信息 */
typedef struct {
    float cpu_usage_percent;            /* 总CPU使用率 */
    float idle_task_usage_percent;      /* 空闲任务CPU使用率 */
    uint32_t context_switches_per_sec;  /* 每秒上下文切换次数 */
    uint32_t interrupts_per_sec;        /* 每秒中断次数 */
    uint32_t tick_count;                /* 系统时钟节拍数 */
    uint32_t uptime_seconds;            /* 系统运行时间(秒) */
} system_load_info_t;

/* 任务系统状态 */
typedef struct {
    uint8_t total_tasks;                /* 总任务数 */
    uint8_t registered_tasks;           /* 已注册任务数 */
    uint8_t running_tasks;              /* 运行中任务数 */
    uint8_t ready_tasks;                /* 就绪任务数 */
    uint8_t blocked_tasks;              /* 阻塞任务数 */
    uint8_t suspended_tasks;            /* 挂起任务数 */
    uint8_t healthy_tasks;              /* 健康任务数 */
    uint8_t warning_tasks;              /* 警告任务数 */
    uint8_t error_tasks;                /* 错误任务数 */
    system_memory_info_t memory;        /* 内存信息 */
    system_load_info_t load;            /* 负载信息 */
    mw_statistics_t total_stats;        /* 总体统计 */
} task_system_status_t;

/* =================================================================== */
/* 任务管理接口函数 */
/* =================================================================== */

/* 任务管理系统初始化/反初始化 */
mw_result_t task_manager_init(void);
mw_result_t task_manager_deinit(void);
mw_result_t task_manager_start_monitoring(void);
mw_result_t task_manager_stop_monitoring(void);

/* =================================================================== */
/* 任务注册和管理 */
/* =================================================================== */

/* 任务注册 */
mw_result_t task_register(const char* name, TaskHandle_t handle, const task_config_t* config);
mw_result_t task_unregister(const char* name);
mw_result_t task_unregister_by_handle(TaskHandle_t handle);

/* 任务查找 */
int task_find_by_name(const char* name);
int task_find_by_handle(TaskHandle_t handle);
TaskHandle_t task_get_handle_by_id(uint8_t task_id);
TaskHandle_t task_get_handle_by_name(const char* name);

/* 任务信息获取 */
mw_result_t task_get_info(uint8_t task_id, task_info_t* info);
mw_result_t task_get_info_by_name(const char* name, task_info_t* info);
mw_result_t task_get_all_info(task_info_t* info_array, uint8_t max_count, uint8_t* actual_count);

/* =================================================================== */
/* 任务控制和状态管理 */
/* =================================================================== */

/* 任务控制 */
mw_result_t task_suspend_by_id(uint8_t task_id);
mw_result_t task_resume_by_id(uint8_t task_id);
mw_result_t task_suspend_by_name(const char* name);
mw_result_t task_resume_by_name(const char* name);
mw_result_t task_enable_by_id(uint8_t task_id, bool enable);
mw_result_t task_enable_by_name(const char* name, bool enable);

/* 优先级管理 */
mw_result_t task_set_priority_by_id(uint8_t task_id, UBaseType_t priority);
mw_result_t task_set_priority_by_name(const char* name, UBaseType_t priority);
UBaseType_t task_get_priority_by_id(uint8_t task_id);
UBaseType_t task_get_priority_by_name(const char* name);

/* 批量操作 */
mw_result_t task_suspend_all_managed(void);
mw_result_t task_resume_all_managed(void);
mw_result_t task_reset_all_statistics(void);

/* =================================================================== */
/* 性能监控和统计 */
/* =================================================================== */

/* 性能监控 */
mw_result_t task_update_performance_stats(uint8_t task_id);
mw_result_t task_update_all_performance_stats(void);
mw_result_t task_reset_performance_stats(uint8_t task_id);

/* 统计信息获取 */
mw_result_t task_get_performance_stats(uint8_t task_id, task_performance_stats_t* stats);
mw_result_t task_get_timing_info(uint8_t task_id, task_timing_info_t* timing);
mw_result_t task_get_system_status(task_system_status_t* status);

/* CPU使用率计算 */
float task_get_cpu_usage_by_id(uint8_t task_id);
float task_get_cpu_usage_by_name(const char* name);
float task_get_total_cpu_usage(void);
mw_result_t task_calculate_cpu_usage_all(void);

/* 栈使用情况 */
uint32_t task_get_stack_high_water_mark(uint8_t task_id);
float task_get_stack_usage_percent(uint8_t task_id);
mw_result_t task_check_stack_overflow_all(void);

/* =================================================================== */
/* 看门狗和健康监控 */
/* =================================================================== */

/* 看门狗管理 */
mw_result_t task_watchdog_feed(uint8_t task_id);
mw_result_t task_watchdog_feed_by_name(const char* name);
mw_result_t task_watchdog_check_all(void);
mw_result_t task_watchdog_enable(uint8_t task_id, bool enable);
mw_result_t task_watchdog_set_timeout(uint8_t task_id, uint32_t timeout_ms);

/* 健康状态管理 */
task_health_t task_get_health_status(uint8_t task_id);
mw_result_t task_set_health_status(uint8_t task_id, task_health_t health);
mw_result_t task_update_health_status_all(void);
uint8_t task_get_unhealthy_count(void);

/* 截止时间管理 */
mw_result_t task_set_deadline(uint8_t task_id, uint32_t deadline_ms);
mw_result_t task_check_deadline(uint8_t task_id);
mw_result_t task_check_all_deadlines(void);

/* =================================================================== */
/* 系统级诊断和调试 */
/* =================================================================== */

/* 系统诊断 */
mw_result_t task_system_self_test(void);
bool task_is_system_healthy(void);
mw_result_t task_dump_all_info(void);
mw_result_t task_generate_report(char* buffer, uint32_t buffer_size);

/* 内存监控 */
mw_result_t task_check_memory_health(void);
mw_result_t task_get_memory_info(system_memory_info_t* memory_info);
bool task_is_low_memory(void);

/* 负载监控 */
mw_result_t task_get_load_info(system_load_info_t* load_info);
bool task_is_system_overloaded(void);
float task_get_system_efficiency(void);

/* 枚举和迭代 */
mw_result_t task_enum_all_registered(uint8_t* task_ids, uint8_t max_count, uint8_t* actual_count);
mw_result_t task_enum_by_state(task_run_state_t state, uint8_t* task_ids, uint8_t max_count, uint8_t* actual_count);
mw_result_t task_enum_by_health(task_health_t health, uint8_t* task_ids, uint8_t max_count, uint8_t* actual_count);

/* =================================================================== */
/* 配置和工具函数 */
/* =================================================================== */

/* 配置管理 */
mw_result_t task_set_config(uint8_t task_id, const task_config_t* config);
mw_result_t task_get_config(uint8_t task_id, task_config_t* config);
mw_result_t task_set_monitoring_period(uint32_t period_ms);
uint32_t task_get_monitoring_period(void);

/* 工具函数 */
const char* task_state_to_string(task_run_state_t state);
const char* task_health_to_string(task_health_t health);
const char* task_priority_class_to_string(task_priority_class_t priority_class);
bool task_is_valid_id(uint8_t task_id);

/* 时间工具 */
uint32_t task_get_current_time_ms(void);
uint32_t task_get_uptime_seconds(void);
mw_result_t task_delay_until_next_period(uint8_t task_id);

/* =================================================================== */
/* 高级功能 */
/* =================================================================== */

/* 任务通信统计 */
mw_result_t task_record_ipc_event(uint8_t task_id, const char* event_type);
mw_result_t task_get_ipc_statistics(uint8_t task_id, uint32_t* send_count, uint32_t* receive_count);

/* 任务调度分析 */
mw_result_t task_analyze_scheduling_behavior(uint8_t task_id, float* response_time_avg, float* jitter);
mw_result_t task_detect_priority_inversion(void);

/* 性能优化建议 */
mw_result_t task_get_optimization_suggestions(uint8_t task_id, char* suggestions, uint32_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_H */