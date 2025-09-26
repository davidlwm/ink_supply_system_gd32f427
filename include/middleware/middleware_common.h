/**
 * @file middleware_common.h
 * @brief 中间件层通用定义 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 中间件层所有模块使用的通用类型定义和配置
 */

#ifndef MIDDLEWARE_COMMON_H
#define MIDDLEWARE_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =================================================================== */
/* 中间件版本信息 */
/* =================================================================== */
#define MIDDLEWARE_VERSION_MAJOR    4       /* 主版本号 */
#define MIDDLEWARE_VERSION_MINOR    0       /* 次版本号 */
#define MIDDLEWARE_VERSION_PATCH    0       /* 补丁版本号 */
#define MIDDLEWARE_VERSION_STRING   "V4.0.0" /* 版本字符串 */

/* =================================================================== */
/* 通用错误码定义 */
/* =================================================================== */
typedef enum {
    MW_OK = 0,                          /* 操作成功 */
    MW_ERROR = 1,                       /* 通用错误 */
    MW_ERROR_INVALID_PARAM = 2,         /* 无效参数 */
    MW_ERROR_NULL_POINTER = 3,          /* 空指针错误 */
    MW_ERROR_NOT_INITIALIZED = 4,       /* 未初始化 */
    MW_ERROR_ALREADY_INITIALIZED = 5,   /* 已初始化 */
    MW_ERROR_OUT_OF_MEMORY = 6,         /* 内存不足 */
    MW_ERROR_OUT_OF_RANGE = 7,          /* 超出范围 */
    MW_ERROR_TIMEOUT = 8,               /* 超时 */
    MW_ERROR_BUSY = 9,                  /* 忙碌状态 */
    MW_ERROR_NOT_SUPPORTED = 10,        /* 不支持的操作 */
    MW_ERROR_CALCULATION = 11,          /* 计算错误 */
    MW_ERROR_OVERFLOW = 12,             /* 溢出错误 */
    MW_ERROR_UNDERFLOW = 13             /* 下溢错误 */
} mw_result_t;

/* =================================================================== */
/* 中间件模块类型定义 */
/* =================================================================== */
typedef enum {
    MW_MODULE_FILTER = 0,               /* 滤波器模块 */
    MW_MODULE_PID = 1,                  /* PID控制器模块 */
    MW_MODULE_TASKS = 2,                /* 任务管理模块 */
    MW_MODULE_COUNT                     /* 模块总数 */
} mw_module_type_t;

/* =================================================================== */
/* 中间件配置参数 */
/* =================================================================== */

/* 滤波器配置 */
#define MW_MAX_FILTERS              32      /* 最大滤波器数量 */
#define MW_MAX_FILTER_WINDOW        64      /* 最大滤波器窗口 */
#define MW_FILTER_EPSILON           1e-6f   /* 浮点数精度 */

/* PID控制器配置 */
#define MW_MAX_PID_CONTROLLERS      16      /* 最大PID控制器数量 */
#define MW_PID_DEFAULT_DT           0.02f   /* 默认采样时间20ms */
#define MW_PID_MIN_DT               0.001f  /* 最小采样时间1ms */
#define MW_PID_MAX_DT               1.0f    /* 最大采样时间1s */

/* 任务管理配置 */
#define MW_MAX_MANAGED_TASKS        16      /* 最大管理任务数量 */
#define MW_TASK_NAME_MAX_LEN        16      /* 任务名称最大长度 */
#define MW_WATCHDOG_TIMEOUT_MS      5000    /* 看门狗超时时间 */

/* =================================================================== */
/* 通用数据结构 */
/* =================================================================== */

/* 时间戳结构 */
typedef struct {
    uint32_t seconds;                   /* 秒 */
    uint32_t microseconds;              /* 微秒 */
} mw_timestamp_t;

/* 统计信息结构 */
typedef struct {
    uint32_t process_count;             /* 处理次数 */
    uint32_t error_count;               /* 错误次数 */
    uint32_t last_process_time_us;      /* 上次处理时间(微秒) */
    uint32_t max_process_time_us;       /* 最大处理时间(微秒) */
    uint32_t min_process_time_us;       /* 最小处理时间(微秒) */
    float avg_process_time_us;          /* 平均处理时间(微秒) */
} mw_statistics_t;

/* 中间件模块状态 */
typedef enum {
    MW_STATE_UNINITIALIZED = 0,         /* 未初始化 */
    MW_STATE_INITIALIZING = 1,          /* 初始化中 */
    MW_STATE_READY = 2,                 /* 就绪 */
    MW_STATE_RUNNING = 3,               /* 运行中 */
    MW_STATE_ERROR = 4,                 /* 错误状态 */
    MW_STATE_SUSPENDED = 5              /* 挂起状态 */
} mw_state_t;

/* 中间件模块信息 */
typedef struct {
    mw_module_type_t type;              /* 模块类型 */
    const char* name;                   /* 模块名称 */
    const char* version;                /* 模块版本 */
    mw_state_t state;                   /* 模块状态 */
    uint32_t init_time;                 /* 初始化时间 */
    mw_statistics_t stats;              /* 统计信息 */
} mw_module_info_t;

/* =================================================================== */
/* 通用工具宏 */
/* =================================================================== */

/* 参数检查宏 */
#define MW_CHECK_PARAM(condition) \
    do { \
        if (!(condition)) { \
            return MW_ERROR_INVALID_PARAM; \
        } \
    } while(0)

#define MW_CHECK_NULL(ptr) \
    do { \
        if ((ptr) == NULL) { \
            return MW_ERROR_NULL_POINTER; \
        } \
    } while(0)

/* 范围检查宏 */
#define MW_CHECK_RANGE(value, min, max) \
    do { \
        if ((value) < (min) || (value) > (max)) { \
            return MW_ERROR_OUT_OF_RANGE; \
        } \
    } while(0)

/* 浮点数比较宏 */
#define MW_FLOAT_EQUAL(a, b) (fabsf((a) - (b)) < MW_FILTER_EPSILON)
#define MW_FLOAT_ZERO(a) (fabsf(a) < MW_FILTER_EPSILON)

/* 限幅宏 */
#define MW_CLAMP(value, min, max) \
    ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

/* 数组元素个数宏 */
#define MW_ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/* =================================================================== */
/* 通用接口函数 */
/* =================================================================== */

/* 中间件系统初始化/反初始化 */
mw_result_t middleware_system_init(void);
mw_result_t middleware_system_deinit(void);

/* 模块注册/注销 */
mw_result_t middleware_register_module(mw_module_type_t type, const char* name, const char* version);
mw_result_t middleware_unregister_module(mw_module_type_t type);

/* 模块状态管理 */
mw_result_t middleware_set_module_state(mw_module_type_t type, mw_state_t state);
mw_state_t middleware_get_module_state(mw_module_type_t type);
mw_result_t middleware_get_module_info(mw_module_type_t type, mw_module_info_t* info);

/* 统计信息管理 */
mw_result_t middleware_update_statistics(mw_module_type_t type, uint32_t process_time_us);
mw_result_t middleware_reset_statistics(mw_module_type_t type);
mw_result_t middleware_get_system_statistics(mw_statistics_t* total_stats);

/* 时间戳工具函数 */
mw_timestamp_t middleware_get_timestamp(void);
uint32_t middleware_timestamp_diff_us(mw_timestamp_t start, mw_timestamp_t end);
uint32_t middleware_get_tick_us(void);

/* 自检和诊断函数 */
mw_result_t middleware_self_test(void);
bool middleware_is_system_healthy(void);
mw_result_t middleware_dump_status(void);

#ifdef __cplusplus
}
#endif

#endif /* MIDDLEWARE_COMMON_H */