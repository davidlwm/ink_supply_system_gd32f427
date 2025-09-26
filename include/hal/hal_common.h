/**
 * @file hal_common.h
 * @brief HAL层通用定义和接口规范
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统HAL层统一接口规范，符合8周v4标准
 */

#ifndef HAL_COMMON_H
#define HAL_COMMON_H

#include "common_includes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HAL层版本信息 */
#define HAL_VERSION_MAJOR       1
#define HAL_VERSION_MINOR       0
#define HAL_VERSION_PATCH       0

/* HAL层通用错误码 */
typedef enum {
    HAL_OK = 0,                 /* 操作成功 */
    HAL_ERROR = 1,              /* 通用错误 */
    HAL_BUSY = 2,               /* 设备忙碌 */
    HAL_TIMEOUT = 3,            /* 操作超时 */
    HAL_INVALID_PARAM = 4,      /* 无效参数 */
    HAL_NOT_INITIALIZED = 5,    /* 未初始化 */
    HAL_NOT_SUPPORTED = 6,      /* 不支持的操作 */
    HAL_RESOURCE_LOCKED = 7,    /* 资源被锁定 */
    HAL_BUFFER_FULL = 8,        /* 缓冲区满 */
    HAL_BUFFER_EMPTY = 9,       /* 缓冲区空 */
    HAL_CRC_ERROR = 10,         /* CRC校验错误 */
    HAL_DMA_ERROR = 11,         /* DMA错误 */
    HAL_HARDWARE_ERROR = 12     /* 硬件错误 */
} hal_result_t;

/* HAL层设备状态 */
typedef enum {
    HAL_STATE_RESET = 0,        /* 复位状态 */
    HAL_STATE_READY = 1,        /* 就绪状态 */
    HAL_STATE_BUSY = 2,         /* 忙碌状态 */
    HAL_STATE_ERROR = 3         /* 错误状态 */
} hal_state_t;

/* HAL层电源模式 */
typedef enum {
    HAL_POWER_MODE_RUN = 0,     /* 运行模式 */
    HAL_POWER_MODE_SLEEP = 1,   /* 睡眠模式 */
    HAL_POWER_MODE_STOP = 2,    /* 停止模式 */
    HAL_POWER_MODE_STANDBY = 3  /* 待机模式 */
} hal_power_mode_t;

/* HAL层中断优先级 */
typedef enum {
    HAL_IRQ_PRIORITY_LOWEST = 15,   /* 最低优先级 */
    HAL_IRQ_PRIORITY_LOW = 12,      /* 低优先级 */
    HAL_IRQ_PRIORITY_MEDIUM = 8,    /* 中等优先级 */
    HAL_IRQ_PRIORITY_HIGH = 4,      /* 高优先级 */
    HAL_IRQ_PRIORITY_HIGHEST = 0    /* 最高优先级 */
} hal_irq_priority_t;

/* 通用回调函数类型 */
typedef void (*hal_callback_t)(void* context);
typedef void (*hal_error_callback_t)(hal_result_t error, void* context);

/* HAL层统计信息 */
typedef struct {
    uint32_t init_count;            /* 初始化次数 */
    uint32_t operation_count;       /* 操作次数 */
    uint32_t error_count;           /* 错误次数 */
    uint32_t timeout_count;         /* 超时次数 */
    uint32_t last_error_code;       /* 最后错误码 */
    uint32_t last_error_timestamp;  /* 最后错误时间戳 */
} hal_statistics_t;

/* HAL层配置结构体基类 */
typedef struct {
    uint32_t magic;                 /* 魔数标识 */
    uint32_t version;               /* 版本号 */
    uint32_t size;                  /* 结构体大小 */
    hal_irq_priority_t irq_priority; /* 中断优先级 */
    hal_callback_t init_callback;   /* 初始化回调 */
    hal_error_callback_t error_callback; /* 错误回调 */
    void* user_context;             /* 用户上下文 */
} hal_config_base_t;

/* HAL层句柄结构体基类 */
typedef struct {
    hal_state_t state;              /* 设备状态 */
    hal_config_base_t* config;      /* 配置指针 */
    hal_statistics_t stats;         /* 统计信息 */
    SemaphoreHandle_t mutex;        /* 互斥锁 */
    void* hw_instance;              /* 硬件实例 */
    bool initialized;               /* 初始化标志 */
} hal_handle_base_t;

/* HAL层通用宏定义 */
#define HAL_CONFIG_MAGIC            0x48414C00  /* "HAL\0" */
#define HAL_TIMEOUT_DEFAULT         1000        /* 默认超时时间(ms) */
#define HAL_RETRY_COUNT_DEFAULT     3           /* 默认重试次数 */

/* 参数检查宏 */
#define HAL_CHECK_HANDLE(handle) do { \
    if ((handle) == NULL || !(handle)->initialized) { \
        return HAL_INVALID_PARAM; \
    } \
} while(0)

#define HAL_CHECK_PARAM(param) do { \
    if ((param) == NULL) { \
        return HAL_INVALID_PARAM; \
    } \
} while(0)

#define HAL_CHECK_STATE(handle, expected_state) do { \
    if ((handle)->state != (expected_state)) { \
        return HAL_BUSY; \
    } \
} while(0)

/* 锁定/解锁宏 */
#define HAL_LOCK(handle) do { \
    if (xSemaphoreTake((handle)->mutex, pdMS_TO_TICKS(HAL_TIMEOUT_DEFAULT)) != pdTRUE) { \
        return HAL_TIMEOUT; \
    } \
} while(0)

#define HAL_UNLOCK(handle) do { \
    xSemaphoreGive((handle)->mutex); \
} while(0)

/* 统计更新宏 */
#define HAL_UPDATE_STATS_SUCCESS(handle) do { \
    (handle)->stats.operation_count++; \
} while(0)

#define HAL_UPDATE_STATS_ERROR(handle, error) do { \
    (handle)->stats.error_count++; \
    (handle)->stats.last_error_code = (error); \
    (handle)->stats.last_error_timestamp = xTaskGetTickCount(); \
} while(0)

/* HAL层通用接口函数 */
hal_result_t hal_get_version(uint32_t* major, uint32_t* minor, uint32_t* patch);
hal_result_t hal_system_init(void);
hal_result_t hal_system_deinit(void);
hal_result_t hal_enter_power_mode(hal_power_mode_t mode);
hal_result_t hal_exit_power_mode(void);
hal_result_t hal_get_system_clock(uint32_t* clock_freq);
hal_result_t hal_delay_ms(uint32_t ms);
hal_result_t hal_delay_us(uint32_t us);

/* 通用处理函数 */
hal_result_t hal_handle_init(hal_handle_base_t* handle, const hal_config_base_t* config);
hal_result_t hal_handle_deinit(hal_handle_base_t* handle);
hal_result_t hal_get_statistics(hal_handle_base_t* handle, hal_statistics_t* stats);
hal_result_t hal_clear_statistics(hal_handle_base_t* handle);

#ifdef __cplusplus
}
#endif

#endif /* HAL_COMMON_H */