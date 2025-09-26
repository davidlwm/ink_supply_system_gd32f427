/**
 * @file hal_manager.h
 * @brief HAL层管理器 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统HAL层统一管理器，管理所有HAL模块
 */

#ifndef HAL_MANAGER_H
#define HAL_MANAGER_H

#include "hal/hal_common.h"
#include "hal/gpio_hal.h"
#include "hal/adc_hal.h"
#include "hal/pwm_hal.h"
#include "hal/uart_hal.h"
#include "hal/spi_hal.h"
#include "hal/i2c_hal.h"
#include "hal/timer_hal.h"
#include "hal/eth_hal.h"
#include "hal/flash_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HAL模块ID */
typedef enum {
    HAL_MODULE_GPIO = 0,        /* GPIO模块 */
    HAL_MODULE_ADC = 1,         /* ADC模块 */
    HAL_MODULE_PWM = 2,         /* PWM模块 */
    HAL_MODULE_UART = 3,        /* UART模块 */
    HAL_MODULE_SPI = 4,         /* SPI模块 */
    HAL_MODULE_I2C = 5,         /* I2C模块 */
    HAL_MODULE_TIMER = 6,       /* 定时器模块 */
    HAL_MODULE_ETH = 7,         /* 以太网模块 */
    HAL_MODULE_FLASH = 8,       /* Flash模块 */
    HAL_MODULE_COUNT            /* 模块总数 */
} hal_module_id_t;

/* HAL模块状态 */
typedef enum {
    HAL_MODULE_STATE_UNINITIALIZED = 0, /* 未初始化 */
    HAL_MODULE_STATE_INITIALIZED = 1,   /* 已初始化 */
    HAL_MODULE_STATE_RUNNING = 2,       /* 运行中 */
    HAL_MODULE_STATE_ERROR = 3,         /* 错误状态 */
    HAL_MODULE_STATE_SUSPENDED = 4      /* 暂停状态 */
} hal_module_state_t;

/* HAL模块信息 */
typedef struct {
    hal_module_id_t id;         /* 模块ID */
    const char* name;           /* 模块名称 */
    hal_module_state_t state;   /* 模块状态 */
    uint32_t version;           /* 模块版本 */
    uint32_t init_time;         /* 初始化时间 */
    uint32_t error_count;       /* 错误计数 */
    uint32_t last_error;        /* 最后错误代码 */
    void* handle;               /* 模块句柄 */
} hal_module_info_t;

/* HAL管理器配置 */
typedef struct {
    hal_config_base_t base;     /* 基础配置 */
    bool auto_recovery;         /* 自动恢复使能 */
    uint32_t watchdog_timeout;  /* 看门狗超时时间 */
    uint32_t health_check_period; /* 健康检查周期 */
    hal_callback_t error_callback; /* 错误回调 */
    hal_callback_t recovery_callback; /* 恢复回调 */
} hal_manager_config_t;

/* HAL管理器句柄 */
typedef struct {
    hal_handle_base_t base;         /* 基础句柄 */
    hal_manager_config_t* config;   /* 管理器配置 */
    hal_module_info_t modules[HAL_MODULE_COUNT]; /* 模块信息数组 */
    uint32_t initialized_count;     /* 已初始化模块数 */
    bool system_ready;              /* 系统就绪标志 */
    TaskHandle_t monitor_task;      /* 监控任务句柄 */
} hal_manager_handle_t;

/* HAL系统统计信息 */
typedef struct {
    uint32_t total_modules;         /* 总模块数 */
    uint32_t initialized_modules;   /* 已初始化模块数 */
    uint32_t running_modules;       /* 运行中模块数 */
    uint32_t error_modules;         /* 错误模块数 */
    uint32_t total_errors;          /* 总错误数 */
    uint32_t total_recoveries;      /* 总恢复次数 */
    uint32_t uptime_seconds;        /* 系统运行时间 */
    uint32_t memory_usage;          /* 内存使用量 */
} hal_system_stats_t;

/* HAL管理器接口 */
hal_result_t hal_manager_init(hal_manager_handle_t* handle, const hal_manager_config_t* config);
hal_result_t hal_manager_deinit(hal_manager_handle_t* handle);
hal_result_t hal_manager_start_all_modules(hal_manager_handle_t* handle);
hal_result_t hal_manager_stop_all_modules(hal_manager_handle_t* handle);
hal_result_t hal_manager_restart_all_modules(hal_manager_handle_t* handle);

/* 模块管理接口 */
hal_result_t hal_manager_register_module(hal_manager_handle_t* handle, hal_module_id_t module_id, void* module_handle, const char* name);
hal_result_t hal_manager_unregister_module(hal_manager_handle_t* handle, hal_module_id_t module_id);
hal_result_t hal_manager_start_module(hal_manager_handle_t* handle, hal_module_id_t module_id);
hal_result_t hal_manager_stop_module(hal_manager_handle_t* handle, hal_module_id_t module_id);
hal_result_t hal_manager_restart_module(hal_manager_handle_t* handle, hal_module_id_t module_id);
hal_result_t hal_manager_suspend_module(hal_manager_handle_t* handle, hal_module_id_t module_id);
hal_result_t hal_manager_resume_module(hal_manager_handle_t* handle, hal_module_id_t module_id);

/* 状态查询接口 */
hal_result_t hal_manager_get_module_info(hal_manager_handle_t* handle, hal_module_id_t module_id, hal_module_info_t* info);
hal_result_t hal_manager_get_module_state(hal_manager_handle_t* handle, hal_module_id_t module_id, hal_module_state_t* state);
hal_result_t hal_manager_get_system_stats(hal_manager_handle_t* handle, hal_system_stats_t* stats);
hal_result_t hal_manager_is_system_ready(hal_manager_handle_t* handle, bool* ready);

/* 错误处理接口 */
hal_result_t hal_manager_report_error(hal_manager_handle_t* handle, hal_module_id_t module_id, hal_result_t error);
hal_result_t hal_manager_clear_module_errors(hal_manager_handle_t* handle, hal_module_id_t module_id);
hal_result_t hal_manager_clear_all_errors(hal_manager_handle_t* handle);
hal_result_t hal_manager_get_error_history(hal_manager_handle_t* handle, hal_module_id_t module_id, hal_result_t* errors, uint8_t* count);

/* 健康检查接口 */
hal_result_t hal_manager_health_check(hal_manager_handle_t* handle);
hal_result_t hal_manager_health_check_module(hal_manager_handle_t* handle, hal_module_id_t module_id);
hal_result_t hal_manager_start_health_monitor(hal_manager_handle_t* handle);
hal_result_t hal_manager_stop_health_monitor(hal_manager_handle_t* handle);

/* 电源管理接口 */
hal_result_t hal_manager_enter_low_power(hal_manager_handle_t* handle, hal_power_mode_t mode);
hal_result_t hal_manager_exit_low_power(hal_manager_handle_t* handle);
hal_result_t hal_manager_suspend_all_modules(hal_manager_handle_t* handle);
hal_result_t hal_manager_resume_all_modules(hal_manager_handle_t* handle);

/* 供墨系统专用接口 */
hal_result_t hal_manager_ink_system_init(void);
hal_result_t hal_manager_ink_system_deinit(void);
hal_result_t hal_manager_emergency_shutdown(void);
hal_result_t hal_manager_safe_restart(void);
hal_result_t hal_manager_get_system_health(uint8_t* health_percent);

/* 诊断和测试接口 */
hal_result_t hal_manager_self_test_all(hal_manager_handle_t* handle);
hal_result_t hal_manager_self_test_module(hal_manager_handle_t* handle, hal_module_id_t module_id);
hal_result_t hal_manager_generate_diagnostic_report(hal_manager_handle_t* handle, char* report, uint16_t max_length);

/* 配置管理接口 */
hal_result_t hal_manager_save_configuration(hal_manager_handle_t* handle);
hal_result_t hal_manager_load_configuration(hal_manager_handle_t* handle);
hal_result_t hal_manager_reset_to_defaults(hal_manager_handle_t* handle);

/* 回调函数类型 */
typedef void (*hal_manager_error_callback_t)(hal_module_id_t module_id, hal_result_t error);
typedef void (*hal_manager_recovery_callback_t)(hal_module_id_t module_id, bool success);
typedef void (*hal_manager_health_callback_t)(hal_system_stats_t* stats);

/* 回调注册接口 */
hal_result_t hal_manager_register_error_callback(hal_manager_handle_t* handle, hal_manager_error_callback_t callback);
hal_result_t hal_manager_register_recovery_callback(hal_manager_handle_t* handle, hal_manager_recovery_callback_t callback);
hal_result_t hal_manager_register_health_callback(hal_manager_handle_t* handle, hal_manager_health_callback_t callback);
hal_result_t hal_manager_unregister_callbacks(hal_manager_handle_t* handle);

/* 工具函数 */
const char* hal_manager_get_module_name(hal_module_id_t module_id);
const char* hal_manager_get_state_name(hal_module_state_t state);
const char* hal_manager_get_error_name(hal_result_t error);

#ifdef __cplusplus
}
#endif

#endif /* HAL_MANAGER_H */