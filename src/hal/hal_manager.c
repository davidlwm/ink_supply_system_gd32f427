/**
 * @file hal_manager.c
 * @brief HAL层管理器实现
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统HAL层统一管理器实现
 */

#include "hal/hal_manager.h"
#include <string.h>

/* 私有变量 */
static hal_manager_handle_t* g_hal_manager = NULL;
static const char* g_module_names[HAL_MODULE_COUNT] = {
    "GPIO", "ADC", "PWM", "UART", "SPI", "I2C", "TIMER", "ETH", "FLASH"
};

/**
 * @brief HAL管理器初始化
 * @param handle HAL管理器句柄指针
 * @param config HAL管理器配置指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_init(hal_manager_handle_t* handle, const hal_manager_config_t* config)
{
    HAL_CHECK_PARAM(handle);
    HAL_CHECK_PARAM(config);

    /* 初始化基础句柄 */
    hal_result_t result = hal_handle_init(&handle->base, &config->base);
    if (result != HAL_OK) {
        return result;
    }

    /* 设置管理器配置 */
    handle->config = (hal_manager_config_t*)config;

    /* 初始化模块信息 */
    for (int i = 0; i < HAL_MODULE_COUNT; i++) {
        handle->modules[i].id = (hal_module_id_t)i;
        handle->modules[i].name = g_module_names[i];
        handle->modules[i].state = HAL_MODULE_STATE_UNINITIALIZED;
        handle->modules[i].version = 0x00010000; /* V1.0.0 */
        handle->modules[i].init_time = 0;
        handle->modules[i].error_count = 0;
        handle->modules[i].last_error = HAL_OK;
        handle->modules[i].handle = NULL;
    }

    handle->initialized_count = 0;
    handle->system_ready = false;
    handle->monitor_task = NULL;

    /* 设置全局管理器指针 */
    g_hal_manager = handle;

    return HAL_OK;
}

/**
 * @brief HAL管理器反初始化
 * @param handle HAL管理器句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_deinit(hal_manager_handle_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    /* 停止所有模块 */
    hal_manager_stop_all_modules(handle);

    /* 删除监控任务 */
    if (handle->monitor_task != NULL) {
        vTaskDelete(handle->monitor_task);
        handle->monitor_task = NULL;
    }

    /* 清除全局管理器指针 */
    g_hal_manager = NULL;

    /* 反初始化基础句柄 */
    return hal_handle_deinit(&handle->base);
}

/**
 * @brief 注册模块
 * @param handle HAL管理器句柄指针
 * @param module_id 模块ID
 * @param module_handle 模块句柄
 * @param name 模块名称
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_register_module(hal_manager_handle_t* handle, hal_module_id_t module_id, void* module_handle, const char* name)
{
    HAL_CHECK_HANDLE(handle);

    if (module_id >= HAL_MODULE_COUNT) {
        return HAL_INVALID_PARAM;
    }

    HAL_LOCK(handle);

    handle->modules[module_id].handle = module_handle;
    handle->modules[module_id].state = HAL_MODULE_STATE_INITIALIZED;
    handle->modules[module_id].init_time = xTaskGetTickCount();

    if (name != NULL) {
        handle->modules[module_id].name = name;
    }

    handle->initialized_count++;

    HAL_UNLOCK(handle);

    return HAL_OK;
}

/**
 * @brief 启动所有模块
 * @param handle HAL管理器句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_start_all_modules(hal_manager_handle_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    hal_result_t result = HAL_OK;

    for (int i = 0; i < HAL_MODULE_COUNT; i++) {
        if (handle->modules[i].handle != NULL &&
            handle->modules[i].state == HAL_MODULE_STATE_INITIALIZED) {

            handle->modules[i].state = HAL_MODULE_STATE_RUNNING;
        }
    }

    handle->system_ready = true;
    return result;
}

/**
 * @brief 停止所有模块
 * @param handle HAL管理器句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_stop_all_modules(hal_manager_handle_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    for (int i = 0; i < HAL_MODULE_COUNT; i++) {
        if (handle->modules[i].state == HAL_MODULE_STATE_RUNNING) {
            handle->modules[i].state = HAL_MODULE_STATE_INITIALIZED;
        }
    }

    handle->system_ready = false;
    return HAL_OK;
}

/**
 * @brief 获取模块信息
 * @param handle HAL管理器句柄指针
 * @param module_id 模块ID
 * @param info 模块信息指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_get_module_info(hal_manager_handle_t* handle, hal_module_id_t module_id, hal_module_info_t* info)
{
    HAL_CHECK_HANDLE(handle);
    HAL_CHECK_PARAM(info);

    if (module_id >= HAL_MODULE_COUNT) {
        return HAL_INVALID_PARAM;
    }

    HAL_LOCK(handle);
    memcpy(info, &handle->modules[module_id], sizeof(hal_module_info_t));
    HAL_UNLOCK(handle);

    return HAL_OK;
}

/**
 * @brief 获取系统统计信息
 * @param handle HAL管理器句柄指针
 * @param stats 统计信息指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_get_system_stats(hal_manager_handle_t* handle, hal_system_stats_t* stats)
{
    HAL_CHECK_HANDLE(handle);
    HAL_CHECK_PARAM(stats);

    HAL_LOCK(handle);

    stats->total_modules = HAL_MODULE_COUNT;
    stats->initialized_modules = 0;
    stats->running_modules = 0;
    stats->error_modules = 0;
    stats->total_errors = 0;
    stats->total_recoveries = 0;
    stats->uptime_seconds = xTaskGetTickCount() / 1000;
    stats->memory_usage = xPortGetFreeHeapSize();

    for (int i = 0; i < HAL_MODULE_COUNT; i++) {
        if (handle->modules[i].state != HAL_MODULE_STATE_UNINITIALIZED) {
            stats->initialized_modules++;
        }
        if (handle->modules[i].state == HAL_MODULE_STATE_RUNNING) {
            stats->running_modules++;
        }
        if (handle->modules[i].state == HAL_MODULE_STATE_ERROR) {
            stats->error_modules++;
        }
        stats->total_errors += handle->modules[i].error_count;
    }

    HAL_UNLOCK(handle);

    return HAL_OK;
}

/**
 * @brief 报告模块错误
 * @param handle HAL管理器句柄指针
 * @param module_id 模块ID
 * @param error 错误代码
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_report_error(hal_manager_handle_t* handle, hal_module_id_t module_id, hal_result_t error)
{
    HAL_CHECK_HANDLE(handle);

    if (module_id >= HAL_MODULE_COUNT) {
        return HAL_INVALID_PARAM;
    }

    HAL_LOCK(handle);

    handle->modules[module_id].error_count++;
    handle->modules[module_id].last_error = error;
    handle->modules[module_id].state = HAL_MODULE_STATE_ERROR;

    HAL_UNLOCK(handle);

    /* 调用错误回调 */
    if (handle->config->error_callback != NULL) {
        handle->config->error_callback(handle->config->user_context);
    }

    return HAL_OK;
}

/**
 * @brief 供墨系统初始化
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_ink_system_init(void)
{
    /* 初始化HAL系统 */
    hal_result_t result = hal_system_init();
    if (result != HAL_OK) {
        return result;
    }

    /* 初始化各个HAL模块的系统部分 */
    result = gpio_hal_system_init();
    if (result != HAL_OK) {
        return result;
    }

    result = adc_hal_system_init();
    if (result != HAL_OK) {
        return result;
    }

    result = pwm_hal_system_init();
    if (result != HAL_OK) {
        return result;
    }

    result = i2c_hal_system_init();
    if (result != HAL_OK) {
        return result;
    }

    result = timer_hal_system_init();
    if (result != HAL_OK) {
        return result;
    }

    return HAL_OK;
}

/**
 * @brief 供墨系统反初始化
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_ink_system_deinit(void)
{
    /* 反初始化各个HAL模块 */
    timer_hal_system_deinit();
    i2c_hal_system_deinit();
    pwm_hal_system_deinit();
    adc_hal_system_deinit();
    gpio_hal_system_deinit();

    /* 反初始化HAL系统 */
    return hal_system_deinit();
}

/**
 * @brief 紧急关闭
 * @return hal_result_t 操作结果
 */
hal_result_t hal_manager_emergency_shutdown(void)
{
    if (g_hal_manager != NULL) {
        /* 停止所有模块 */
        hal_manager_stop_all_modules(g_hal_manager);
    }

    /* 执行紧急GPIO关闭 */
    gpio_hal_emergency_shutdown();

    /* 停止所有PWM输出 */
    pwm_hal_emergency_stop_all();

    return HAL_OK;
}

/**
 * @brief 获取模块名称
 * @param module_id 模块ID
 * @return const char* 模块名称
 */
const char* hal_manager_get_module_name(hal_module_id_t module_id)
{
    if (module_id >= HAL_MODULE_COUNT) {
        return "UNKNOWN";
    }

    return g_module_names[module_id];
}

/**
 * @brief 获取状态名称
 * @param state 状态
 * @return const char* 状态名称
 */
const char* hal_manager_get_state_name(hal_module_state_t state)
{
    switch (state) {
        case HAL_MODULE_STATE_UNINITIALIZED: return "UNINITIALIZED";
        case HAL_MODULE_STATE_INITIALIZED: return "INITIALIZED";
        case HAL_MODULE_STATE_RUNNING: return "RUNNING";
        case HAL_MODULE_STATE_ERROR: return "ERROR";
        case HAL_MODULE_STATE_SUSPENDED: return "SUSPENDED";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 获取错误名称
 * @param error 错误代码
 * @return const char* 错误名称
 */
const char* hal_manager_get_error_name(hal_result_t error)
{
    switch (error) {
        case HAL_OK: return "OK";
        case HAL_ERROR: return "ERROR";
        case HAL_BUSY: return "BUSY";
        case HAL_TIMEOUT: return "TIMEOUT";
        case HAL_INVALID_PARAM: return "INVALID_PARAM";
        case HAL_NOT_INITIALIZED: return "NOT_INITIALIZED";
        case HAL_NOT_SUPPORTED: return "NOT_SUPPORTED";
        case HAL_RESOURCE_LOCKED: return "RESOURCE_LOCKED";
        case HAL_BUFFER_FULL: return "BUFFER_FULL";
        case HAL_BUFFER_EMPTY: return "BUFFER_EMPTY";
        case HAL_CRC_ERROR: return "CRC_ERROR";
        case HAL_DMA_ERROR: return "DMA_ERROR";
        case HAL_HARDWARE_ERROR: return "HARDWARE_ERROR";
        default: return "UNKNOWN";
    }
}