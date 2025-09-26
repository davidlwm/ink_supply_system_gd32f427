/**
 * @file middleware_common.c
 * @brief 中间件层通用实现 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 中间件层通用功能实现
 */

#include "middleware/middleware_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/* 私有变量 */
static mw_module_info_t g_modules[MW_MODULE_COUNT];
static bool g_middleware_initialized = false;
static SemaphoreHandle_t g_middleware_mutex = NULL;

/* 私有函数声明 */
static uint32_t get_system_tick_us(void);
static void update_module_statistics(mw_module_type_t type, uint32_t process_time_us);

/**
 * @brief 中间件系统初始化
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_system_init(void)
{
    if (g_middleware_initialized) {
        return MW_ERROR_ALREADY_INITIALIZED;
    }

    /* 创建互斥锁 */
    g_middleware_mutex = xSemaphoreCreateMutex();
    if (g_middleware_mutex == NULL) {
        return MW_ERROR_OUT_OF_MEMORY;
    }

    /* 初始化模块信息 */
    memset(g_modules, 0, sizeof(g_modules));

    for (int i = 0; i < MW_MODULE_COUNT; i++) {
        g_modules[i].type = (mw_module_type_t)i;
        g_modules[i].state = MW_STATE_UNINITIALIZED;
        g_modules[i].init_time = 0;
        memset(&g_modules[i].stats, 0, sizeof(mw_statistics_t));
        g_modules[i].stats.min_process_time_us = UINT32_MAX;
    }

    g_middleware_initialized = true;
    return MW_OK;
}

/**
 * @brief 中间件系统反初始化
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_system_deinit(void)
{
    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    /* 删除互斥锁 */
    if (g_middleware_mutex != NULL) {
        vSemaphoreDelete(g_middleware_mutex);
        g_middleware_mutex = NULL;
    }

    /* 清除所有模块状态 */
    memset(g_modules, 0, sizeof(g_modules));

    g_middleware_initialized = false;
    return MW_OK;
}

/**
 * @brief 注册中间件模块
 * @param type 模块类型
 * @param name 模块名称
 * @param version 模块版本
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_register_module(mw_module_type_t type, const char* name, const char* version)
{
    MW_CHECK_PARAM(type < MW_MODULE_COUNT);
    MW_CHECK_NULL(name);
    MW_CHECK_NULL(version);

    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    if (xSemaphoreTake(g_middleware_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return MW_ERROR_TIMEOUT;
    }

    g_modules[type].name = name;
    g_modules[type].version = version;
    g_modules[type].state = MW_STATE_READY;
    g_modules[type].init_time = get_system_tick_us();

    xSemaphoreGive(g_middleware_mutex);
    return MW_OK;
}

/**
 * @brief 注销中间件模块
 * @param type 模块类型
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_unregister_module(mw_module_type_t type)
{
    MW_CHECK_PARAM(type < MW_MODULE_COUNT);

    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    if (xSemaphoreTake(g_middleware_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return MW_ERROR_TIMEOUT;
    }

    g_modules[type].name = NULL;
    g_modules[type].version = NULL;
    g_modules[type].state = MW_STATE_UNINITIALIZED;
    memset(&g_modules[type].stats, 0, sizeof(mw_statistics_t));

    xSemaphoreGive(g_middleware_mutex);
    return MW_OK;
}

/**
 * @brief 设置模块状态
 * @param type 模块类型
 * @param state 状态
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_set_module_state(mw_module_type_t type, mw_state_t state)
{
    MW_CHECK_PARAM(type < MW_MODULE_COUNT);

    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    if (xSemaphoreTake(g_middleware_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return MW_ERROR_TIMEOUT;
    }

    g_modules[type].state = state;

    xSemaphoreGive(g_middleware_mutex);
    return MW_OK;
}

/**
 * @brief 获取模块状态
 * @param type 模块类型
 * @return mw_state_t 模块状态
 */
mw_state_t middleware_get_module_state(mw_module_type_t type)
{
    if (type >= MW_MODULE_COUNT || !g_middleware_initialized) {
        return MW_STATE_UNINITIALIZED;
    }

    return g_modules[type].state;
}

/**
 * @brief 获取模块信息
 * @param type 模块类型
 * @param info 模块信息结构体指针
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_get_module_info(mw_module_type_t type, mw_module_info_t* info)
{
    MW_CHECK_PARAM(type < MW_MODULE_COUNT);
    MW_CHECK_NULL(info);

    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    if (xSemaphoreTake(g_middleware_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return MW_ERROR_TIMEOUT;
    }

    memcpy(info, &g_modules[type], sizeof(mw_module_info_t));

    xSemaphoreGive(g_middleware_mutex);
    return MW_OK;
}

/**
 * @brief 更新统计信息
 * @param type 模块类型
 * @param process_time_us 处理时间(微秒)
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_update_statistics(mw_module_type_t type, uint32_t process_time_us)
{
    MW_CHECK_PARAM(type < MW_MODULE_COUNT);

    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    update_module_statistics(type, process_time_us);
    return MW_OK;
}

/**
 * @brief 重置统计信息
 * @param type 模块类型
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_reset_statistics(mw_module_type_t type)
{
    MW_CHECK_PARAM(type < MW_MODULE_COUNT);

    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    if (xSemaphoreTake(g_middleware_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return MW_ERROR_TIMEOUT;
    }

    memset(&g_modules[type].stats, 0, sizeof(mw_statistics_t));
    g_modules[type].stats.min_process_time_us = UINT32_MAX;

    xSemaphoreGive(g_middleware_mutex);
    return MW_OK;
}

/**
 * @brief 获取系统总统计信息
 * @param total_stats 总统计信息指针
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_get_system_statistics(mw_statistics_t* total_stats)
{
    MW_CHECK_NULL(total_stats);

    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    if (xSemaphoreTake(g_middleware_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return MW_ERROR_TIMEOUT;
    }

    memset(total_stats, 0, sizeof(mw_statistics_t));
    total_stats->min_process_time_us = UINT32_MAX;

    /* 汇总所有模块的统计信息 */
    for (int i = 0; i < MW_MODULE_COUNT; i++) {
        total_stats->process_count += g_modules[i].stats.process_count;
        total_stats->error_count += g_modules[i].stats.error_count;

        if (g_modules[i].stats.max_process_time_us > total_stats->max_process_time_us) {
            total_stats->max_process_time_us = g_modules[i].stats.max_process_time_us;
        }

        if (g_modules[i].stats.min_process_time_us < total_stats->min_process_time_us) {
            total_stats->min_process_time_us = g_modules[i].stats.min_process_time_us;
        }
    }

    /* 计算平均处理时间 */
    if (total_stats->process_count > 0) {
        uint64_t total_time = 0;
        for (int i = 0; i < MW_MODULE_COUNT; i++) {
            total_time += (uint64_t)g_modules[i].stats.process_count * g_modules[i].stats.avg_process_time_us;
        }
        total_stats->avg_process_time_us = (float)total_time / total_stats->process_count;
    }

    xSemaphoreGive(g_middleware_mutex);
    return MW_OK;
}

/**
 * @brief 获取时间戳
 * @return mw_timestamp_t 时间戳
 */
mw_timestamp_t middleware_get_timestamp(void)
{
    mw_timestamp_t timestamp;
    uint32_t tick_us = get_system_tick_us();

    timestamp.seconds = tick_us / 1000000;
    timestamp.microseconds = tick_us % 1000000;

    return timestamp;
}

/**
 * @brief 计算时间戳差值
 * @param start 起始时间戳
 * @param end 结束时间戳
 * @return uint32_t 时间差(微秒)
 */
uint32_t middleware_timestamp_diff_us(mw_timestamp_t start, mw_timestamp_t end)
{
    uint32_t start_us = start.seconds * 1000000 + start.microseconds;
    uint32_t end_us = end.seconds * 1000000 + end.microseconds;

    return (end_us >= start_us) ? (end_us - start_us) : (UINT32_MAX - start_us + end_us);
}

/**
 * @brief 获取系统时钟(微秒)
 * @return uint32_t 系统时钟(微秒)
 */
uint32_t middleware_get_tick_us(void)
{
    return get_system_tick_us();
}

/**
 * @brief 中间件系统自检
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_self_test(void)
{
    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    /* 检查互斥锁 */
    if (g_middleware_mutex == NULL) {
        return MW_ERROR;
    }

    /* 检查每个模块的状态 */
    for (int i = 0; i < MW_MODULE_COUNT; i++) {
        if (g_modules[i].state == MW_STATE_ERROR) {
            return MW_ERROR;
        }
    }

    return MW_OK;
}

/**
 * @brief 检查系统是否健康
 * @return bool 健康状态
 */
bool middleware_is_system_healthy(void)
{
    return (middleware_self_test() == MW_OK);
}

/**
 * @brief 输出系统状态
 * @return mw_result_t 操作结果
 */
mw_result_t middleware_dump_status(void)
{
    if (!g_middleware_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    /* 这里可以添加调试输出代码 */
    /* 例如：通过UART输出各模块状态 */

    return MW_OK;
}

/* 私有函数实现 */

/**
 * @brief 获取系统时钟(微秒)
 * @return uint32_t 系统时钟(微秒)
 */
static uint32_t get_system_tick_us(void)
{
    /* 假设系统时钟为1kHz (1ms per tick) */
    return xTaskGetTickCount() * 1000;
}

/**
 * @brief 更新模块统计信息
 * @param type 模块类型
 * @param process_time_us 处理时间(微秒)
 */
static void update_module_statistics(mw_module_type_t type, uint32_t process_time_us)
{
    if (type >= MW_MODULE_COUNT) {
        return;
    }

    if (xSemaphoreTake(g_middleware_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    mw_statistics_t* stats = &g_modules[type].stats;

    stats->process_count++;
    stats->last_process_time_us = process_time_us;

    /* 更新最大处理时间 */
    if (process_time_us > stats->max_process_time_us) {
        stats->max_process_time_us = process_time_us;
    }

    /* 更新最小处理时间 */
    if (process_time_us < stats->min_process_time_us) {
        stats->min_process_time_us = process_time_us;
    }

    /* 更新平均处理时间 */
    if (stats->process_count == 1) {
        stats->avg_process_time_us = (float)process_time_us;
    } else {
        stats->avg_process_time_us = (stats->avg_process_time_us * (stats->process_count - 1) + process_time_us) / stats->process_count;
    }

    xSemaphoreGive(g_middleware_mutex);
}