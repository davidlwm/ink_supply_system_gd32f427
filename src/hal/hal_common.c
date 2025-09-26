/**
 * @file hal_common.c
 * @brief HAL层通用功能实现
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统HAL层通用功能实现
 */

#include "hal/hal_common.h"
#include <string.h>

/* 私有变量 */
static bool g_hal_system_initialized = false;
static uint32_t g_system_clock_freq = 0;

/**
 * @brief 获取HAL层版本信息
 * @param major 主版本号指针
 * @param minor 次版本号指针
 * @param patch 补丁版本号指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_get_version(uint32_t* major, uint32_t* minor, uint32_t* patch)
{
    HAL_CHECK_PARAM(major);
    HAL_CHECK_PARAM(minor);
    HAL_CHECK_PARAM(patch);

    *major = HAL_VERSION_MAJOR;
    *minor = HAL_VERSION_MINOR;
    *patch = HAL_VERSION_PATCH;

    return HAL_OK;
}

/**
 * @brief HAL系统初始化
 * @return hal_result_t 操作结果
 */
hal_result_t hal_system_init(void)
{
    if (g_hal_system_initialized) {
        return HAL_OK;
    }

    /* 初始化系统时钟 */
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_BKPSRAM);

    /* 获取系统时钟频率 */
    g_system_clock_freq = rcu_clock_freq_get(CK_SYS);

    /* 初始化SysTick */
    systick_clksource_set(SYSTICK_CLKSOURCE_HCLK);

    g_hal_system_initialized = true;
    return HAL_OK;
}

/**
 * @brief HAL系统反初始化
 * @return hal_result_t 操作结果
 */
hal_result_t hal_system_deinit(void)
{
    if (!g_hal_system_initialized) {
        return HAL_OK;
    }

    g_hal_system_initialized = false;
    g_system_clock_freq = 0;

    return HAL_OK;
}

/**
 * @brief 进入电源模式
 * @param mode 电源模式
 * @return hal_result_t 操作结果
 */
hal_result_t hal_enter_power_mode(hal_power_mode_t mode)
{
    switch (mode) {
        case HAL_POWER_MODE_RUN:
            /* 已经在运行模式 */
            break;

        case HAL_POWER_MODE_SLEEP:
            pmu_to_sleepmode(WFI_CMD);
            break;

        case HAL_POWER_MODE_STOP:
            pmu_to_deepsleepmode(PMU_LDO_NORMAL, PMU_LOWDRIVER_DISABLE, WFI_CMD);
            break;

        case HAL_POWER_MODE_STANDBY:
            pmu_to_standbymode();
            break;

        default:
            return HAL_INVALID_PARAM;
    }

    return HAL_OK;
}

/**
 * @brief 退出电源模式
 * @return hal_result_t 操作结果
 */
hal_result_t hal_exit_power_mode(void)
{
    /* 系统会自动从低功耗模式唤醒到运行模式 */
    return HAL_OK;
}

/**
 * @brief 获取系统时钟频率
 * @param clock_freq 时钟频率指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_get_system_clock(uint32_t* clock_freq)
{
    HAL_CHECK_PARAM(clock_freq);

    if (!g_hal_system_initialized) {
        return HAL_NOT_INITIALIZED;
    }

    *clock_freq = g_system_clock_freq;
    return HAL_OK;
}

/**
 * @brief 毫秒延时
 * @param ms 延时时间(毫秒)
 * @return hal_result_t 操作结果
 */
hal_result_t hal_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        /* 调度器未运行时使用简单延时 */
        uint32_t cycles = (g_system_clock_freq / 1000) * ms;
        for (volatile uint32_t i = 0; i < cycles / 10; i++) {
            __NOP();
        }
    }

    return HAL_OK;
}

/**
 * @brief 微秒延时
 * @param us 延时时间(微秒)
 * @return hal_result_t 操作结果
 */
hal_result_t hal_delay_us(uint32_t us)
{
    uint32_t cycles = (g_system_clock_freq / 1000000) * us;
    for (volatile uint32_t i = 0; i < cycles / 10; i++) {
        __NOP();
    }

    return HAL_OK;
}

/**
 * @brief 初始化HAL句柄
 * @param handle HAL句柄指针
 * @param config 配置指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_handle_init(hal_handle_base_t* handle, const hal_config_base_t* config)
{
    HAL_CHECK_PARAM(handle);
    HAL_CHECK_PARAM(config);

    /* 检查配置有效性 */
    if (config->magic != HAL_CONFIG_MAGIC) {
        return HAL_INVALID_PARAM;
    }

    /* 初始化句柄 */
    memset(handle, 0, sizeof(hal_handle_base_t));
    handle->config = (hal_config_base_t*)config;
    handle->state = HAL_STATE_RESET;

    /* 创建互斥锁 */
    handle->mutex = xSemaphoreCreateMutex();
    if (handle->mutex == NULL) {
        return HAL_ERROR;
    }

    /* 初始化统计信息 */
    memset(&handle->stats, 0, sizeof(hal_statistics_t));
    handle->stats.init_count = 1;

    handle->initialized = true;
    handle->state = HAL_STATE_READY;

    /* 调用初始化回调 */
    if (config->init_callback != NULL) {
        config->init_callback(config->user_context);
    }

    return HAL_OK;
}

/**
 * @brief 反初始化HAL句柄
 * @param handle HAL句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_handle_deinit(hal_handle_base_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    /* 删除互斥锁 */
    if (handle->mutex != NULL) {
        vSemaphoreDelete(handle->mutex);
        handle->mutex = NULL;
    }

    /* 清除句柄 */
    memset(handle, 0, sizeof(hal_handle_base_t));

    return HAL_OK;
}

/**
 * @brief 获取统计信息
 * @param handle HAL句柄指针
 * @param stats 统计信息指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_get_statistics(hal_handle_base_t* handle, hal_statistics_t* stats)
{
    HAL_CHECK_HANDLE(handle);
    HAL_CHECK_PARAM(stats);

    HAL_LOCK(handle);
    memcpy(stats, &handle->stats, sizeof(hal_statistics_t));
    HAL_UNLOCK(handle);

    return HAL_OK;
}

/**
 * @brief 清除统计信息
 * @param handle HAL句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t hal_clear_statistics(hal_handle_base_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    HAL_LOCK(handle);
    memset(&handle->stats, 0, sizeof(hal_statistics_t));
    HAL_UNLOCK(handle);

    return HAL_OK;
}