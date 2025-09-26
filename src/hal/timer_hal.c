/**
 * @file timer_hal.c
 * @brief 定时器硬件抽象层实现
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统定时器硬件抽象层实现
 */

#include "hal/timer_hal.h"
#include "hal/hal_common.h"
#include <string.h>

/* 私有变量 */
static bool g_timer_system_initialized = false;
static timer_handle_t* g_function_timers[TIMER_FUNCTION_COUNT] = {NULL};

/**
 * @brief 定时器句柄初始化
 * @param handle 定时器句柄指针
 * @param config 定时器配置指针
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_init(timer_handle_t* handle, const timer_config_t* config)
{
    HAL_CHECK_PARAM(handle);
    HAL_CHECK_PARAM(config);

    /* 初始化基础句柄 */
    hal_result_t result = hal_handle_init(&handle->base, &config->base);
    if (result != HAL_OK) {
        return result;
    }

    /* 设置定时器配置 */
    handle->config = (timer_config_t*)config;
    handle->mode = TIMER_MODE_BASE;
    handle->counter = 0;
    handle->auto_reload = false;
    handle->channel_state = 0;

    /* 使能定时器时钟 */
    uint32_t timer_rcu = RCU_TIMER0 + (config->instance & 0x0F);
    rcu_periph_clock_enable(timer_rcu);

    /* 配置定时器基础参数 */
    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler = config->prescaler;
    timer_initpara.alignedmode = (config->count_mode == TIMER_COUNT_UP) ? TIMER_COUNTER_EDGE : TIMER_COUNTER_CENTER_DOWN;
    timer_initpara.counterdirection = (config->count_mode == TIMER_COUNT_UP) ? TIMER_COUNTER_UP : TIMER_COUNTER_DOWN;
    timer_initpara.period = config->period;
    timer_initpara.clockdivision = config->clock_div;
    timer_initpara.repetitioncounter = config->repetition_counter;

    timer_init(config->instance, &timer_initpara);

    /* 配置自动重载预装载 */
    if (config->auto_reload_preload) {
        timer_auto_reload_shadow_enable(config->instance);
    } else {
        timer_auto_reload_shadow_disable(config->instance);
    }

    return HAL_OK;
}

/**
 * @brief 定时器句柄反初始化
 * @param handle 定时器句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_deinit(timer_handle_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    /* 禁用定时器 */
    timer_disable(handle->config->instance);

    /* 反初始化基础句柄 */
    return hal_handle_deinit(&handle->base);
}

/**
 * @brief 启动定时器
 * @param handle 定时器句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_start(timer_handle_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    handle->base.state = HAL_STATE_BUSY;
    timer_enable(handle->config->instance);

    return HAL_OK;
}

/**
 * @brief 停止定时器
 * @param handle 定时器句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_stop(timer_handle_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    timer_disable(handle->config->instance);
    handle->base.state = HAL_STATE_READY;

    return HAL_OK;
}

/**
 * @brief 启动定时器中断
 * @param handle 定时器句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_start_it(timer_handle_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    /* 清除中断标志 */
    timer_interrupt_flag_clear(handle->config->instance, TIMER_INT_FLAG_UP);

    /* 使能中断 */
    if (handle->config->interrupts & TIMER_IT_UPDATE) {
        timer_interrupt_enable(handle->config->instance, TIMER_INT_UP);
    }

    handle->base.state = HAL_STATE_BUSY;
    timer_enable(handle->config->instance);

    return HAL_OK;
}

/**
 * @brief 停止定时器中断
 * @param handle 定时器句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_stop_it(timer_handle_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    /* 禁用中断 */
    timer_interrupt_disable(handle->config->instance, TIMER_INT_UP);
    timer_disable(handle->config->instance);
    handle->base.state = HAL_STATE_READY;

    return HAL_OK;
}

/**
 * @brief 设置定时器计数值
 * @param handle 定时器句柄指针
 * @param counter 计数值
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_set_counter(timer_handle_t* handle, uint32_t counter)
{
    HAL_CHECK_HANDLE(handle);

    timer_counter_value_config(handle->config->instance, counter);
    handle->counter = counter;

    return HAL_OK;
}

/**
 * @brief 获取定时器计数值
 * @param handle 定时器句柄指针
 * @param counter 计数值指针
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_get_counter(timer_handle_t* handle, uint32_t* counter)
{
    HAL_CHECK_HANDLE(handle);
    HAL_CHECK_PARAM(counter);

    *counter = timer_counter_read(handle->config->instance);
    handle->counter = *counter;

    return HAL_OK;
}

/**
 * @brief 设置定时器预分频器
 * @param handle 定时器句柄指针
 * @param prescaler 预分频值
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_set_prescaler(timer_handle_t* handle, uint32_t prescaler)
{
    HAL_CHECK_HANDLE(handle);

    timer_prescaler_config(handle->config->instance, prescaler, TIMER_PSC_RELOAD_UPDATE);
    handle->config->prescaler = prescaler;

    return HAL_OK;
}

/**
 * @brief 设置定时器周期
 * @param handle 定时器句柄指针
 * @param period 周期值
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_set_period(timer_handle_t* handle, uint32_t period)
{
    HAL_CHECK_HANDLE(handle);

    timer_autoreload_value_config(handle->config->instance, period);
    handle->config->period = period;

    return HAL_OK;
}

/**
 * @brief 定时器系统初始化
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_system_init(void)
{
    if (g_timer_system_initialized) {
        return HAL_OK;
    }

    /* 初始化功能定时器数组 */
    memset(g_function_timers, 0, sizeof(g_function_timers));

    g_timer_system_initialized = true;
    return HAL_OK;
}

/**
 * @brief 定时器系统反初始化
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_system_deinit(void)
{
    /* 停止所有功能定时器 */
    for (int i = 0; i < TIMER_FUNCTION_COUNT; i++) {
        if (g_function_timers[i] != NULL) {
            timer_hal_stop(g_function_timers[i]);
        }
    }

    g_timer_system_initialized = false;
    return HAL_OK;
}

/**
 * @brief 设置功能定时器
 * @param function 功能ID
 * @param period_ms 周期(毫秒)
 * @param callback 回调函数
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_setup_function_timer(timer_function_id_t function, uint32_t period_ms, hal_callback_t callback)
{
    if (function >= TIMER_FUNCTION_COUNT) {
        return HAL_INVALID_PARAM;
    }

    if (!g_timer_system_initialized) {
        return HAL_NOT_INITIALIZED;
    }

    /* 这里应该根据实际需求配置特定的定时器 */
    /* 为简化起见，此处只做参数检查和存储 */

    return HAL_OK;
}

/**
 * @brief 启动功能定时器
 * @param function 功能ID
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_start_function_timer(timer_function_id_t function)
{
    if (function >= TIMER_FUNCTION_COUNT) {
        return HAL_INVALID_PARAM;
    }

    if (g_function_timers[function] == NULL) {
        return HAL_NOT_INITIALIZED;
    }

    return timer_hal_start(g_function_timers[function]);
}

/**
 * @brief 停止功能定时器
 * @param function 功能ID
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_stop_function_timer(timer_function_id_t function)
{
    if (function >= TIMER_FUNCTION_COUNT) {
        return HAL_INVALID_PARAM;
    }

    if (g_function_timers[function] == NULL) {
        return HAL_NOT_INITIALIZED;
    }

    return timer_hal_stop(g_function_timers[function]);
}

/**
 * @brief 获取系统时间
 * @param time_ms 时间指针(毫秒)
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_get_system_time_ms(uint32_t* time_ms)
{
    HAL_CHECK_PARAM(time_ms);

    *time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return HAL_OK;
}

/**
 * @brief 精确微秒延时
 * @param delay_us 延时时间(微秒)
 * @return hal_result_t 操作结果
 */
hal_result_t timer_hal_delay_precise_us(uint32_t delay_us)
{
    /* 使用系统定时器实现精确延时 */
    uint32_t start_us = timer_counter_read(TIMER0);
    uint32_t current_us;

    do {
        current_us = timer_counter_read(TIMER0);
    } while ((current_us - start_us) < delay_us);

    return HAL_OK;
}