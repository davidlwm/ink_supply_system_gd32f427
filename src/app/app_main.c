/**
 * @file app_main.c
 * @brief 应用层主入口实现 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统应用层统一入口实现
 */

#include "app/app_main.h"
#include "app/safety_task.h"
#include "app/control_task.h"
#include "app/sensor_task.h"
#include "app/actuator_task.h"
#include "app/hmi_task.h"
#include "app/comm_task.h"
#include "app/config_task.h"

/* 中间件支持 */
#include "middleware/middleware_common.h"
#include "middleware/tasks.h"
#include "middleware/filter.h"
#include "middleware/pid.h"

/* HAL层支持 */
#include "hal/gpio_hal.h"
#include "hal/uart_hal.h"

#include <string.h>

/* 私有变量 */
static app_task_handles_t g_task_handles;
static app_system_status_t g_system_status;
static app_config_t g_app_config;
static app_state_t g_app_state = APP_STATE_STOPPED;
static SemaphoreHandle_t g_state_mutex = NULL;
static TimerHandle_t g_statistics_timer = NULL;
static app_error_callback_t g_error_callback = NULL;
static app_state_change_callback_t g_state_change_callback = NULL;
static bool g_app_initialized = false;

/* 私有函数声明 */
static app_result_t app_create_all_tasks(void);
static app_result_t app_start_all_tasks(void);
static app_result_t app_stop_all_tasks(void);
static app_result_t app_delete_all_tasks(void);
static void app_statistics_timer_callback(TimerHandle_t timer);
static app_result_t app_set_state(app_state_t new_state);
static void app_calculate_cpu_usage(void);

/**
 * @brief 应用层初始化
 * @param config 应用配置参数
 * @return app_result_t 操作结果
 */
app_result_t app_main_init(const app_config_t* config)
{
    if (config == NULL) {
        return APP_INVALID_PARAM;
    }

    if (g_app_initialized) {
        return APP_OK;
    }

    /* 复制配置 */
    memcpy(&g_app_config, config, sizeof(app_config_t));

    /* 创建状态互斥锁 */
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        return APP_RESOURCE_EXHAUSTED;
    }

    /* 初始化系统状态 */
    memset(&g_system_status, 0, sizeof(app_system_status_t));
    g_system_status.state = APP_STATE_STOPPED;
    g_system_status.free_heap_size = xPortGetFreeHeapSize();
    g_system_status.min_free_heap_size = g_system_status.free_heap_size;

    /* 初始化任务句柄 */
    memset(&g_task_handles, 0, sizeof(app_task_handles_t));

    /* 初始化中间件系统 */
    mw_result_t mw_result = middleware_system_init();
    if (mw_result != MW_OK) {
        return APP_ERROR;
    }

    /* 初始化各个中间件模块 */
    mw_result = filter_system_init();
    if (mw_result != MW_OK) {
        return APP_ERROR;
    }

    mw_result = pid_system_init();
    if (mw_result != MW_OK) {
        return APP_ERROR;
    }

    /* 初始化任务管理器 */
    mw_result = task_manager_init();
    if (mw_result != MW_OK) {
        return APP_ERROR;
    }

    /* 创建统计定时器 */
    if (g_app_config.statistics_period_ms > 0) {
        g_statistics_timer = xTimerCreate("AppStats",
                                        pdMS_TO_TICKS(g_app_config.statistics_period_ms),
                                        pdTRUE,
                                        NULL,
                                        app_statistics_timer_callback);
        if (g_statistics_timer == NULL) {
            vSemaphoreDelete(g_state_mutex);
            return APP_RESOURCE_EXHAUSTED;
        }
    }

    /* 初始化HAL层 */
    gpio_result_t gpio_result = gpio_hal_init();
    if (gpio_result != GPIO_OK) {
        return APP_ERROR;
    }

    uart_result_t uart_result = uart_hal_init();
    if (uart_result != UART_OK) {
        return APP_ERROR;
    }

    g_app_initialized = true;
    return APP_OK;
}

/**
 * @brief 启动应用系统
 * @return app_result_t 操作结果
 */
app_result_t app_main_start(void)
{
    if (!g_app_initialized) {
        return APP_NOT_INITIALIZED;
    }

    if (g_app_state != APP_STATE_STOPPED) {
        return APP_BUSY;
    }

    app_set_state(APP_STATE_STARTING);

    /* 创建所有任务 */
    app_result_t result = app_create_all_tasks();
    if (result != APP_OK) {
        app_set_state(APP_STATE_ERROR);
        return result;
    }

    /* 启动所有任务 */
    result = app_start_all_tasks();
    if (result != APP_OK) {
        app_delete_all_tasks();
        app_set_state(APP_STATE_ERROR);
        return result;
    }

    /* 启动统计定时器 */
    if (g_statistics_timer != NULL) {
        xTimerStart(g_statistics_timer, portMAX_DELAY);
    }

    app_set_state(APP_STATE_RUNNING);
    return APP_OK;
}

/**
 * @brief 停止应用系统
 * @return app_result_t 操作结果
 */
app_result_t app_main_stop(void)
{
    if (g_app_state != APP_STATE_RUNNING) {
        return APP_BUSY;
    }

    app_set_state(APP_STATE_STOPPING);

    /* 停止统计定时器 */
    if (g_statistics_timer != NULL) {
        xTimerStop(g_statistics_timer, portMAX_DELAY);
    }

    /* 停止所有任务 */
    app_result_t result = app_stop_all_tasks();
    if (result != APP_OK) {
        app_set_state(APP_STATE_ERROR);
        return result;
    }

    /* 删除所有任务 */
    result = app_delete_all_tasks();
    if (result != APP_OK) {
        app_set_state(APP_STATE_ERROR);
        return result;
    }

    app_set_state(APP_STATE_STOPPED);
    return APP_OK;
}

/**
 * @brief 获取系统状态
 * @param status 状态结构体指针
 * @return app_result_t 操作结果
 */
app_result_t app_get_system_status(app_system_status_t* status)
{
    if (status == NULL) {
        return APP_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return APP_TIMEOUT;
    }

    /* 更新实时状态 */
    g_system_status.free_heap_size = xPortGetFreeHeapSize();
    if (g_system_status.free_heap_size < g_system_status.min_free_heap_size) {
        g_system_status.min_free_heap_size = g_system_status.free_heap_size;
    }

    memcpy(status, &g_system_status, sizeof(app_system_status_t));
    xSemaphoreGive(g_state_mutex);
    return APP_OK;
}

/**
 * @brief 获取当前状态
 * @return app_state_t 当前状态
 */
app_state_t app_get_state(void)
{
    return g_app_state;
}

/**
 * @brief 错误处理
 * @param task_name 任务名称
 * @param error_code 错误代码
 * @return app_result_t 操作结果
 */
app_result_t app_handle_error(const char* task_name, uint32_t error_code)
{
    if (task_name == NULL) {
        return APP_INVALID_PARAM;
    }

    /* 增加错误计数 */
    g_system_status.error_count++;

    /* 调用错误回调 */
    if (g_error_callback != NULL) {
        g_error_callback(task_name, error_code);
    }

    /* 记录错误到任务管理器 */
    task_error_report(task_name, error_code);

    return APP_OK;
}

/* 私有函数实现 */

/**
 * @brief 创建所有应用任务
 * @return app_result_t 操作结果
 */
static app_result_t app_create_all_tasks(void)
{
    BaseType_t result;

    /* 创建安全监控任务 (最高优先级) */
    result = xTaskCreate(safety_task, "SafetyTask",
                        APP_STACK_SIZE_MEDIUM, NULL,
                        APP_PRIORITY_CRITICAL, &g_task_handles.safety_task);
    if (result != pdPASS) {
        return APP_TASK_CREATE_FAILED;
    }

    /* 创建控制任务 */
    result = xTaskCreate(control_task, "ControlTask",
                        APP_STACK_SIZE_LARGE, NULL,
                        APP_PRIORITY_HIGH, &g_task_handles.control_task);
    if (result != pdPASS) {
        return APP_TASK_CREATE_FAILED;
    }

    /* 创建传感器任务 */
    result = xTaskCreate(sensor_task, "SensorTask",
                        APP_STACK_SIZE_MEDIUM, NULL,
                        APP_PRIORITY_NORMAL, &g_task_handles.sensor_task);
    if (result != pdPASS) {
        return APP_TASK_CREATE_FAILED;
    }

    /* 创建执行器任务 */
    result = xTaskCreate(actuator_task, "ActuatorTask",
                        APP_STACK_SIZE_MEDIUM, NULL,
                        APP_PRIORITY_NORMAL, &g_task_handles.actuator_task);
    if (result != pdPASS) {
        return APP_TASK_CREATE_FAILED;
    }

    /* 创建HMI任务 */
    result = xTaskCreate(hmi_task, "HMITask",
                        APP_STACK_SIZE_MEDIUM, NULL,
                        APP_PRIORITY_LOW, &g_task_handles.hmi_task);
    if (result != pdPASS) {
        return APP_TASK_CREATE_FAILED;
    }

    /* 创建通信任务 */
    result = xTaskCreate(comm_task, "CommTask",
                        APP_STACK_SIZE_MEDIUM, NULL,
                        APP_PRIORITY_LOW, &g_task_handles.comm_task);
    if (result != pdPASS) {
        return APP_TASK_CREATE_FAILED;
    }

    /* 创建配置管理任务 */
    result = xTaskCreate(config_task, "ConfigTask",
                        APP_STACK_SIZE_SMALL, NULL,
                        APP_PRIORITY_BACKGROUND, &g_task_handles.config_task);
    if (result != pdPASS) {
        return APP_TASK_CREATE_FAILED;
    }

    /* 向任务管理器注册所有任务 */
    task_config_t task_config;

    task_config.priority = APP_PRIORITY_CRITICAL;
    task_config.priority_class = TASK_PRIORITY_CRITICAL;
    task_config.stack_size = APP_STACK_SIZE_MEDIUM;
    task_config.watchdog_enabled = true;
    task_config.watchdog_timeout_ms = 100;
    task_config.performance_monitoring = true;
    task_register("SafetyTask", g_task_handles.safety_task, &task_config);

    task_config.priority = APP_PRIORITY_HIGH;
    task_config.priority_class = TASK_PRIORITY_HIGH;
    task_config.stack_size = APP_STACK_SIZE_LARGE;
    task_config.watchdog_timeout_ms = 200;
    task_register("ControlTask", g_task_handles.control_task, &task_config);

    task_config.priority = APP_PRIORITY_NORMAL;
    task_config.priority_class = TASK_PRIORITY_NORMAL;
    task_config.stack_size = APP_STACK_SIZE_MEDIUM;
    task_config.watchdog_timeout_ms = 500;
    task_register("SensorTask", g_task_handles.sensor_task, &task_config);
    task_register("ActuatorTask", g_task_handles.actuator_task, &task_config);

    task_config.priority = APP_PRIORITY_LOW;
    task_config.priority_class = TASK_PRIORITY_LOW;
    task_config.watchdog_timeout_ms = 1000;
    task_register("HMITask", g_task_handles.hmi_task, &task_config);
    task_register("CommTask", g_task_handles.comm_task, &task_config);

    task_config.priority = APP_PRIORITY_BACKGROUND;
    task_config.priority_class = TASK_PRIORITY_LOW;
    task_config.watchdog_timeout_ms = 5000;
    task_register("ConfigTask", g_task_handles.config_task, &task_config);

    return APP_OK;
}

/**
 * @brief 启动所有任务
 * @return app_result_t 操作结果
 */
static app_result_t app_start_all_tasks(void)
{
    /* FreeRTOS任务创建后自动开始运行，这里可以添加额外的启动逻辑 */
    return APP_OK;
}

/**
 * @brief 停止所有任务
 * @return app_result_t 操作结果
 */
static app_result_t app_stop_all_tasks(void)
{
    /* 挂起所有任务 */
    if (g_task_handles.safety_task != NULL) {
        vTaskSuspend(g_task_handles.safety_task);
    }
    if (g_task_handles.control_task != NULL) {
        vTaskSuspend(g_task_handles.control_task);
    }
    if (g_task_handles.sensor_task != NULL) {
        vTaskSuspend(g_task_handles.sensor_task);
    }
    if (g_task_handles.actuator_task != NULL) {
        vTaskSuspend(g_task_handles.actuator_task);
    }
    if (g_task_handles.hmi_task != NULL) {
        vTaskSuspend(g_task_handles.hmi_task);
    }
    if (g_task_handles.comm_task != NULL) {
        vTaskSuspend(g_task_handles.comm_task);
    }
    if (g_task_handles.config_task != NULL) {
        vTaskSuspend(g_task_handles.config_task);
    }

    return APP_OK;
}

/**
 * @brief 删除所有任务
 * @return app_result_t 操作结果
 */
static app_result_t app_delete_all_tasks(void)
{
    /* 从任务管理器注销所有任务 */
    task_unregister("SafetyTask");
    task_unregister("ControlTask");
    task_unregister("SensorTask");
    task_unregister("ActuatorTask");
    task_unregister("HMITask");
    task_unregister("CommTask");
    task_unregister("ConfigTask");

    /* 删除所有任务 */
    if (g_task_handles.safety_task != NULL) {
        vTaskDelete(g_task_handles.safety_task);
        g_task_handles.safety_task = NULL;
    }
    if (g_task_handles.control_task != NULL) {
        vTaskDelete(g_task_handles.control_task);
        g_task_handles.control_task = NULL;
    }
    if (g_task_handles.sensor_task != NULL) {
        vTaskDelete(g_task_handles.sensor_task);
        g_task_handles.sensor_task = NULL;
    }
    if (g_task_handles.actuator_task != NULL) {
        vTaskDelete(g_task_handles.actuator_task);
        g_task_handles.actuator_task = NULL;
    }
    if (g_task_handles.hmi_task != NULL) {
        vTaskDelete(g_task_handles.hmi_task);
        g_task_handles.hmi_task = NULL;
    }
    if (g_task_handles.comm_task != NULL) {
        vTaskDelete(g_task_handles.comm_task);
        g_task_handles.comm_task = NULL;
    }
    if (g_task_handles.config_task != NULL) {
        vTaskDelete(g_task_handles.config_task);
        g_task_handles.config_task = NULL;
    }

    return APP_OK;
}

/**
 * @brief 统计定时器回调
 * @param timer 定时器句柄
 */
static void app_statistics_timer_callback(TimerHandle_t timer)
{
    (void)timer;

    /* 更新运行时间 */
    g_system_status.uptime_seconds += g_app_config.statistics_period_ms / 1000;

    /* 更新周期计数 */
    g_system_status.cycle_count++;

    /* 计算CPU使用率 */
    app_calculate_cpu_usage();

    /* 检查栈使用情况 */
    app_check_stack_usage();
}

/**
 * @brief 设置系统状态
 * @param new_state 新状态
 * @return app_result_t 操作结果
 */
static app_result_t app_set_state(app_state_t new_state)
{
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return APP_TIMEOUT;
    }

    app_state_t old_state = g_app_state;
    g_app_state = new_state;
    g_system_status.state = new_state;

    xSemaphoreGive(g_state_mutex);

    /* 调用状态变化回调 */
    if (g_state_change_callback != NULL && old_state != new_state) {
        g_state_change_callback(old_state, new_state);
    }

    return APP_OK;
}

/**
 * @brief 计算CPU使用率
 */
static void app_calculate_cpu_usage(void)
{
    /* 这里可以实现CPU使用率计算 */
    /* 暂时使用模拟值 */
    g_system_status.cpu_usage_percent = 25.0f;
}

/**
 * @brief 检查栈使用情况
 * @return app_result_t 操作结果
 */
app_result_t app_check_stack_usage(void)
{
    /* 检查各任务栈使用情况 */
    UBaseType_t stack_remaining;

    if (g_task_handles.safety_task != NULL) {
        stack_remaining = uxTaskGetStackHighWaterMark(g_task_handles.safety_task);
        if (stack_remaining < 100) {
            app_handle_error("SafetyTask", 0x5001);
        }
    }

    if (g_task_handles.control_task != NULL) {
        stack_remaining = uxTaskGetStackHighWaterMark(g_task_handles.control_task);
        if (stack_remaining < 100) {
            app_handle_error("ControlTask", 0x5002);
        }
    }

    if (g_task_handles.sensor_task != NULL) {
        stack_remaining = uxTaskGetStackHighWaterMark(g_task_handles.sensor_task);
        if (stack_remaining < 100) {
            app_handle_error("SensorTask", 0x5003);
        }
    }

    return APP_OK;
}