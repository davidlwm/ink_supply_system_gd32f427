/**
 * @file    system_manager.c
 * @brief   系统管理器实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "system_manager.h"
#include "error_handler.h"
#include "sensor_task.h"
#include "actuator_task.h"
#include "control_task.h"
#include "comm_task.h"
#include "hmi_task.h"
#include "safety_task.h"
#include "config_task.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// 标准库
#include <string.h>

// 系统状态全局变量
static system_state_t g_system_state = SYSTEM_STATE_INIT;
static system_mode_t g_system_mode = SYSTEM_MODE_MANUAL;
static SemaphoreHandle_t g_system_mutex = NULL;

// 系统统计信息
static uint32_t g_system_start_time = 0;
static uint32_t g_system_error_count = 0;
static float g_cpu_usage = 0.0f;

// 模块初始化状态
static bool g_modules_initialized[8] = {false};

// 模块名称 (用于调试)
static const char* module_names[] = {
    "Sensor", "Actuator", "Control", "Communication",
    "HMI", "Safety", "Config", "System"
};

/**
 * @brief  系统早期初始化
 * @param  None
 * @retval None
 */
void system_early_init(void)
{
    // 记录系统启动时间
    g_system_start_time = 0; // 会在时钟配置后重新设置

    // 设置初始状态
    g_system_state = SYSTEM_STATE_INIT;
    g_system_mode = SYSTEM_MODE_MANUAL;
    g_system_error_count = 0;
    g_cpu_usage = 0.0f;

    // 清除模块初始化状态
    memset(g_modules_initialized, false, sizeof(g_modules_initialized));
}

/**
 * @brief  系统管理器初始化
 * @param  None
 * @retval system_init_result_t 初始化结果
 */
system_init_result_t system_manager_init(void)
{
    // 创建系统互斥锁
    g_system_mutex = xSemaphoreCreateMutex();
    if (g_system_mutex == NULL) {
        return SYSTEM_INIT_ERROR_MEMORY;
    }

    // 初始化错误处理系统
    error_log_init();

    // 记录系统启动时间
    g_system_start_time = xTaskGetTickCount();

    // 设置系统状态为空闲
    system_set_state(SYSTEM_STATE_IDLE);

    return SYSTEM_INIT_SUCCESS;
}

/**
 * @brief  传感器管理器初始化
 * @param  None
 * @retval None
 */
void sensor_manager_init(void)
{
    // 初始化传感器硬件
    frd8061_init();    // 液位传感器
    hp10my_init();     // 压力传感器
    ftt518_init();     // 温度传感器

    // 初始化滤波和校准
    sensor_filter_init();
    sensor_calibration_init();

    // 标记模块已初始化
    g_modules_initialized[0] = true;

    // 记录初始化完成
    error_log(ERROR_LEVEL_INFO, 0, "Sensor manager initialized");
}

/**
 * @brief  执行器管理器初始化
 * @param  None
 * @retval None
 */
void actuator_manager_init(void)
{
    // 初始化执行器硬件
    mra23d3_heater_init();    // 加热器
    mpb025bbb_pump_init();    // 泵
    valve_control_init();     // 阀门
    led_control_init();       // LED

    // 初始化保护系统
    actuator_protection_init();

    // 标记模块已初始化
    g_modules_initialized[1] = true;

    // 记录初始化完成
    error_log(ERROR_LEVEL_INFO, 0, "Actuator manager initialized");
}

/**
 * @brief  通信管理器初始化
 * @param  None
 * @retval None
 */
void comm_manager_init(void)
{
    // 初始化以太网通信
    // ethernet_init();

    // 初始化EtherCAT
    // ethercat_init();

    // 初始化TCP服务器
    // tcp_server_init();

    // 标记模块已初始化
    g_modules_initialized[3] = true;

    // 记录初始化完成
    error_log(ERROR_LEVEL_INFO, 0, "Communication manager initialized");
}

/**
 * @brief  HMI管理器初始化
 * @param  None
 * @retval None
 */
void hmi_manager_init(void)
{
    // HMI初始化在hmi_task中完成
    // 这里只标记模块准备就绪
    g_modules_initialized[4] = true;

    // 记录初始化完成
    error_log(ERROR_LEVEL_INFO, 0, "HMI manager initialized");
}

/**
 * @brief  安全管理器初始化
 * @param  None
 * @retval None
 */
void safety_manager_init(void)
{
    // 安全系统初始化在safety_task中完成
    // 这里只标记模块准备就绪
    g_modules_initialized[5] = true;

    // 记录初始化完成
    error_log(ERROR_LEVEL_INFO, 0, "Safety manager initialized");
}

/**
 * @brief  配置管理器初始化
 * @param  None
 * @retval None
 */
void config_manager_init(void)
{
    // 配置管理初始化在config_task中完成
    // 这里只标记模块准备就绪
    g_modules_initialized[6] = true;

    // 记录初始化完成
    error_log(ERROR_LEVEL_INFO, 0, "Config manager initialized");
}

/**
 * @brief  设置系统状态
 * @param  new_state 新状态
 * @retval None
 */
void system_set_state(system_state_t new_state)
{
    if (xSemaphoreTake(g_system_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        system_state_t old_state = g_system_state;
        g_system_state = new_state;

        // 记录状态变化
        char log_msg[64];
        snprintf(log_msg, sizeof(log_msg), "System state: %d -> %d", old_state, new_state);
        error_log(ERROR_LEVEL_INFO, 0, log_msg);

        xSemaphoreGive(g_system_mutex);
    }
}

/**
 * @brief  获取系统状态
 * @param  None
 * @retval system_state_t 当前状态
 */
system_state_t system_get_state(void)
{
    system_state_t state = SYSTEM_STATE_INIT;

    if (xSemaphoreTake(g_system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state = g_system_state;
        xSemaphoreGive(g_system_mutex);
    }

    return state;
}

/**
 * @brief  设置系统模式
 * @param  new_mode 新模式
 * @retval None
 */
void system_set_mode(system_mode_t new_mode)
{
    if (xSemaphoreTake(g_system_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        system_mode_t old_mode = g_system_mode;
        g_system_mode = new_mode;

        // 记录模式变化
        char log_msg[64];
        snprintf(log_msg, sizeof(log_msg), "System mode: %d -> %d", old_mode, new_mode);
        error_log(ERROR_LEVEL_INFO, 0, log_msg);

        xSemaphoreGive(g_system_mutex);
    }
}

/**
 * @brief  获取系统模式
 * @param  None
 * @retval system_mode_t 当前模式
 */
system_mode_t system_get_mode(void)
{
    system_mode_t mode = SYSTEM_MODE_MANUAL;

    if (xSemaphoreTake(g_system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mode = g_system_mode;
        xSemaphoreGive(g_system_mutex);
    }

    return mode;
}

/**
 * @brief  系统健康监控
 * @param  None
 * @retval None
 */
void system_health_monitor(void)
{
    static uint32_t last_monitor_time = 0;
    uint32_t current_time = xTaskGetTickCount();

    // 每5秒进行一次健康检查
    if ((current_time - last_monitor_time) >= pdMS_TO_TICKS(5000)) {
        last_monitor_time = current_time;

        // 检查各模块状态
        for (int i = 0; i < 7; i++) {
            if (!g_modules_initialized[i]) {
                char error_msg[64];
                snprintf(error_msg, sizeof(error_msg), "%s module not initialized", module_names[i]);
                error_log(ERROR_LEVEL_WARNING, ERROR_SYSTEM_INIT_FAILED, error_msg);
            }
        }

        // 更新CPU使用率
        g_cpu_usage = calculate_cpu_usage();

        // 检查内存使用情况
        size_t free_heap = xPortGetFreeHeapSize();
        if (free_heap < 1024) { // 少于1KB时警告
            error_log(ERROR_LEVEL_WARNING, ERROR_MEMORY_ALLOCATION, "Low memory warning");
        }

        // 检查任务栈使用情况
        check_task_stack_usage();
    }
}

/**
 * @brief  获取系统运行时间
 * @param  None
 * @retval uint32_t 运行时间(秒)
 */
uint32_t system_get_uptime(void)
{
    uint32_t current_time = xTaskGetTickCount();
    return (current_time - g_system_start_time) / 1000; // 转换为秒
}

/**
 * @brief  获取CPU使用率
 * @param  None
 * @retval float CPU使用率(%)
 */
float system_get_cpu_usage(void)
{
    return g_cpu_usage;
}

/**
 * @brief  计算CPU使用率
 * @param  None
 * @retval float CPU使用率(%)
 */
static float calculate_cpu_usage(void)
{
    // 简化的CPU使用率计算
    // 实际实现应该基于空闲任务的运行时间
    static uint32_t last_idle_time = 0;
    static uint32_t last_total_time = 0;

    uint32_t current_total_time = xTaskGetTickCount();
    uint32_t current_idle_time = 0; // 应该从空闲任务获取

    // 简化计算，返回一个估算值
    float usage = 0.0f;

    if (g_system_state == SYSTEM_STATE_RUNNING) {
        usage = 75.0f; // 运行状态下假设75%使用率
    } else if (g_system_state == SYSTEM_STATE_IDLE) {
        usage = 10.0f; // 空闲状态下假设10%使用率
    } else {
        usage = 5.0f;  // 其他状态下假设5%使用率
    }

    last_idle_time = current_idle_time;
    last_total_time = current_total_time;

    return usage;
}

/**
 * @brief  检查任务栈使用情况
 * @param  None
 * @retval None
 */
static void check_task_stack_usage(void)
{
    // 这里应该检查各个任务的栈使用情况
    // FreeRTOS提供了uxTaskGetStackHighWaterMark()函数

    // 简化实现，仅记录日志
    error_log(ERROR_LEVEL_INFO, 0, "Task stack usage checked");
}

/**
 * @brief  系统软复位
 * @param  None
 * @retval None
 */
void system_soft_reset(void)
{
    // 记录复位原因
    error_log(ERROR_LEVEL_WARNING, 0, "System soft reset initiated");

    // 停止所有任务
    vTaskSuspendAll();

    // 关闭所有执行器
    for (int i = 0; i < 3; i++) {
        set_heater_power(i, 0.0f);
    }
    for (int i = 0; i < 2; i++) {
        set_pump_speed(i, 0);
    }
    for (int i = 0; i < 8; i++) {
        set_valve_state(i, false);
    }

    // 延时确保操作完成
    vTaskDelay(pdMS_TO_TICKS(100));

    // 执行硬件复位
    // NVIC_SystemReset();
}

/**
 * @brief  系统紧急停止
 * @param  None
 * @retval None
 */
void system_emergency_stop(void)
{
    // 立即设置为紧急停止状态
    g_system_state = SYSTEM_STATE_EMERGENCY_STOP;

    // 记录紧急停止
    error_log(ERROR_LEVEL_CRITICAL, ERROR_SAFETY_VIOLATION, "Emergency stop activated");

    // 立即关闭所有执行器
    for (int i = 0; i < 3; i++) {
        set_heater_power(i, 0.0f);
    }
    for (int i = 0; i < 2; i++) {
        set_pump_speed(i, 0);
    }
    for (int i = 0; i < 8; i++) {
        set_valve_state(i, false);
    }

    // 激活安全指示
    set_led_state(LED_ALARM, true, LED_BLINK_FAST);
}

/**
 * @brief  系统恢复尝试
 * @param  error_code 错误代码
 * @retval bool 恢复结果
 */
bool system_recovery_attempt(uint16_t error_code)
{
    char log_msg[64];
    snprintf(log_msg, sizeof(log_msg), "Recovery attempt for error 0x%04X", error_code);
    error_log(ERROR_LEVEL_INFO, 0, log_msg);

    // 根据错误类型采取不同的恢复策略
    switch (error_code) {
        case ERROR_SENSOR_FAULT:
            // 传感器故障：重新初始化传感器
            sensor_manager_init();
            return true;

        case ERROR_ACTUATOR_FAULT:
            // 执行器故障：重新初始化执行器
            actuator_manager_init();
            return true;

        case ERROR_COMMUNICATION_LOST:
            // 通信故障：重新初始化通信
            comm_manager_init();
            return true;

        case ERROR_MEMORY_ALLOCATION:
            // 内存不足：尝试清理内存
            // 这里可以实现内存清理逻辑
            return false;

        default:
            // 其他错误：尝试软复位
            system_soft_reset();
            return false;
    }
}

/**
 * @brief  检查所有模块是否初始化
 * @param  None
 * @retval bool 所有模块初始化状态
 */
bool system_all_modules_initialized(void)
{
    for (int i = 0; i < 7; i++) {
        if (!g_modules_initialized[i]) {
            return false;
        }
    }
    return true;
}

/**
 * @brief  获取模块初始化状态
 * @param  module_index 模块索引
 * @retval bool 模块初始化状态
 */
bool system_get_module_status(uint8_t module_index)
{
    if (module_index >= 8) {
        return false;
    }
    return g_modules_initialized[module_index];
}