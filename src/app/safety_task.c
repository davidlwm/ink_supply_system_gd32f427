/**
 * @file    safety_task.c
 * @brief   安全任务实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "app/safety_task.h"
#include "app/sensor_task.h"
#include "app/actuator_task.h"
#include "app/system_config.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

// 标准库
#include <string.h>
#include <math.h>

// 安全系统状态
static safety_status_t g_safety_status = {0};
static safety_limits_t g_safety_limits = {0};
static safety_fault_record_t g_fault_records[32] = {0}; // 最多记录32个故障
static SemaphoreHandle_t g_safety_mutex = NULL;

// 故障计数器
static uint8_t g_fault_record_count = 0;
static bool g_emergency_stop_triggered = false;

// 默认安全限制参数
static const safety_limits_t default_safety_limits = {
    .max_temperature = {80.0f, 85.0f, 90.0f},     // 温度上限
    .min_temperature = {5.0f, 5.0f, 5.0f},        // 温度下限
    .max_pressure = {150.0f, 120.0f},             // 压力上限 (kPa)
    .min_pressure = {5.0f, 5.0f},                 // 压力下限
    .max_liquid_level = {95.0f, 95.0f},           // 液位上限 (%)
    .min_liquid_level = {10.0f, 10.0f},           // 液位下限
    .sensor_timeout_ms = 5000,                     // 传感器超时5秒
    .actuator_timeout_ms = 3000                    // 执行器超时3秒
};

/**
 * @brief  安全管理器初始化
 * @param  None
 * @retval safety_result_t 初始化结果
 */
safety_result_t safety_manager_init(void)
{
    // 创建互斥锁
    g_safety_mutex = xSemaphoreCreateMutex();
    if (g_safety_mutex == NULL) {
        return SAFETY_ERROR_WATCHDOG_TIMEOUT; // 重用错误码
    }

    // 初始化安全状态
    g_safety_status.emergency_stop = false;
    g_safety_status.system_fault = false;
    g_safety_status.safety_level = SAFETY_LEVEL_NORMAL;
    g_safety_status.fault_code = 0;
    g_safety_status.fault_count = 0;
    g_safety_status.last_check_time = xTaskGetTickCount();

    // 加载默认安全限制
    memcpy(&g_safety_limits, &default_safety_limits, sizeof(safety_limits_t));

    // 初始化故障记录
    memset(g_fault_records, 0, sizeof(g_fault_records));
    g_fault_record_count = 0;
    g_emergency_stop_triggered = false;

    return SAFETY_SUCCESS;
}

/**
 * @brief  安全任务主函数
 * @param  pvParameters 任务参数
 * @retval None
 */
void safety_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    uint32_t task_counter = 0;

    // 等待系统初始化完成
    vTaskDelay(pdMS_TO_TICKS(200));

    // 初始化安全系统
    if (safety_manager_init() != SAFETY_SUCCESS) {
        vTaskDelete(NULL);
        return;
    }

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 严格5ms周期执行 (最高优先级)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAFETY_TASK_PERIOD_MS));

        task_counter++;

        // 更新检查时间
        g_safety_status.last_check_time = xTaskGetTickCount();

        // 1. 检查急停按钮 (每周期)
        safety_check_emergency_stop();

        // 2. 检查传感器数据安全性 (每周期)
        safety_check_all_systems();

        // 3. 检查传感器健康状态 (每20ms)
        if (task_counter % (20 / SAFETY_TASK_PERIOD_MS) == 0) {
            safety_check_sensor_health();
        }

        // 4. 检查执行器健康状态 (每50ms)
        if (task_counter % (50 / SAFETY_TASK_PERIOD_MS) == 0) {
            safety_check_actuator_health();
        }

        // 5. 监控系统级安全 (每100ms)
        if (task_counter % (100 / SAFETY_TASK_PERIOD_MS) == 0) {
            safety_monitor_cycle();
        }

        // 6. 如果触发紧急停止，执行安全关闭
        if (g_emergency_stop_triggered) {
            safety_emergency_shutdown();
        }
    }
}

/**
 * @brief  检查所有系统安全性
 * @param  None
 * @retval safety_result_t 检查结果
 */
safety_result_t safety_check_all_systems(void)
{
    safety_result_t result = SAFETY_SUCCESS;

    // 检查温度安全
    if (safety_check_temperatures() != SAFETY_SUCCESS) {
        result = SAFETY_ERROR_TEMPERATURE_OVER_LIMIT;
    }

    // 检查压力安全
    if (safety_check_pressures() != SAFETY_SUCCESS) {
        result = SAFETY_ERROR_PRESSURE_OVER_LIMIT;
    }

    // 检查液位安全
    if (safety_check_liquid_levels() != SAFETY_SUCCESS) {
        result = SAFETY_ERROR_PRESSURE_OVER_LIMIT; // 重用错误码
    }

    return result;
}

/**
 * @brief  检查温度安全
 * @param  None
 * @retval safety_result_t 检查结果
 */
safety_result_t safety_check_temperatures(void)
{
    safety_result_t result = SAFETY_SUCCESS;

    for (int i = 0; i < 3; i++) {
        float temp = get_pt100_temperature(i + 4); // 温度传感器ID 4-6

        // 检查传感器故障
        if (get_sensor_fault_status(SENSOR_TYPE_TEMPERATURE, i)) {
            continue; // 传感器故障时跳过检查
        }

        // 检查温度上限
        if (temp > g_safety_limits.max_temperature[i]) {
            uint16_t fault_code = SAFETY_FAULT_TEMPERATURE_HIGH_1 + i;
            safety_handle_fault(fault_code, SAFETY_LEVEL_EMERGENCY);
            result = SAFETY_ERROR_TEMPERATURE_OVER_LIMIT;
        }

        // 检查温度下限
        if (temp < g_safety_limits.min_temperature[i]) {
            uint16_t fault_code = SAFETY_FAULT_TEMPERATURE_LOW_1 + i;
            safety_handle_fault(fault_code, SAFETY_LEVEL_WARNING);
        }
    }

    return result;
}

/**
 * @brief  检查压力安全
 * @param  None
 * @retval safety_result_t 检查结果
 */
safety_result_t safety_check_pressures(void)
{
    safety_result_t result = SAFETY_SUCCESS;

    for (int i = 0; i < 2; i++) {
        float pressure = get_pressure_sensor(i + 2); // 压力传感器ID 2-3

        // 检查传感器故障
        if (get_sensor_fault_status(SENSOR_TYPE_PRESSURE, i)) {
            continue; // 传感器故障时跳过检查
        }

        // 检查压力上限
        if (pressure > g_safety_limits.max_pressure[i]) {
            uint16_t fault_code = SAFETY_FAULT_PRESSURE_HIGH_1 + i;
            safety_handle_fault(fault_code, SAFETY_LEVEL_ALARM);
            result = SAFETY_ERROR_PRESSURE_OVER_LIMIT;
        }

        // 检查压力下限
        if (pressure < g_safety_limits.min_pressure[i]) {
            uint16_t fault_code = SAFETY_FAULT_PRESSURE_LOW_1 + i;
            safety_handle_fault(fault_code, SAFETY_LEVEL_WARNING);
        }
    }

    return result;
}

/**
 * @brief  检查液位安全
 * @param  None
 * @retval safety_result_t 检查结果
 */
safety_result_t safety_check_liquid_levels(void)
{
    safety_result_t result = SAFETY_SUCCESS;

    for (int i = 0; i < 2; i++) {
        float level = get_liquid_level_sensor(i); // 液位传感器ID 0-1

        // 检查传感器故障
        if (get_sensor_fault_status(SENSOR_TYPE_LIQUID_LEVEL, i)) {
            continue; // 传感器故障时跳过检查
        }

        // 检查液位上限
        if (level > g_safety_limits.max_liquid_level[i]) {
            uint16_t fault_code = SAFETY_FAULT_LEVEL_HIGH_1 + i;
            safety_handle_fault(fault_code, SAFETY_LEVEL_ALARM);
        }

        // 检查液位下限
        if (level < g_safety_limits.min_liquid_level[i]) {
            uint16_t fault_code = SAFETY_FAULT_LEVEL_LOW_1 + i;
            safety_handle_fault(fault_code, SAFETY_LEVEL_WARNING);
        }
    }

    return result;
}

/**
 * @brief  检查急停状态
 * @param  None
 * @retval safety_result_t 检查结果
 */
safety_result_t safety_check_emergency_stop(void)
{
    bool emergency_pressed = emergency_stop_button_pressed();

    if (emergency_pressed && !g_safety_status.emergency_stop) {
        // 急停按钮被按下
        g_safety_status.emergency_stop = true;
        g_emergency_stop_triggered = true;

        safety_handle_fault(SAFETY_FAULT_EMERGENCY_STOP, SAFETY_LEVEL_CRITICAL);
        return SAFETY_ERROR_EMERGENCY_STOP;
    }

    return SAFETY_SUCCESS;
}

/**
 * @brief  检查传感器健康状态
 * @param  None
 * @retval safety_result_t 检查结果
 */
safety_result_t safety_check_sensor_health(void)
{
    // 检查各类传感器的健康状态
    for (int i = 0; i < 7; i++) {
        bool fault = false;

        if (i < 2) {
            fault = get_sensor_fault_status(SENSOR_TYPE_LIQUID_LEVEL, i);
        } else if (i < 4) {
            fault = get_sensor_fault_status(SENSOR_TYPE_PRESSURE, i - 2);
        } else {
            fault = get_sensor_fault_status(SENSOR_TYPE_TEMPERATURE, i - 4);
        }

        if (fault) {
            safety_handle_fault(SAFETY_FAULT_SENSOR_TIMEOUT + i, SAFETY_LEVEL_ALARM);
        }
    }

    return SAFETY_SUCCESS;
}

/**
 * @brief  检查执行器健康状态
 * @param  None
 * @retval safety_result_t 检查结果
 */
safety_result_t safety_check_actuator_health(void)
{
    // 检查加热器
    for (int i = 0; i < 3; i++) {
        if (get_actuator_fault_status(ACTUATOR_TYPE_HEATER, i)) {
            safety_handle_fault(SAFETY_FAULT_ACTUATOR_TIMEOUT + i, SAFETY_LEVEL_ALARM);
        }
    }

    // 检查泵
    for (int i = 0; i < 2; i++) {
        if (get_actuator_fault_status(ACTUATOR_TYPE_PUMP, i)) {
            safety_handle_fault(SAFETY_FAULT_ACTUATOR_TIMEOUT + 3 + i, SAFETY_LEVEL_ALARM);
        }
    }

    // 检查阀门
    for (int i = 0; i < 8; i++) {
        if (get_actuator_fault_status(ACTUATOR_TYPE_VALVE, i)) {
            safety_handle_fault(SAFETY_FAULT_ACTUATOR_TIMEOUT + 5 + i, SAFETY_LEVEL_WARNING);
        }
    }

    return SAFETY_SUCCESS;
}

/**
 * @brief  处理安全故障
 * @param  fault_code 故障代码
 * @param  level 安全等级
 * @retval safety_result_t 处理结果
 */
safety_result_t safety_handle_fault(uint16_t fault_code, safety_level_t level)
{
    if (xSemaphoreTake(g_safety_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return SAFETY_ERROR_WATCHDOG_TIMEOUT;
    }

    // 更新系统安全状态
    g_safety_status.system_fault = true;
    g_safety_status.fault_code = fault_code;
    g_safety_status.fault_count++;

    // 更新安全等级 (只升级，不降级)
    if (level > g_safety_status.safety_level) {
        g_safety_status.safety_level = level;
    }

    // 记录故障
    safety_update_fault_records(fault_code, level);

    // 根据安全等级执行相应动作
    switch (level) {
        case SAFETY_LEVEL_WARNING:
            // 警告级别：记录日志，发出警告
            break;

        case SAFETY_LEVEL_ALARM:
            // 报警级别：限制部分功能
            break;

        case SAFETY_LEVEL_EMERGENCY:
            // 紧急级别：立即停止相关执行器
            safety_trigger_emergency_actions();
            break;

        case SAFETY_LEVEL_CRITICAL:
            // 关键级别：系统完全停止
            g_emergency_stop_triggered = true;
            break;

        default:
            break;
    }

    xSemaphoreGive(g_safety_mutex);
    return SAFETY_SUCCESS;
}

/**
 * @brief  紧急关闭系统
 * @param  None
 * @retval safety_result_t 关闭结果
 */
safety_result_t safety_emergency_shutdown(void)
{
    // 立即关闭所有加热器
    for (int i = 0; i < 3; i++) {
        set_heater_power(i, 0.0f);
    }

    // 立即停止所有泵
    for (int i = 0; i < 2; i++) {
        set_pump_speed(i, 0);
    }

    // 关闭所有阀门
    for (int i = 0; i < 8; i++) {
        set_valve_state(i, false);
    }

    // 设置安全指示
    set_led_state(LED_ALARM, true, LED_BLINK_FAST);

    return SAFETY_SUCCESS;
}

/**
 * @brief  获取系统安全状态
 * @param  status 状态结构指针
 * @retval safety_result_t 获取结果
 */
safety_result_t safety_get_system_status(safety_status_t *status)
{
    if (status == NULL) {
        return SAFETY_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_safety_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(status, &g_safety_status, sizeof(safety_status_t));
        xSemaphoreGive(g_safety_mutex);
        return SAFETY_SUCCESS;
    }

    return SAFETY_ERROR_WATCHDOG_TIMEOUT;
}

/**
 * @brief  检查系统是否安全
 * @param  None
 * @retval bool 安全状态
 */
bool safety_is_system_safe(void)
{
    return (g_safety_status.safety_level <= SAFETY_LEVEL_WARNING) &&
           (!g_safety_status.emergency_stop);
}

/**
 * @brief  检查急停是否激活
 * @param  None
 * @retval bool 急停状态
 */
bool safety_is_emergency_stop_active(void)
{
    return g_safety_status.emergency_stop;
}

/**
 * @brief  系统恢复
 * @param  None
 * @retval safety_result_t 恢复结果
 */
safety_result_t safety_system_recovery(void)
{
    if (xSemaphoreTake(g_safety_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return SAFETY_ERROR_WATCHDOG_TIMEOUT;
    }

    // 检查急停按钮是否释放
    if (emergency_stop_button_pressed()) {
        xSemaphoreGive(g_safety_mutex);
        return SAFETY_ERROR_EMERGENCY_STOP;
    }

    // 重置安全状态
    g_safety_status.emergency_stop = false;
    g_safety_status.system_fault = false;
    g_safety_status.safety_level = SAFETY_LEVEL_NORMAL;
    g_safety_status.fault_code = 0;
    g_emergency_stop_triggered = false;

    // 关闭报警指示
    set_led_state(LED_ALARM, false, 0);

    xSemaphoreGive(g_safety_mutex);
    return SAFETY_SUCCESS;
}

/**
 * @brief  更新故障记录
 * @param  fault_code 故障代码
 * @param  level 安全等级
 * @retval None
 */
static void safety_update_fault_records(uint16_t fault_code, safety_level_t level)
{
    // 查找是否已存在相同故障代码
    for (int i = 0; i < g_fault_record_count; i++) {
        if (g_fault_records[i].fault_code == fault_code) {
            g_fault_records[i].occurrence_count++;
            g_fault_records[i].timestamp = xTaskGetTickCount();
            g_fault_records[i].active = true;
            if (level > g_fault_records[i].level) {
                g_fault_records[i].level = level;
            }
            return;
        }
    }

    // 添加新的故障记录
    if (g_fault_record_count < 32) {
        g_fault_records[g_fault_record_count].fault_code = fault_code;
        g_fault_records[g_fault_record_count].level = level;
        g_fault_records[g_fault_record_count].timestamp = xTaskGetTickCount();
        g_fault_records[g_fault_record_count].active = true;
        g_fault_records[g_fault_record_count].occurrence_count = 1;
        g_fault_record_count++;
    }
}

/**
 * @brief  触发紧急动作
 * @param  None
 * @retval None
 */
static void safety_trigger_emergency_actions(void)
{
    // 根据故障类型执行相应的紧急动作
    switch (g_safety_status.fault_code) {
        case SAFETY_FAULT_TEMPERATURE_HIGH_1:
        case SAFETY_FAULT_TEMPERATURE_HIGH_2:
        case SAFETY_FAULT_TEMPERATURE_HIGH_3:
            // 温度过高：立即关闭对应加热器
            {
                int heater_id = g_safety_status.fault_code - SAFETY_FAULT_TEMPERATURE_HIGH_1;
                set_heater_power(heater_id, 0.0f);
            }
            break;

        case SAFETY_FAULT_PRESSURE_HIGH_1:
        case SAFETY_FAULT_PRESSURE_HIGH_2:
            // 压力过高：立即停止对应泵
            {
                int pump_id = g_safety_status.fault_code - SAFETY_FAULT_PRESSURE_HIGH_1;
                set_pump_speed(pump_id, 0);
            }
            break;

        default:
            break;
    }
}

/**
 * @brief  安全监控周期
 * @param  None
 * @retval None
 */
static void safety_monitor_cycle(void)
{
    // 这里可以添加周期性的安全监控逻辑
    // 例如：系统自检、看门狗喂狗等
}

// 静态函数实现
static void safety_task_init(void)
{
    // 任务初始化的实现
}

/**
 * @brief  设置安全限制
 * @param  limits 安全限制参数
 * @retval safety_result_t 设置结果
 */
safety_result_t safety_set_limits(const safety_limits_t *limits)
{
    if (limits == NULL) {
        return SAFETY_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_safety_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&g_safety_limits, limits, sizeof(safety_limits_t));
        xSemaphoreGive(g_safety_mutex);
        return SAFETY_SUCCESS;
    }

    return SAFETY_ERROR_WATCHDOG_TIMEOUT;
}

/**
 * @brief  获取安全限制
 * @param  limits 安全限制参数
 * @retval safety_result_t 获取结果
 */
safety_result_t safety_get_limits(safety_limits_t *limits)
{
    if (limits == NULL) {
        return SAFETY_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_safety_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(limits, &g_safety_limits, sizeof(safety_limits_t));
        xSemaphoreGive(g_safety_mutex);
        return SAFETY_SUCCESS;
    }

    return SAFETY_ERROR_WATCHDOG_TIMEOUT;
}