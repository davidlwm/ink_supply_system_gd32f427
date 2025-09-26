/**
 * @file    actuator_task.c
 * @brief   执行器任务实现 - 保持v1版本所有执行器功能
 * @version V4.0
 * @date    2025-09-27
 */

#include "app/actuator_task.h"
#include "drivers/actuators/mra23d3_driver.h"
#include "drivers/actuators/mpb025bbb_driver.h"
#include "drivers/actuators/valve_control.h"
#include "config/system_config.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// 执行器状态结构 (基于v1设计)
typedef struct {
    // 加热器状态 (MRA-23D3)
    struct {
        bool enabled;               // 使能状态
        float power_percent;        // 功率百分比 (0-100%)
        float target_temperature;   // 目标温度
        uint32_t total_run_time;    // 总运行时间 (ms)
        bool fault_status;          // 故障状态
        uint16_t fault_code;        // 故障代码
    } heater[HEATER_COUNT];

    // 泵状态 (MPB025BBB)
    struct {
        bool enabled;               // 使能状态
        uint16_t speed_rpm;         // 转速 (RPM)
        float flow_rate;            // 流量 (L/min)
        uint32_t total_run_time;    // 总运行时间 (ms)
        bool fault_status;          // 故障状态
        uint16_t fault_code;        // 故障代码
    } pump[PUMP_COUNT];

    // 电磁阀状态
    struct {
        bool state;                 // 开关状态 (true=开, false=关)
        uint32_t total_switch_count; // 总切换次数
        uint32_t last_switch_time;  // 最后切换时间
        bool fault_status;          // 故障状态
        uint16_t fault_code;        // 故障代码
    } valve[VALVE_COUNT];

    // LED状态
    struct {
        bool state;                 // 亮灭状态
        uint8_t brightness;         // 亮度 (0-100%)
        uint32_t blink_period;      // 闪烁周期 (ms, 0=常亮)
        uint32_t last_toggle_time;  // 上次切换时间
    } led[LED_COUNT];

    uint32_t last_update_time;      // 最后更新时间

} actuator_status_t;

// 全局执行器状态
static actuator_status_t g_actuator_status;
static SemaphoreHandle_t actuator_status_mutex;

/**
 * @brief  执行器管理器初始化
 * @param  None
 * @retval None
 */
void actuator_manager_init(void)
{
    // 创建执行器状态互斥锁
    actuator_status_mutex = xSemaphoreCreateMutex();

    // 初始化执行器硬件
    mra23d3_heater_init();      // 加热器初始化
    mpb025bbb_pump_init();      // 泵初始化
    valve_control_init();       // 电磁阀初始化
    led_control_init();         // LED初始化

    // 初始化保护系统
    actuator_protection_init();

    // 清零执行器状态
    memset(&g_actuator_status, 0, sizeof(actuator_status_t));

    // 设置默认状态
    for (int i = 0; i < HEATER_COUNT; i++) {
        g_actuator_status.heater[i].enabled = false;
        g_actuator_status.heater[i].power_percent = 0.0f;
    }

    for (int i = 0; i < PUMP_COUNT; i++) {
        g_actuator_status.pump[i].enabled = false;
        g_actuator_status.pump[i].speed_rpm = 0;
    }

    for (int i = 0; i < VALVE_COUNT; i++) {
        g_actuator_status.valve[i].state = false;
    }

    for (int i = 0; i < LED_COUNT; i++) {
        g_actuator_status.led[i].state = false;
        g_actuator_status.led[i].brightness = 100;
    }
}

/**
 * @brief  执行器任务主函数 (保持v1设计模式)
 * @param  pvParameters 任务参数
 * @retval None
 */
void actuator_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    uint32_t task_counter = 0;

    // 任务初始化
    actuator_task_init();

    // 初始化时间基准
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 严格50ms周期执行
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ACTUATOR_UPDATE_PERIOD_MS));

        task_counter++;

        // 获取执行器状态互斥锁
        if (xSemaphoreTake(actuator_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

            // 1. 更新加热器输出 (MRA-23D3)
            for (int i = 0; i < HEATER_COUNT; i++) {
                if (g_actuator_status.heater[i].enabled) {
                    // 安全检查
                    if (actuator_safety_check(ACTUATOR_TYPE_HEATER, i)) {
                        // 更新加热器功率
                        control_heater_mra23d3(i, g_actuator_status.heater[i].power_percent);
                        g_actuator_status.heater[i].total_run_time += ACTUATOR_UPDATE_PERIOD_MS;
                        g_actuator_status.heater[i].fault_status = false;
                    } else {
                        // 安全故障，关闭加热器
                        control_heater_mra23d3(i, 0.0f);
                        g_actuator_status.heater[i].fault_status = true;
                        g_actuator_status.heater[i].fault_code = FAULT_HEATER_1_SAFETY + i;
                    }
                } else {
                    // 关闭加热器
                    control_heater_mra23d3(i, 0.0f);
                }
            }

            // 2. 更新泵输出 (MPB025BBB)
            for (int i = 0; i < PUMP_COUNT; i++) {
                if (g_actuator_status.pump[i].enabled) {
                    // 安全检查
                    if (actuator_safety_check(ACTUATOR_TYPE_PUMP, i)) {
                        // 更新泵转速
                        control_pump_speed_mpb025bbb(i, g_actuator_status.pump[i].speed_rpm);
                        g_actuator_status.pump[i].total_run_time += ACTUATOR_UPDATE_PERIOD_MS;
                        g_actuator_status.pump[i].fault_status = false;

                        // 更新流量 (简化计算)
                        g_actuator_status.pump[i].flow_rate =
                            (float)g_actuator_status.pump[i].speed_rpm * 0.001f; // 1RPM = 0.001L/min
                    } else {
                        // 安全故障，停止泵
                        control_pump_speed_mpb025bbb(i, 0);
                        g_actuator_status.pump[i].fault_status = true;
                        g_actuator_status.pump[i].fault_code = FAULT_PUMP_1_SAFETY + i;
                    }
                } else {
                    // 停止泵
                    control_pump_speed_mpb025bbb(i, 0);
                    g_actuator_status.pump[i].flow_rate = 0.0f;
                }
            }

            // 3. 更新电磁阀输出
            for (int i = 0; i < VALVE_COUNT; i++) {
                // 安全检查
                if (actuator_safety_check(ACTUATOR_TYPE_VALVE, i)) {
                    control_valve(i, g_actuator_status.valve[i].state);
                    g_actuator_status.valve[i].fault_status = false;
                } else {
                    // 安全故障，关闭阀门
                    control_valve(i, false);
                    g_actuator_status.valve[i].fault_status = true;
                    g_actuator_status.valve[i].fault_code = FAULT_VALVE_1_SAFETY + i;
                }
            }

            // 4. 更新LED输出
            update_led_outputs();

            g_actuator_status.last_update_time = xTaskGetTickCount();

            // 释放互斥锁
            xSemaphoreGive(actuator_status_mutex);
        }

        // 性能监控 (每1000次报告一次)
        if (task_counter % 1000 == 0) {
            report_actuator_performance(task_counter);
        }
    }
}

/**
 * @brief  执行器任务初始化
 * @param  None
 * @retval None
 */
static void actuator_task_init(void)
{
    // 执行器硬件初始化
    actuator_manager_init();

    // 执行器自检
    actuator_self_test();
}

/**
 * @brief  更新LED输出
 * @param  None
 * @retval None
 */
static void update_led_outputs(void)
{
    uint32_t current_time = xTaskGetTickCount();

    for (int i = 0; i < LED_COUNT; i++) {
        bool output_state = g_actuator_status.led[i].state;

        // 处理闪烁逻辑
        if (g_actuator_status.led[i].blink_period > 0) {
            uint32_t time_in_period = current_time % g_actuator_status.led[i].blink_period;
            uint32_t half_period = g_actuator_status.led[i].blink_period / 2;

            output_state = (time_in_period < half_period) ? true : false;
        }

        // 应用亮度控制 (简化实现)
        if (output_state && g_actuator_status.led[i].brightness < 100) {
            // 这里可以实现PWM亮度控制
            output_state = true; // 简化为开/关
        }

        // 输出到硬件
        control_led(i, output_state);
    }
}

// 公共接口函数 (保持v1接口兼容)
bool set_heater_power(uint8_t heater_id, float power_percent)
{
    if (heater_id >= HEATER_COUNT || power_percent < 0.0f || power_percent > 100.0f) {
        return false;
    }

    bool result = false;
    if (xSemaphoreTake(actuator_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_actuator_status.heater[heater_id].power_percent = power_percent;
        g_actuator_status.heater[heater_id].enabled = (power_percent > 0.0f);
        result = true;
        xSemaphoreGive(actuator_status_mutex);
    }
    return result;
}

bool set_pump_speed(uint8_t pump_id, uint16_t speed_rpm)
{
    if (pump_id >= PUMP_COUNT || speed_rpm > 5000) {
        return false;
    }

    bool result = false;
    if (xSemaphoreTake(actuator_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_actuator_status.pump[pump_id].speed_rpm = speed_rpm;
        g_actuator_status.pump[pump_id].enabled = (speed_rpm > 0);
        result = true;
        xSemaphoreGive(actuator_status_mutex);
    }
    return result;
}

bool set_valve_state(uint8_t valve_id, bool open)
{
    if (valve_id >= VALVE_COUNT) {
        return false;
    }

    bool result = false;
    if (xSemaphoreTake(actuator_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (g_actuator_status.valve[valve_id].state != open) {
            g_actuator_status.valve[valve_id].total_switch_count++;
            g_actuator_status.valve[valve_id].last_switch_time = xTaskGetTickCount();
        }
        g_actuator_status.valve[valve_id].state = open;
        result = true;
        xSemaphoreGive(actuator_status_mutex);
    }
    return result;
}

bool set_led_state(uint8_t led_id, bool on, uint32_t blink_period_ms)
{
    if (led_id >= LED_COUNT) {
        return false;
    }

    bool result = false;
    if (xSemaphoreTake(actuator_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_actuator_status.led[led_id].state = on;
        g_actuator_status.led[led_id].blink_period = blink_period_ms;
        result = true;
        xSemaphoreGive(actuator_status_mutex);
    }
    return result;
}

// 状态查询函数
float get_heater_power(uint8_t heater_id)
{
    if (heater_id >= HEATER_COUNT) return 0.0f;

    float power = 0.0f;
    if (xSemaphoreTake(actuator_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        power = g_actuator_status.heater[heater_id].power_percent;
        xSemaphoreGive(actuator_status_mutex);
    }
    return power;
}

uint16_t get_pump_speed(uint8_t pump_id)
{
    if (pump_id >= PUMP_COUNT) return 0;

    uint16_t speed = 0;
    if (xSemaphoreTake(actuator_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        speed = g_actuator_status.pump[pump_id].speed_rpm;
        xSemaphoreGive(actuator_status_mutex);
    }
    return speed;
}

bool get_valve_state(uint8_t valve_id)
{
    if (valve_id >= VALVE_COUNT) return false;

    bool state = false;
    if (xSemaphoreTake(actuator_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        state = g_actuator_status.valve[valve_id].state;
        xSemaphoreGive(actuator_status_mutex);
    }
    return state;
}

bool get_actuator_fault_status(uint8_t actuator_type, uint8_t actuator_id)
{
    bool fault = true;

    if (xSemaphoreTake(actuator_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        switch (actuator_type) {
            case ACTUATOR_TYPE_HEATER:
                if (actuator_id < HEATER_COUNT) {
                    fault = g_actuator_status.heater[actuator_id].fault_status;
                }
                break;
            case ACTUATOR_TYPE_PUMP:
                if (actuator_id < PUMP_COUNT) {
                    fault = g_actuator_status.pump[actuator_id].fault_status;
                }
                break;
            case ACTUATOR_TYPE_VALVE:
                if (actuator_id < VALVE_COUNT) {
                    fault = g_actuator_status.valve[actuator_id].fault_status;
                }
                break;
        }
        xSemaphoreGive(actuator_status_mutex);
    }

    return fault;
}

/* 控制任务兼容接口实现 */

/**
 * @brief  设置加热器功率 (兼容接口)
 * @param  heater_id 加热器ID
 * @param  power_percent 功率百分比
 * @retval bool 设置结果
 */
bool actuator_set_heater_power(uint8_t heater_id, float power_percent)
{
    return set_heater_power(heater_id, power_percent);
}

/**
 * @brief  设置泵转速 (兼容接口)
 * @param  pump_id 泵ID
 * @param  speed_rpm 转速
 * @retval bool 设置结果
 */
bool actuator_set_pump_speed(uint8_t pump_id, uint16_t speed_rpm)
{
    return set_pump_speed(pump_id, speed_rpm);
}

/**
 * @brief  设置阀门状态 (兼容接口)
 * @param  valve_id 阀门ID
 * @param  state 状态
 * @retval bool 设置结果
 */
bool actuator_set_valve_state(uint8_t valve_id, bool state)
{
    return set_valve_state(valve_id, state);
}