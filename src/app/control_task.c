/**
 * @file    control_task.c
 * @brief   控制任务实现 - PID控制算法和系统控制逻辑 (8周v4标准)
 * @version V4.0
 * @date    2024-12-27
 */

#include "app/control_task.h"
#include "app/sensor_task.h"
#include "app/actuator_task.h"
#include "app/safety_task.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

// 中间件 (8周v4标准)
#include "middleware/pid.h"
#include "middleware/filter.h"
#include "middleware/tasks.h"

// HAL层
#include "hal/adc_hal.h"
#include "hal/pwm_hal.h"

// 控制系统状态
typedef struct {
    bool control_enabled;          // 控制使能状态
    bool auto_mode;               // 自动/手动模式
    uint8_t control_mode;         // 控制模式
    uint32_t control_cycle_count; // 控制周期计数
    float system_efficiency;     // 系统效率
    SemaphoreHandle_t status_mutex; // 状态互斥锁
} control_system_status_t;

// 温度控制器组
typedef struct {
    pid_controller_t pid;         // PID控制器 (使用v4中间件)
    moving_average_filter_t filter; // 滤波器
    float target_temp;           // 目标温度
    float current_temp;          // 当前温度
    float output_power;          // 输出功率
    bool enable;                 // 使能状态
    uint32_t stable_count;       // 稳定计数
    float max_power;             // 最大功率限制
} temperature_controller_t;

// 压力控制器组
typedef struct {
    pid_controller_t pid;         // PID控制器 (使用v4中间件)
    moving_average_filter_t filter; // 滤波器
    float target_pressure;       // 目标压力
    float current_pressure;      // 当前压力
    uint16_t pump_speed;         // 泵转速输出
    bool enable;                 // 使能状态
    uint32_t stable_count;       // 稳定计数
    uint16_t max_speed;          // 最大转速限制
} pressure_controller_t;

// 液位控制器组
typedef struct {
    float target_level;          // 目标液位
    float current_level;         // 当前液位
    bool valve_state;            // 阀门状态
    bool enable;                 // 使能状态
    float hysteresis;            // 滞回差
    uint32_t last_action_time;   // 上次动作时间
} level_controller_t;

// 全局控制系统状态
static control_system_status_t g_control_status = {0};

// 控制器实例
static temperature_controller_t g_temp_controllers[3] = {0}; // 3路温度控制
static pressure_controller_t g_pressure_controllers[2] = {0}; // 2路压力控制
static level_controller_t g_level_controllers[2] = {0};      // 2路液位控制

// 控制参数配置
static const control_config_t default_control_config = {
    .temperature_control = {
        // 温度控制PID参数 (墨盒加热器)
        .pid_params = {
            {.kp = 2.0f, .ki = 0.1f, .kd = 0.05f}, // 温度控制器1
            {.kp = 2.0f, .ki = 0.1f, .kd = 0.05f}, // 温度控制器2
            {.kp = 1.8f, .ki = 0.08f, .kd = 0.04f} // 温度控制器3
        },
        .target_temps = {60.0f, 65.0f, 70.0f},     // 目标温度
        .max_powers = {100.0f, 100.0f, 80.0f},     // 最大功率限制
        .temp_tolerance = 2.0f,                     // 温度容差
        .overshoot_limit = 5.0f                     // 超调限制
    },
    .pressure_control = {
        // 压力控制PID参数
        .pid_params = {
            {.kp = 1.5f, .ki = 0.05f, .kd = 0.02f}, // 压力控制器1
            {.kp = 1.5f, .ki = 0.05f, .kd = 0.02f}  // 压力控制器2
        },
        .target_pressures = {50.0f, 45.0f},         // 目标压力(kPa)
        .max_speeds = {4500, 4000},                 // 最大泵速(RPM)
        .pressure_tolerance = 5.0f,                 // 压力容差
        .min_pump_speed = 500                       // 最小泵速
    },
    .level_control = {
        .target_levels = {80.0f, 75.0f},           // 目标液位(%)
        .hysteresis = {5.0f, 5.0f},                // 滞回差
        .min_action_interval = 2000                 // 最小动作间隔(ms)
    }
};

/**
 * @brief  控制系统初始化
 * @param  None
 * @retval control_result_t 初始化结果
 */
control_result_t control_manager_init(void)
{
    // 创建状态互斥锁
    g_control_status.status_mutex = xSemaphoreCreateMutex();
    if (g_control_status.status_mutex == NULL) {
        return CONTROL_ERROR_MEMORY_ALLOCATION;
    }

    // 初始化控制状态
    g_control_status.control_enabled = false;
    g_control_status.auto_mode = true;
    g_control_status.control_mode = CONTROL_MODE_AUTOMATIC;
    g_control_status.control_cycle_count = 0;
    g_control_status.system_efficiency = 0.0f;

    // 初始化温度控制器
    for (int i = 0; i < 3; i++) {
        pid_controller_init(&g_temp_controllers[i].pid,
                           default_control_config.temperature_control.pid_params[i].kp,
                           default_control_config.temperature_control.pid_params[i].ki,
                           default_control_config.temperature_control.pid_params[i].kd);

        g_temp_controllers[i].target_temp = default_control_config.temperature_control.target_temps[i];
        g_temp_controllers[i].max_power = default_control_config.temperature_control.max_powers[i];
        g_temp_controllers[i].enable = false;
        g_temp_controllers[i].stable_count = 0;
    }

    // 初始化压力控制器
    for (int i = 0; i < 2; i++) {
        pid_controller_init(&g_pressure_controllers[i].pid,
                           default_control_config.pressure_control.pid_params[i].kp,
                           default_control_config.pressure_control.pid_params[i].ki,
                           default_control_config.pressure_control.pid_params[i].kd);

        g_pressure_controllers[i].target_pressure = default_control_config.pressure_control.target_pressures[i];
        g_pressure_controllers[i].max_speed = default_control_config.pressure_control.max_speeds[i];
        g_pressure_controllers[i].enable = false;
        g_pressure_controllers[i].stable_count = 0;
    }

    // 初始化液位控制器
    for (int i = 0; i < 2; i++) {
        g_level_controllers[i].target_level = default_control_config.level_control.target_levels[i];
        g_level_controllers[i].hysteresis = default_control_config.level_control.hysteresis[i];
        g_level_controllers[i].enable = false;
        g_level_controllers[i].valve_state = false;
        g_level_controllers[i].last_action_time = 0;
    }

    return CONTROL_SUCCESS;
}

/**
 * @brief  控制任务主函数
 * @param  pvParameters 任务参数
 * @retval None
 */
void control_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    uint32_t task_counter = 0;
    sensor_data_t sensor_data;
    system_status_t system_status;

    // 等待系统初始化完成
    vTaskDelay(pdMS_TO_TICKS(300));

    // 初始化控制系统
    if (control_manager_init() != CONTROL_SUCCESS) {
        vTaskDelete(NULL);
        return;
    }

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 严格10ms周期执行
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));

        task_counter++;

        // 1. 获取系统状态
        if (safety_get_system_status(&system_status) != SAFETY_SUCCESS) {
            continue; // 无法获取系统状态，跳过本周期
        }

        // 2. 获取传感器数据
        if (sensor_get_all_data(&sensor_data) != SENSOR_SUCCESS) {
            continue; // 无法获取传感器数据，跳过本周期
        }

        // 3. 检查控制使能状态
        if (!g_control_status.control_enabled || system_status.emergency_stop) {
            control_disable_all_outputs();
            continue;
        }

        // 4. 执行温度控制
        control_temperature_loop(&sensor_data);

        // 5. 执行压力控制
        control_pressure_loop(&sensor_data);

        // 6. 执行液位控制
        control_level_loop(&sensor_data);

        // 7. 更新控制统计信息
        control_update_statistics();

        // 8. 每秒执行一次系统效率计算
        if (task_counter % (1000 / CONTROL_TASK_PERIOD_MS) == 0) {
            control_calculate_system_efficiency();
        }

        // 9. 每5秒执行一次控制参数自适应调整
        if (task_counter % (5000 / CONTROL_TASK_PERIOD_MS) == 0) {
            control_adaptive_tuning();
        }

        g_control_status.control_cycle_count++;
    }
}

/**
 * @brief  温度控制循环
 * @param  sensor_data 传感器数据指针
 * @retval None
 */
static void control_temperature_loop(const sensor_data_t *sensor_data)
{
    for (int i = 0; i < 3; i++) {
        temperature_controller_t *ctrl = &g_temp_controllers[i];

        if (!ctrl->enable) {
            // 控制器未使能，关闭加热器
            actuator_set_heater_power(i, 0.0f);
            continue;
        }

        // 更新当前温度
        ctrl->current_temp = sensor_data->temperature[i];

        // 检查传感器故障
        if (sensor_data->fault_status[4 + i]) {
            // 传感器故障，安全关闭加热器
            actuator_set_heater_power(i, 0.0f);
            ctrl->stable_count = 0;
            continue;
        }

        // 执行PID控制
        float error = ctrl->target_temp - ctrl->current_temp;
        float pid_output = pid_controller_update(&ctrl->pid, error, CONTROL_TASK_PERIOD_MS);

        // 输出限制和安全检查
        if (pid_output > ctrl->max_power) {
            pid_output = ctrl->max_power;
        } else if (pid_output < 0.0f) {
            pid_output = 0.0f;
        }

        // 过温保护
        if (ctrl->current_temp > (ctrl->target_temp + 10.0f)) {
            pid_output = 0.0f; // 紧急关闭
        }

        // 更新输出
        ctrl->output_power = pid_output;
        actuator_set_heater_power(i, pid_output);

        // 稳定性统计
        if (fabs(error) < 1.0f) {
            ctrl->stable_count++;
        } else {
            ctrl->stable_count = 0;
        }
    }
}

/**
 * @brief  压力控制循环
 * @param  sensor_data 传感器数据指针
 * @retval None
 */
static void control_pressure_loop(const sensor_data_t *sensor_data)
{
    for (int i = 0; i < 2; i++) {
        pressure_controller_t *ctrl = &g_pressure_controllers[i];

        if (!ctrl->enable) {
            // 控制器未使能，停止泵
            actuator_set_pump_speed(i, 0);
            continue;
        }

        // 更新当前压力
        ctrl->current_pressure = sensor_data->pressure[i];

        // 检查传感器故障
        if (sensor_data->fault_status[2 + i]) {
            // 传感器故障，安全停止泵
            actuator_set_pump_speed(i, 0);
            ctrl->stable_count = 0;
            continue;
        }

        // 执行PID控制
        float error = ctrl->target_pressure - ctrl->current_pressure;
        float pid_output = pid_controller_update(&ctrl->pid, error, CONTROL_TASK_PERIOD_MS);

        // 转换为泵速 (PID输出范围0-100对应最小到最大转速)
        uint16_t pump_speed;
        if (pid_output <= 0.0f) {
            pump_speed = 0;
        } else {
            pump_speed = default_control_config.pressure_control.min_pump_speed +
                        (uint16_t)(pid_output * (ctrl->max_speed - default_control_config.pressure_control.min_pump_speed) / 100.0f);
        }

        // 转速限制
        if (pump_speed > ctrl->max_speed) {
            pump_speed = ctrl->max_speed;
        }

        // 更新输出
        ctrl->pump_speed = pump_speed;
        actuator_set_pump_speed(i, pump_speed);

        // 稳定性统计
        if (fabs(error) < 2.0f) {
            ctrl->stable_count++;
        } else {
            ctrl->stable_count = 0;
        }
    }
}

/**
 * @brief  液位控制循环
 * @param  sensor_data 传感器数据指针
 * @retval None
 */
static void control_level_loop(const sensor_data_t *sensor_data)
{
    uint32_t current_time = xTaskGetTickCount();

    for (int i = 0; i < 2; i++) {
        level_controller_t *ctrl = &g_level_controllers[i];

        if (!ctrl->enable) {
            // 控制器未使能，关闭阀门
            actuator_set_valve_state(i, false);
            ctrl->valve_state = false;
            continue;
        }

        // 更新当前液位
        ctrl->current_level = sensor_data->liquid_level[i];

        // 检查传感器故障
        if (sensor_data->fault_status[i]) {
            // 传感器故障，维持当前阀门状态
            continue;
        }

        // 检查最小动作间隔
        if ((current_time - ctrl->last_action_time) < pdMS_TO_TICKS(default_control_config.level_control.min_action_interval)) {
            continue;
        }

        // 滞回控制逻辑
        bool should_open = false;

        if (!ctrl->valve_state) {
            // 阀门当前关闭，检查是否需要开启
            if (ctrl->current_level < (ctrl->target_level - ctrl->hysteresis)) {
                should_open = true;
            }
        } else {
            // 阀门当前开启，检查是否需要关闭
            if (ctrl->current_level < (ctrl->target_level + ctrl->hysteresis)) {
                should_open = true; // 继续开启
            }
        }

        // 更新阀门状态
        if (should_open != ctrl->valve_state) {
            ctrl->valve_state = should_open;
            actuator_set_valve_state(i, should_open);
            ctrl->last_action_time = current_time;
        }
    }
}

/**
 * @brief  禁用所有控制输出
 * @param  None
 * @retval None
 */
static void control_disable_all_outputs(void)
{
    // 关闭所有加热器
    for (int i = 0; i < 3; i++) {
        actuator_set_heater_power(i, 0.0f);
        g_temp_controllers[i].output_power = 0.0f;
    }

    // 停止所有泵
    for (int i = 0; i < 2; i++) {
        actuator_set_pump_speed(i, 0);
        g_pressure_controllers[i].pump_speed = 0;
    }

    // 关闭所有阀门
    for (int i = 0; i < 2; i++) {
        actuator_set_valve_state(i, false);
        g_level_controllers[i].valve_state = false;
    }
}

/**
 * @brief  更新控制统计信息
 * @param  None
 * @retval None
 */
static void control_update_statistics(void)
{
    // 这里可以添加控制性能统计
    // 例如：稳定时间、超调量、稳态误差等
}

/**
 * @brief  计算系统效率
 * @param  None
 * @retval None
 */
static void control_calculate_system_efficiency(void)
{
    float total_efficiency = 0.0f;
    int active_controllers = 0;

    // 温度控制效率
    for (int i = 0; i < 3; i++) {
        if (g_temp_controllers[i].enable) {
            float error = fabs(g_temp_controllers[i].target_temp - g_temp_controllers[i].current_temp);
            float efficiency = (error < 2.0f) ? 100.0f : (100.0f - error * 10.0f);
            if (efficiency < 0.0f) efficiency = 0.0f;
            total_efficiency += efficiency;
            active_controllers++;
        }
    }

    // 压力控制效率
    for (int i = 0; i < 2; i++) {
        if (g_pressure_controllers[i].enable) {
            float error = fabs(g_pressure_controllers[i].target_pressure - g_pressure_controllers[i].current_pressure);
            float efficiency = (error < 5.0f) ? 100.0f : (100.0f - error * 2.0f);
            if (efficiency < 0.0f) efficiency = 0.0f;
            total_efficiency += efficiency;
            active_controllers++;
        }
    }

    // 更新系统效率
    if (active_controllers > 0) {
        if (xSemaphoreTake(g_control_status.status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_control_status.system_efficiency = total_efficiency / active_controllers;
            xSemaphoreGive(g_control_status.status_mutex);
        }
    }
}

/**
 * @brief  自适应参数调整
 * @param  None
 * @retval None
 */
static void control_adaptive_tuning(void)
{
    // 根据控制性能自动调整PID参数
    // 这里可以实现简化的自适应算法

    for (int i = 0; i < 3; i++) {
        temperature_controller_t *ctrl = &g_temp_controllers[i];
        if (ctrl->enable && ctrl->stable_count > 100) {
            // 系统稳定时间较长，可以略微增加积分增益
            float ki = ctrl->pid.ki * 1.01f;
            if (ki < 0.2f) {
                ctrl->pid.ki = ki;
            }
        }
    }
}

/**
 * @brief  启用控制系统
 * @param  None
 * @retval control_result_t 执行结果
 */
control_result_t control_enable_system(void)
{
    if (xSemaphoreTake(g_control_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_control_status.control_enabled = true;
        xSemaphoreGive(g_control_status.status_mutex);
        return CONTROL_SUCCESS;
    }
    return CONTROL_ERROR_TIMEOUT;
}

/**
 * @brief  禁用控制系统
 * @param  None
 * @retval control_result_t 执行结果
 */
control_result_t control_disable_system(void)
{
    if (xSemaphoreTake(g_control_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_control_status.control_enabled = false;
        xSemaphoreGive(g_control_status.status_mutex);

        // 安全关闭所有输出
        control_disable_all_outputs();

        return CONTROL_SUCCESS;
    }
    return CONTROL_ERROR_TIMEOUT;
}

/**
 * @brief  设置温度控制目标
 * @param  controller_id 控制器ID (0-2)
 * @param  target_temp 目标温度
 * @retval control_result_t 设置结果
 */
control_result_t control_set_temperature_target(uint8_t controller_id, float target_temp)
{
    if (controller_id >= 3) {
        return CONTROL_ERROR_INVALID_PARAMETER;
    }

    if (target_temp < 0.0f || target_temp > 100.0f) {
        return CONTROL_ERROR_INVALID_PARAMETER;
    }

    g_temp_controllers[controller_id].target_temp = target_temp;
    return CONTROL_SUCCESS;
}

/**
 * @brief  设置压力控制目标
 * @param  controller_id 控制器ID (0-1)
 * @param  target_pressure 目标压力
 * @retval control_result_t 设置结果
 */
control_result_t control_set_pressure_target(uint8_t controller_id, float target_pressure)
{
    if (controller_id >= 2) {
        return CONTROL_ERROR_INVALID_PARAMETER;
    }

    if (target_pressure < 0.0f || target_pressure > 200.0f) {
        return CONTROL_ERROR_INVALID_PARAMETER;
    }

    g_pressure_controllers[controller_id].target_pressure = target_pressure;
    return CONTROL_SUCCESS;
}

/**
 * @brief  获取控制系统状态
 * @param  status 状态结构指针
 * @retval control_result_t 获取结果
 */
control_result_t control_get_system_status(control_system_status_info_t *status)
{
    if (status == NULL) {
        return CONTROL_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_control_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status->control_enabled = g_control_status.control_enabled;
        status->auto_mode = g_control_status.auto_mode;
        status->control_mode = g_control_status.control_mode;
        status->control_cycle_count = g_control_status.control_cycle_count;
        status->system_efficiency = g_control_status.system_efficiency;

        // 复制控制器状态
        for (int i = 0; i < 3; i++) {
            status->temperature_status[i].target = g_temp_controllers[i].target_temp;
            status->temperature_status[i].current = g_temp_controllers[i].current_temp;
            status->temperature_status[i].output = g_temp_controllers[i].output_power;
            status->temperature_status[i].enabled = g_temp_controllers[i].enable;
        }

        for (int i = 0; i < 2; i++) {
            status->pressure_status[i].target = g_pressure_controllers[i].target_pressure;
            status->pressure_status[i].current = g_pressure_controllers[i].current_pressure;
            status->pressure_status[i].output = g_pressure_controllers[i].pump_speed;
            status->pressure_status[i].enabled = g_pressure_controllers[i].enable;
        }

        xSemaphoreGive(g_control_status.status_mutex);
        return CONTROL_SUCCESS;
    }

    return CONTROL_ERROR_TIMEOUT;
}