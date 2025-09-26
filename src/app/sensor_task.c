/**
 * @file    sensor_task.c
 * @brief   传感器任务实现 - 8周v4标准
 * @version V4.0
 * @date    2024-12-27
 */

#include "app/sensor_task.h"

// 传感器驱动 (使用drivers层)
#include "drivers/sensors/frd8061_driver.h"
#include "drivers/sensors/hp10my_driver.h"
#include "drivers/sensors/ftt518_driver.h"

// 中间件 (8周v4标准)
#include "middleware/filter.h"
#include "middleware/tasks.h"

// HAL层
#include "hal/adc_hal.h"
#include "hal/gpio_hal.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// 内部传感器数据结构 (基于v1设计)
typedef struct {
    // FRD-8061液位传感器数据 (保持v1配置)
    struct {
        float raw_value;        // 原始值 (mm)
        float filtered_value;   // 滤波值 (mm)
        float calibrated_value; // 校准值 (mm)
        uint32_t timestamp;     // 时间戳
        bool fault_status;      // 故障状态
        uint16_t fault_code;    // 故障代码
    } liquid_level[LIQUID_LEVEL_SENSOR_COUNT];

    // HP10MY压力传感器数据 (保持v1配置)
    struct {
        float raw_value;        // 原始值 (kPa)
        float filtered_value;   // 滤波值 (kPa)
        float calibrated_value; // 校准值 (kPa)
        uint32_t timestamp;     // 时间戳
        bool fault_status;      // 故障状态
        uint16_t fault_code;    // 故障代码
    } pressure[PRESSURE_SENSOR_COUNT];

    // FTT518温度传感器数据 (保持v1配置)
    struct {
        float raw_value;        // 原始值 (°C)
        float filtered_value;   // 滤波值 (°C)
        float calibrated_value; // 校准值 (°C)
        float resistance;       // PT100电阻值 (Ω)
        uint32_t timestamp;     // 时间戳
        bool fault_status;      // 故障状态
        uint16_t fault_code;    // 故障代码
    } temperature[TEMPERATURE_SENSOR_COUNT];

} internal_sensor_data_t;

// 全局传感器数据 (与v1接口兼容)
static internal_sensor_data_t g_sensor_data;
static SemaphoreHandle_t sensor_data_mutex;

/**
 * @brief  传感器管理器初始化
 * @param  None
 * @retval None
 */
void sensor_manager_init(void)
{
    // 创建传感器数据互斥锁
    sensor_data_mutex = xSemaphoreCreateMutex();

    // 初始化传感器硬件
    frd8061_init();         // 液位传感器初始化
    hp10my_init();          // 压力传感器初始化
    ftt518_init();          // 温度传感器初始化

    // 初始化滤波器
    sensor_filter_init();

    // 初始化校准参数
    sensor_calibration_init();

    // 清零传感器数据
    memset(&g_sensor_data, 0, sizeof(internal_sensor_data_t));
}

/**
 * @brief  传感器任务主函数 (保持v1设计模式)
 * @param  pvParameters 任务参数
 * @retval None
 */
void sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    uint32_t task_counter = 0;

    // 任务初始化
    sensor_task_init();

    // 初始化时间基准
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 严格10ms周期执行
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_SAMPLE_PERIOD_MS));

        task_counter++;

        // 获取传感器数据互斥锁
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {

            // 1. 液位传感器处理 (保持v1算法)
            for (int i = 0; i < LIQUID_LEVEL_SENSOR_COUNT; i++) {
                // 读取原始数据 (保持v1接口)
                float raw_level = read_liquid_level_frd8061(i);
                g_sensor_data.liquid_level[i].raw_value = raw_level;

                // 故障检测
                if (raw_level < 0) {
                    g_sensor_data.liquid_level[i].fault_status = true;
                    g_sensor_data.liquid_level[i].fault_code = FAULT_LIQUID_LEVEL_1_FAULT + i;
                } else {
                    g_sensor_data.liquid_level[i].fault_status = false;
                    g_sensor_data.liquid_level[i].fault_code = 0;

                    // 数字滤波处理
                    float filtered_level = sensor_filter_process(FILTER_LIQUID_LEVEL_1 + i, raw_level);
                    g_sensor_data.liquid_level[i].filtered_value = filtered_level;

                    // 校准处理
                    float calibrated_level = sensor_calibration_apply(SENSOR_LIQUID_LEVEL_1 + i, filtered_level);
                    g_sensor_data.liquid_level[i].calibrated_value = calibrated_level;
                }

                g_sensor_data.liquid_level[i].timestamp = xTaskGetTickCount();
            }

            // 2. 压力传感器处理 (保持v1算法)
            for (int i = 0; i < PRESSURE_SENSOR_COUNT; i++) {
                // 读取原始数据 (保持v1接口)
                float raw_pressure = read_pressure_hp10my(i);
                g_sensor_data.pressure[i].raw_value = raw_pressure;

                // 故障检测
                if (raw_pressure < -900) {
                    g_sensor_data.pressure[i].fault_status = true;
                    g_sensor_data.pressure[i].fault_code = FAULT_PRESSURE_SENSOR_1_FAULT + i;
                } else {
                    g_sensor_data.pressure[i].fault_status = false;
                    g_sensor_data.pressure[i].fault_code = 0;

                    // 数字滤波处理
                    float filtered_pressure = sensor_filter_process(FILTER_PRESSURE_1 + i, raw_pressure);
                    g_sensor_data.pressure[i].filtered_value = filtered_pressure;

                    // 校准处理
                    float calibrated_pressure = sensor_calibration_apply(SENSOR_PRESSURE_1 + i, filtered_pressure);
                    g_sensor_data.pressure[i].calibrated_value = calibrated_pressure;
                }

                g_sensor_data.pressure[i].timestamp = xTaskGetTickCount();
            }

            // 3. 温度传感器处理 (保持v1算法)
            for (int i = 0; i < TEMPERATURE_SENSOR_COUNT; i++) {
                // 读取原始数据 (保持v1接口)
                float raw_temperature = read_temperature_ftt518_pt100(i);
                g_sensor_data.temperature[i].raw_value = raw_temperature;

                // 读取PT100电阻值 (用于故障诊断)
                float pt100_resistance = get_pt100_resistance_value(i);
                g_sensor_data.temperature[i].resistance = pt100_resistance;

                // 故障检测 (PT100开路/短路检测)
                if (pt100_resistance < 50.0f || pt100_resistance > 500.0f) {
                    g_sensor_data.temperature[i].fault_status = true;
                    if (pt100_resistance < 50.0f) {
                        g_sensor_data.temperature[i].fault_code = FAULT_TEMP_SENSOR_1_SHORT + i;
                    } else {
                        g_sensor_data.temperature[i].fault_code = FAULT_TEMP_SENSOR_1_OPEN + i;
                    }
                } else {
                    g_sensor_data.temperature[i].fault_status = false;
                    g_sensor_data.temperature[i].fault_code = 0;

                    // 数字滤波处理
                    float filtered_temperature = sensor_filter_process(FILTER_TEMPERATURE_1 + i, raw_temperature);
                    g_sensor_data.temperature[i].filtered_value = filtered_temperature;

                    // 校准处理
                    float calibrated_temperature = sensor_calibration_apply(SENSOR_TEMPERATURE_1 + i, filtered_temperature);
                    g_sensor_data.temperature[i].calibrated_value = calibrated_temperature;
                }

                g_sensor_data.temperature[i].timestamp = xTaskGetTickCount();
            }

            // 释放互斥锁
            xSemaphoreGive(sensor_data_mutex);
        }

        // 性能监控 (每1000次报告一次)
        if (task_counter % 1000 == 0) {
            report_sensor_performance(task_counter);
        }
    }
}

/**
 * @brief  传感器任务初始化
 * @param  None
 * @retval None
 */
static void sensor_task_init(void)
{
    // 传感器硬件初始化
    sensor_manager_init();

    // 传感器自检
    sensor_self_test();
}

// 公共接口函数 (保持v1接口兼容)
float get_liquid_level_sensor(uint8_t sensor_id)
{
    if (sensor_id >= LIQUID_LEVEL_SENSOR_COUNT) return 0.0f;

    float value = 0.0f;
    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        value = g_sensor_data.liquid_level[sensor_id].calibrated_value;
        xSemaphoreGive(sensor_data_mutex);
    }
    return value;
}

float get_pressure_sensor(uint8_t sensor_id)
{
    if (sensor_id >= PRESSURE_SENSOR_COUNT) return 0.0f;

    float value = 0.0f;
    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        value = g_sensor_data.pressure[sensor_id].calibrated_value;
        xSemaphoreGive(sensor_data_mutex);
    }
    return value;
}

float get_pt100_temperature(uint8_t sensor_id)
{
    if (sensor_id >= TEMPERATURE_SENSOR_COUNT) return 0.0f;

    float value = 0.0f;
    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        value = g_sensor_data.temperature[sensor_id].calibrated_value;
        xSemaphoreGive(sensor_data_mutex);
    }
    return value;
}

bool get_sensor_fault_status(uint8_t sensor_type, uint8_t sensor_id)
{
    bool fault = true;

    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        switch (sensor_type) {
            case SENSOR_TYPE_LIQUID_LEVEL:
                if (sensor_id < LIQUID_LEVEL_SENSOR_COUNT) {
                    fault = g_sensor_data.liquid_level[sensor_id].fault_status;
                }
                break;
            case SENSOR_TYPE_PRESSURE:
                if (sensor_id < PRESSURE_SENSOR_COUNT) {
                    fault = g_sensor_data.pressure[sensor_id].fault_status;
                }
                break;
            case SENSOR_TYPE_TEMPERATURE:
                if (sensor_id < TEMPERATURE_SENSOR_COUNT) {
                    fault = g_sensor_data.temperature[sensor_id].fault_status;
                }
                break;
        }
        xSemaphoreGive(sensor_data_mutex);
    }

    return fault;
}

/**
 * @brief  获取所有传感器数据 (控制任务兼容接口)
 * @param  data 传感器数据指针
 * @retval app_result_t 获取结果
 */
app_result_t sensor_get_all_data(sensor_data_t* data)
{
    if (data == NULL) {
        return APP_RESULT_INVALID_PARAM;
    }

    if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // 复制液位传感器数据
        for (int i = 0; i < LIQUID_LEVEL_SENSOR_COUNT; i++) {
            data->liquid_level[i] = g_sensor_data.liquid_level[i].calibrated_value;
        }

        // 复制压力传感器数据
        for (int i = 0; i < PRESSURE_SENSOR_COUNT; i++) {
            data->pressure[i] = g_sensor_data.pressure[i].calibrated_value;
        }

        // 复制温度传感器数据
        for (int i = 0; i < TEMPERATURE_SENSOR_COUNT; i++) {
            data->temperature[i] = g_sensor_data.temperature[i].calibrated_value;
        }

        // 复制故障状态
        int fault_index = 0;
        for (int i = 0; i < LIQUID_LEVEL_SENSOR_COUNT; i++) {
            data->fault_status[fault_index++] = g_sensor_data.liquid_level[i].fault_status;
        }
        for (int i = 0; i < PRESSURE_SENSOR_COUNT; i++) {
            data->fault_status[fault_index++] = g_sensor_data.pressure[i].fault_status;
        }
        for (int i = 0; i < TEMPERATURE_SENSOR_COUNT; i++) {
            data->fault_status[fault_index++] = g_sensor_data.temperature[i].fault_status;
        }

        data->timestamp = xTaskGetTickCount();

        xSemaphoreGive(sensor_data_mutex);
        return APP_RESULT_OK;
    }

    return APP_RESULT_TIMEOUT;
}