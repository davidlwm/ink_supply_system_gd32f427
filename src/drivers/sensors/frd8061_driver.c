/**
 * @file    frd8061_driver.c
 * @brief   FRD-8061液位传感器驱动实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "frd8061_driver.h"
#include "adc_hal.h"
#include "digital_filter.h"
#include "system_config.h"

// 传感器配置参数
typedef struct {
    adc_channel_id_t adc_channel;   // ADC通道
    float voltage_min;              // 最小电压 (4mA对应)
    float voltage_max;              // 最大电压 (20mA对应)
    float range_min;                // 测量范围最小值 (%)
    float range_max;                // 测量范围最大值 (%)
    float calibration_offset;       // 校准偏移
    float calibration_scale;        // 校准比例
} frd8061_config_t;

// 传感器实例配置
static const frd8061_config_t frd8061_configs[2] = {
    // 液位传感器1
    {
        .adc_channel = ADC_CH_LIQUID_LEVEL_1,
        .voltage_min = 0.8f,        // 4mA × 200Ω = 0.8V
        .voltage_max = 4.0f,        // 20mA × 200Ω = 4.0V
        .range_min = 0.0f,          // 0%
        .range_max = 100.0f,        // 100%
        .calibration_offset = 0.0f,
        .calibration_scale = 1.0f
    },
    // 液位传感器2
    {
        .adc_channel = ADC_CH_LIQUID_LEVEL_2,
        .voltage_min = 0.8f,
        .voltage_max = 4.0f,
        .range_min = 0.0f,
        .range_max = 100.0f,
        .calibration_offset = 0.0f,
        .calibration_scale = 1.0f
    }
};

// 传感器状态
static frd8061_status_t sensor_status[2];

// 滤波器实例
static lowpass_filter_t level_filters[2];

// 故障检测阈值
#define SENSOR_FAULT_VOLTAGE_MIN    0.4f    // 低于0.4V认为故障
#define SENSOR_FAULT_VOLTAGE_MAX    4.5f    // 高于4.5V认为故障
#define SENSOR_FAULT_COUNT_MAX      5       // 连续5次故障认为传感器故障

// 故障计数器
static uint8_t fault_counters[2] = {0, 0};

/**
 * @brief  FRD-8061传感器初始化
 * @param  None
 * @retval None
 */
void frd8061_init(void)
{
    // 初始化ADC通道
    for (int i = 0; i < 2; i++) {
        adc_hal_config_channel(frd8061_configs[i].adc_channel, ADC_12BIT);

        // 初始化滤波器 (截止频率0.1，适合液位传感器)
        lowpass_filter_init(&level_filters[i], 0.1f);

        // 初始化传感器状态
        sensor_status[i].level_percent = 0.0f;
        sensor_status[i].voltage = 0.0f;
        sensor_status[i].fault = false;
        sensor_status[i].initialized = true;

        // 重置故障计数器
        fault_counters[i] = 0;
    }
}

/**
 * @brief  读取液位传感器原始值
 * @param  sensor_id 传感器ID (0-1)
 * @retval float 液位百分比 (0-100%)
 */
float read_liquid_level_frd8061(uint8_t sensor_id)
{
    if (sensor_id >= 2) {
        return 0.0f;
    }

    const frd8061_config_t *config = &frd8061_configs[sensor_id];

    // 读取ADC值
    uint16_t adc_value = adc_hal_read_channel(config->adc_channel);

    // 转换为电压
    float voltage = adc_hal_convert_to_voltage(adc_value, ADC_12BIT, 3.3f);

    // 故障检测
    bool fault_detected = false;
    if (voltage < SENSOR_FAULT_VOLTAGE_MIN || voltage > SENSOR_FAULT_VOLTAGE_MAX) {
        fault_counters[sensor_id]++;
        if (fault_counters[sensor_id] >= SENSOR_FAULT_COUNT_MAX) {
            sensor_status[sensor_id].fault = true;
            fault_detected = true;
        }
    } else {
        fault_counters[sensor_id] = 0;
        sensor_status[sensor_id].fault = false;
    }

    // 如果故障，返回上次有效值
    if (fault_detected) {
        return sensor_status[sensor_id].level_percent;
    }

    // 滤波处理
    voltage = lowpass_filter_update(&level_filters[sensor_id], voltage);

    // 电压到液位百分比转换
    float level_percent;
    if (voltage <= config->voltage_min) {
        level_percent = config->range_min;
    } else if (voltage >= config->voltage_max) {
        level_percent = config->range_max;
    } else {
        // 线性插值
        float voltage_range = config->voltage_max - config->voltage_min;
        float level_range = config->range_max - config->range_min;
        level_percent = config->range_min +
                       ((voltage - config->voltage_min) / voltage_range) * level_range;
    }

    // 应用校准参数
    level_percent = (level_percent + config->calibration_offset) * config->calibration_scale;

    // 限制范围
    if (level_percent < 0.0f) level_percent = 0.0f;
    if (level_percent > 100.0f) level_percent = 100.0f;

    // 更新状态
    sensor_status[sensor_id].level_percent = level_percent;
    sensor_status[sensor_id].voltage = voltage;

    return level_percent;
}

/**
 * @brief  获取传感器状态
 * @param  sensor_id 传感器ID (0-1)
 * @retval frd8061_status_t 传感器状态
 */
frd8061_status_t frd8061_get_status(uint8_t sensor_id)
{
    frd8061_status_t status = {0};

    if (sensor_id < 2) {
        status = sensor_status[sensor_id];
    }

    return status;
}

/**
 * @brief  校准传感器
 * @param  sensor_id 传感器ID (0-1)
 * @param  actual_level 实际液位值 (%)
 * @retval bool 校准成功返回true
 */
bool frd8061_calibrate(uint8_t sensor_id, float actual_level)
{
    if (sensor_id >= 2 || sensor_status[sensor_id].fault) {
        return false;
    }

    // 读取当前传感器值
    float measured_level = read_liquid_level_frd8061(sensor_id);

    // 计算校准参数 (简化的单点校准)
    if (measured_level != 0.0f) {
        frd8061_config_t *config = (frd8061_config_t *)&frd8061_configs[sensor_id];
        config->calibration_scale = actual_level / measured_level;
    }

    return true;
}

/**
 * @brief  设置传感器校准参数
 * @param  sensor_id 传感器ID (0-1)
 * @param  offset 偏移量
 * @param  scale 比例系数
 * @retval bool 设置成功返回true
 */
bool frd8061_set_calibration(uint8_t sensor_id, float offset, float scale)
{
    if (sensor_id >= 2) {
        return false;
    }

    frd8061_config_t *config = (frd8061_config_t *)&frd8061_configs[sensor_id];
    config->calibration_offset = offset;
    config->calibration_scale = scale;

    return true;
}

/**
 * @brief  检查传感器连接状态
 * @param  sensor_id 传感器ID (0-1)
 * @retval bool 连接正常返回true
 */
bool frd8061_check_connection(uint8_t sensor_id)
{
    if (sensor_id >= 2) {
        return false;
    }

    // 读取ADC值
    uint16_t adc_value = adc_hal_read_channel(frd8061_configs[sensor_id].adc_channel);
    float voltage = adc_hal_convert_to_voltage(adc_value, ADC_12BIT, 3.3f);

    // 检查电压是否在正常范围内
    return (voltage >= SENSOR_FAULT_VOLTAGE_MIN && voltage <= SENSOR_FAULT_VOLTAGE_MAX);
}

/**
 * @brief  获取传感器电压值
 * @param  sensor_id 传感器ID (0-1)
 * @retval float 电压值 (V)
 */
float frd8061_get_voltage(uint8_t sensor_id)
{
    if (sensor_id >= 2) {
        return 0.0f;
    }

    return sensor_status[sensor_id].voltage;
}

/**
 * @brief  重置传感器故障状态
 * @param  sensor_id 传感器ID (0-1)
 * @retval None
 */
void frd8061_reset_fault(uint8_t sensor_id)
{
    if (sensor_id < 2) {
        sensor_status[sensor_id].fault = false;
        fault_counters[sensor_id] = 0;

        // 重置滤波器
        lowpass_filter_reset(&level_filters[sensor_id]);
    }
}