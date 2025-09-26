/**
 * @file    hp10my_driver.c
 * @brief   HP10MY压力传感器驱动实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "hp10my_driver.h"
#include "adc_hal.h"
#include "digital_filter.h"
#include "system_config.h"

// 传感器配置参数
typedef struct {
    adc_channel_id_t adc_channel;   // ADC通道
    float voltage_min;              // 最小电压 (4mA对应)
    float voltage_max;              // 最大电压 (20mA对应)
    float pressure_min;             // 测量范围最小值 (kPa)
    float pressure_max;             // 测量范围最大值 (kPa)
    float calibration_offset;       // 校准偏移
    float calibration_scale;        // 校准比例
} hp10my_config_t;

// 传感器实例配置
static const hp10my_config_t hp10my_configs[2] = {
    // 压力传感器1 (0-100kPa)
    {
        .adc_channel = ADC_CH_PRESSURE_1,
        .voltage_min = 0.8f,        // 4mA × 200Ω = 0.8V
        .voltage_max = 4.0f,        // 20mA × 200Ω = 4.0V
        .pressure_min = 0.0f,       // 0 kPa
        .pressure_max = 100.0f,     // 100 kPa
        .calibration_offset = 0.0f,
        .calibration_scale = 1.0f
    },
    // 压力传感器2 (0-150kPa)
    {
        .adc_channel = ADC_CH_PRESSURE_2,
        .voltage_min = 0.8f,
        .voltage_max = 4.0f,
        .pressure_min = 0.0f,
        .pressure_max = 150.0f,
        .calibration_offset = 0.0f,
        .calibration_scale = 1.0f
    }
};

// 传感器状态
static hp10my_status_t sensor_status[2];

// 滤波器实例
static lowpass_filter_t pressure_filters[2];

// 故障检测阈值
#define SENSOR_FAULT_VOLTAGE_MIN    0.4f    // 低于0.4V认为故障
#define SENSOR_FAULT_VOLTAGE_MAX    4.5f    // 高于4.5V认为故障
#define SENSOR_FAULT_COUNT_MAX      5       // 连续5次故障认为传感器故障

// 故障计数器
static uint8_t fault_counters[2] = {0, 0};

/**
 * @brief  HP10MY传感器初始化
 * @param  None
 * @retval None
 */
void hp10my_init(void)
{
    // 初始化ADC通道
    for (int i = 0; i < 2; i++) {
        adc_hal_config_channel(hp10my_configs[i].adc_channel, ADC_12BIT);

        // 初始化滤波器 (截止频率0.2，适合压力传感器)
        lowpass_filter_init(&pressure_filters[i], 0.2f);

        // 初始化传感器状态
        sensor_status[i].pressure_kpa = 0.0f;
        sensor_status[i].voltage = 0.0f;
        sensor_status[i].fault = false;
        sensor_status[i].initialized = true;

        // 重置故障计数器
        fault_counters[i] = 0;
    }
}

/**
 * @brief  读取压力传感器值
 * @param  sensor_id 传感器ID (0-1)
 * @retval float 压力值 (kPa)
 */
float read_pressure_hp10my(uint8_t sensor_id)
{
    if (sensor_id >= 2) {
        return 0.0f;
    }

    const hp10my_config_t *config = &hp10my_configs[sensor_id];

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
        return sensor_status[sensor_id].pressure_kpa;
    }

    // 滤波处理
    voltage = lowpass_filter_update(&pressure_filters[sensor_id], voltage);

    // 电压到压力转换
    float pressure_kpa;
    if (voltage <= config->voltage_min) {
        pressure_kpa = config->pressure_min;
    } else if (voltage >= config->voltage_max) {
        pressure_kpa = config->pressure_max;
    } else {
        // 线性插值
        float voltage_range = config->voltage_max - config->voltage_min;
        float pressure_range = config->pressure_max - config->pressure_min;
        pressure_kpa = config->pressure_min +
                      ((voltage - config->voltage_min) / voltage_range) * pressure_range;
    }

    // 应用校准参数
    pressure_kpa = (pressure_kpa + config->calibration_offset) * config->calibration_scale;

    // 限制范围
    if (pressure_kpa < 0.0f) pressure_kpa = 0.0f;
    if (pressure_kpa > config->pressure_max * 1.1f) pressure_kpa = config->pressure_max * 1.1f;

    // 更新状态
    sensor_status[sensor_id].pressure_kpa = pressure_kpa;
    sensor_status[sensor_id].voltage = voltage;

    return pressure_kpa;
}

/**
 * @brief  获取传感器状态
 * @param  sensor_id 传感器ID (0-1)
 * @retval hp10my_status_t 传感器状态
 */
hp10my_status_t hp10my_get_status(uint8_t sensor_id)
{
    hp10my_status_t status = {0};

    if (sensor_id < 2) {
        status = sensor_status[sensor_id];
    }

    return status;
}

/**
 * @brief  校准传感器
 * @param  sensor_id 传感器ID (0-1)
 * @param  actual_pressure 实际压力值 (kPa)
 * @retval bool 校准成功返回true
 */
bool hp10my_calibrate(uint8_t sensor_id, float actual_pressure)
{
    if (sensor_id >= 2 || sensor_status[sensor_id].fault) {
        return false;
    }

    // 读取当前传感器值
    float measured_pressure = read_pressure_hp10my(sensor_id);

    // 计算校准参数 (简化的单点校准)
    if (measured_pressure != 0.0f) {
        hp10my_config_t *config = (hp10my_config_t *)&hp10my_configs[sensor_id];
        config->calibration_scale = actual_pressure / measured_pressure;
    } else if (actual_pressure == 0.0f) {
        // 零点校准
        hp10my_config_t *config = (hp10my_config_t *)&hp10my_configs[sensor_id];
        config->calibration_offset = -measured_pressure;
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
bool hp10my_set_calibration(uint8_t sensor_id, float offset, float scale)
{
    if (sensor_id >= 2) {
        return false;
    }

    hp10my_config_t *config = (hp10my_config_t *)&hp10my_configs[sensor_id];
    config->calibration_offset = offset;
    config->calibration_scale = scale;

    return true;
}

/**
 * @brief  检查传感器连接状态
 * @param  sensor_id 传感器ID (0-1)
 * @retval bool 连接正常返回true
 */
bool hp10my_check_connection(uint8_t sensor_id)
{
    if (sensor_id >= 2) {
        return false;
    }

    // 读取ADC值
    uint16_t adc_value = adc_hal_read_channel(hp10my_configs[sensor_id].adc_channel);
    float voltage = adc_hal_convert_to_voltage(adc_value, ADC_12BIT, 3.3f);

    // 检查电压是否在正常范围内
    return (voltage >= SENSOR_FAULT_VOLTAGE_MIN && voltage <= SENSOR_FAULT_VOLTAGE_MAX);
}

/**
 * @brief  获取传感器电压值
 * @param  sensor_id 传感器ID (0-1)
 * @retval float 电压值 (V)
 */
float hp10my_get_voltage(uint8_t sensor_id)
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
void hp10my_reset_fault(uint8_t sensor_id)
{
    if (sensor_id < 2) {
        sensor_status[sensor_id].fault = false;
        fault_counters[sensor_id] = 0;

        // 重置滤波器
        lowpass_filter_reset(&pressure_filters[sensor_id]);
    }
}

/**
 * @brief  转换压力单位
 * @param  pressure_kpa 压力值 (kPa)
 * @param  target_unit 目标单位 (0:kPa, 1:bar, 2:psi)
 * @retval float 转换后的压力值
 */
float hp10my_convert_pressure_unit(float pressure_kpa, uint8_t target_unit)
{
    switch (target_unit) {
        case 0: // kPa
            return pressure_kpa;
        case 1: // bar
            return pressure_kpa / 100.0f;
        case 2: // psi
            return pressure_kpa * 0.145038f;
        default:
            return pressure_kpa;
    }
}