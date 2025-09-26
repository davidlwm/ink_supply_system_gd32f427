/**
 * @file    ftt518_driver.c
 * @brief   FTT518 PT100温度传感器驱动实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "ftt518_driver.h"
#include "adc_hal.h"
#include "digital_filter.h"
#include "system_config.h"
#include <math.h>

// PT100特性参数
#define PT100_R0            100.0f      // 0°C时的电阻值
#define PT100_ALPHA         0.00385f    // 温度系数
#define REFERENCE_RESISTOR  1000.0f     // 参考电阻值

// 传感器配置参数
typedef struct {
    adc_channel_id_t signal_channel;   // 信号ADC通道
    adc_channel_id_t ref_channel;      // 参考ADC通道
    float calibration_offset;          // 校准偏移
    float calibration_scale;           // 校准比例
    float temp_min;                    // 测量范围最小值
    float temp_max;                    // 测量范围最大值
} ftt518_config_t;

// 传感器实例配置
static const ftt518_config_t ftt518_configs[3] = {
    // 温度传感器1
    {
        .signal_channel = ADC_CH_TEMP_1_SIGNAL,
        .ref_channel = ADC_CH_TEMP_1_REF,
        .calibration_offset = 0.0f,
        .calibration_scale = 1.0f,
        .temp_min = -10.0f,
        .temp_max = 100.0f
    },
    // 温度传感器2
    {
        .signal_channel = ADC_CH_TEMP_2_SIGNAL,
        .ref_channel = ADC_CH_TEMP_2_REF,
        .calibration_offset = 0.0f,
        .calibration_scale = 1.0f,
        .temp_min = -10.0f,
        .temp_max = 100.0f
    },
    // 温度传感器3
    {
        .signal_channel = ADC_CH_TEMP_3_SIGNAL,
        .ref_channel = ADC_CH_TEMP_3_REF,
        .calibration_offset = 0.0f,
        .calibration_scale = 1.0f,
        .temp_min = -10.0f,
        .temp_max = 100.0f
    }
};

// 传感器状态
static ftt518_status_t sensor_status[3];

// 滤波器实例
static lowpass_filter_t temp_filters[3];

// 故障检测阈值
#define RESISTANCE_FAULT_MIN    50.0f     // 最小有效电阻值
#define RESISTANCE_FAULT_MAX    200.0f    // 最大有效电阻值
#define SENSOR_FAULT_COUNT_MAX  5         // 连续5次故障认为传感器故障

// 故障计数器
static uint8_t fault_counters[3] = {0, 0, 0};

/**
 * @brief  FTT518传感器初始化
 * @param  None
 * @retval None
 */
void ftt518_init(void)
{
    // 初始化ADC通道
    for (int i = 0; i < 3; i++) {
        adc_hal_config_channel(ftt518_configs[i].signal_channel, ADC_12BIT);
        adc_hal_config_channel(ftt518_configs[i].ref_channel, ADC_12BIT);

        // 初始化滤波器 (截止频率0.05，适合温度传感器)
        lowpass_filter_init(&temp_filters[i], 0.05f);

        // 初始化传感器状态
        sensor_status[i].temperature = 0.0f;
        sensor_status[i].resistance = PT100_R0;
        sensor_status[i].fault = false;
        sensor_status[i].open_circuit = false;
        sensor_status[i].short_circuit = false;
        sensor_status[i].initialized = true;

        // 重置故障计数器
        fault_counters[i] = 0;
    }
}

/**
 * @brief  读取PT100温度传感器值
 * @param  sensor_id 传感器ID (0-2)
 * @retval float 温度值 (°C)
 */
float read_temperature_ftt518_pt100(uint8_t sensor_id)
{
    if (sensor_id >= 3) {
        return 0.0f;
    }

    const ftt518_config_t *config = &ftt518_configs[sensor_id];

    // 读取信号和参考ADC值
    uint16_t signal_adc = adc_hal_read_channel(config->signal_channel);
    uint16_t ref_adc = adc_hal_read_channel(config->ref_channel);

    // 转换为电压
    float signal_voltage = adc_hal_convert_to_voltage(signal_adc, ADC_12BIT, 3.3f);
    float ref_voltage = adc_hal_convert_to_voltage(ref_adc, ADC_12BIT, 3.3f);

    // 计算PT100电阻值 (三线制补偿)
    float resistance = get_pt100_resistance_value(sensor_id);

    // 故障检测
    bool fault_detected = false;
    sensor_status[sensor_id].open_circuit = false;
    sensor_status[sensor_id].short_circuit = false;

    if (resistance < RESISTANCE_FAULT_MIN) {
        sensor_status[sensor_id].short_circuit = true;
        fault_detected = true;
    } else if (resistance > RESISTANCE_FAULT_MAX) {
        sensor_status[sensor_id].open_circuit = true;
        fault_detected = true;
    }

    if (fault_detected) {
        fault_counters[sensor_id]++;
        if (fault_counters[sensor_id] >= SENSOR_FAULT_COUNT_MAX) {
            sensor_status[sensor_id].fault = true;
        }
        return sensor_status[sensor_id].temperature; // 返回上次有效值
    } else {
        fault_counters[sensor_id] = 0;
        sensor_status[sensor_id].fault = false;
    }

    // 滤波处理
    resistance = lowpass_filter_update(&temp_filters[sensor_id], resistance);

    // PT100电阻-温度转换 (简化线性公式)
    float temperature = (resistance - PT100_R0) / (PT100_R0 * PT100_ALPHA);

    // 更精确的PT100转换 (Callendar-Van Dusen方程简化)
    if (temperature >= 0.0f) {
        // 正温度范围
        temperature = (-PT100_R0 * PT100_ALPHA +
                      sqrtf(PT100_R0 * PT100_R0 * PT100_ALPHA * PT100_ALPHA -
                           4.0f * PT100_R0 * (-1.5e-7f) * (PT100_R0 - resistance))) /
                     (2.0f * PT100_R0 * (-1.5e-7f));
    } else {
        // 负温度范围 (简化处理)
        temperature = (resistance - PT100_R0) / (PT100_R0 * PT100_ALPHA);
    }

    // 应用校准参数
    temperature = (temperature + config->calibration_offset) * config->calibration_scale;

    // 限制范围
    if (temperature < config->temp_min) temperature = config->temp_min;
    if (temperature > config->temp_max) temperature = config->temp_max;

    // 更新状态
    sensor_status[sensor_id].temperature = temperature;
    sensor_status[sensor_id].resistance = resistance;

    return temperature;
}

/**
 * @brief  获取PT100电阻值
 * @param  sensor_id 传感器ID (0-2)
 * @retval float 电阻值 (Ω)
 */
float get_pt100_resistance_value(uint8_t sensor_id)
{
    if (sensor_id >= 3) {
        return PT100_R0;
    }

    const ftt518_config_t *config = &ftt518_configs[sensor_id];

    // 读取信号和参考ADC值
    uint16_t signal_adc = adc_hal_read_channel(config->signal_channel);
    uint16_t ref_adc = adc_hal_read_channel(config->ref_channel);

    // 防止除零
    if (ref_adc == 0) {
        return PT100_R0;
    }

    // 计算电阻值 (三线制PT100)
    // 假设使用恒流源和差分测量
    float voltage_ratio = (float)signal_adc / (float)ref_adc;
    float resistance = REFERENCE_RESISTOR * voltage_ratio;

    return resistance;
}

/**
 * @brief  获取传感器状态
 * @param  sensor_id 传感器ID (0-2)
 * @retval ftt518_status_t 传感器状态
 */
ftt518_status_t ftt518_get_status(uint8_t sensor_id)
{
    ftt518_status_t status = {0};

    if (sensor_id < 3) {
        status = sensor_status[sensor_id];
    }

    return status;
}

/**
 * @brief  校准传感器
 * @param  sensor_id 传感器ID (0-2)
 * @param  actual_temperature 实际温度值 (°C)
 * @retval bool 校准成功返回true
 */
bool ftt518_calibrate(uint8_t sensor_id, float actual_temperature)
{
    if (sensor_id >= 3 || sensor_status[sensor_id].fault) {
        return false;
    }

    // 读取当前传感器值
    float measured_temperature = read_temperature_ftt518_pt100(sensor_id);

    // 计算校准参数 (简化的单点校准)
    float error = actual_temperature - measured_temperature;

    ftt518_config_t *config = (ftt518_config_t *)&ftt518_configs[sensor_id];
    config->calibration_offset += error;

    return true;
}

/**
 * @brief  设置传感器校准参数
 * @param  sensor_id 传感器ID (0-2)
 * @param  offset 偏移量
 * @param  scale 比例系数
 * @retval bool 设置成功返回true
 */
bool ftt518_set_calibration(uint8_t sensor_id, float offset, float scale)
{
    if (sensor_id >= 3) {
        return false;
    }

    ftt518_config_t *config = (ftt518_config_t *)&ftt518_configs[sensor_id];
    config->calibration_offset = offset;
    config->calibration_scale = scale;

    return true;
}

/**
 * @brief  检查传感器连接状态
 * @param  sensor_id 传感器ID (0-2)
 * @retval bool 连接正常返回true
 */
bool ftt518_check_connection(uint8_t sensor_id)
{
    if (sensor_id >= 3) {
        return false;
    }

    float resistance = get_pt100_resistance_value(sensor_id);

    // 检查电阻是否在正常范围内
    return (resistance >= RESISTANCE_FAULT_MIN && resistance <= RESISTANCE_FAULT_MAX);
}

/**
 * @brief  获取传感器电阻值
 * @param  sensor_id 传感器ID (0-2)
 * @retval float 电阻值 (Ω)
 */
float ftt518_get_resistance(uint8_t sensor_id)
{
    if (sensor_id >= 3) {
        return PT100_R0;
    }

    return sensor_status[sensor_id].resistance;
}

/**
 * @brief  重置传感器故障状态
 * @param  sensor_id 传感器ID (0-2)
 * @retval None
 */
void ftt518_reset_fault(uint8_t sensor_id)
{
    if (sensor_id < 3) {
        sensor_status[sensor_id].fault = false;
        sensor_status[sensor_id].open_circuit = false;
        sensor_status[sensor_id].short_circuit = false;
        fault_counters[sensor_id] = 0;

        // 重置滤波器
        lowpass_filter_reset(&temp_filters[sensor_id]);
    }
}

/**
 * @brief  转换温度单位
 * @param  temperature_c 摄氏温度
 * @param  target_unit 目标单位 (0:°C, 1:°F, 2:K)
 * @retval float 转换后的温度值
 */
float ftt518_convert_temperature_unit(float temperature_c, uint8_t target_unit)
{
    switch (target_unit) {
        case 0: // °C
            return temperature_c;
        case 1: // °F
            return temperature_c * 9.0f / 5.0f + 32.0f;
        case 2: // K
            return temperature_c + 273.15f;
        default:
            return temperature_c;
    }
}

/**
 * @brief  执行传感器自检
 * @param  sensor_id 传感器ID (0-2)
 * @retval bool 自检通过返回true
 */
bool ftt518_self_test(uint8_t sensor_id)
{
    if (sensor_id >= 3) {
        return false;
    }

    // 检查连接状态
    if (!ftt518_check_connection(sensor_id)) {
        return false;
    }

    // 读取多次数据检查稳定性
    float readings[5];
    for (int i = 0; i < 5; i++) {
        readings[i] = read_temperature_ftt518_pt100(sensor_id);
        // 简单延时
        for (volatile int j = 0; j < 10000; j++);
    }

    // 检查数据稳定性 (标准差小于1°C)
    float mean = 0.0f;
    for (int i = 0; i < 5; i++) {
        mean += readings[i];
    }
    mean /= 5.0f;

    float variance = 0.0f;
    for (int i = 0; i < 5; i++) {
        float diff = readings[i] - mean;
        variance += diff * diff;
    }
    variance /= 5.0f;

    float std_dev = sqrtf(variance);

    return (std_dev < 1.0f); // 标准差小于1°C认为稳定
}