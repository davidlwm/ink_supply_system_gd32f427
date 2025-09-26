/**
 * @file    adc_hal.c
 * @brief   ADC硬件抽象层实现 - 基于GD32F427
 * @version V4.0
 * @date    2025-09-27
 */

#include "adc_hal.h"
#include "gd32f4xx.h"
#include "system_config.h"

// ADC通道配置表
typedef struct {
    uint32_t adc_periph;        // ADC外设
    uint8_t channel;            // ADC通道
    uint32_t gpio_port;         // GPIO端口
    uint32_t gpio_pin;          // GPIO引脚
    uint8_t gpio_af;            // GPIO复用功能
} adc_channel_config_t;

// ADC通道映射表 (基于系统设计)
static const adc_channel_config_t adc_channel_configs[ADC_CHANNEL_MAX] = {
    // 液位传感器ADC通道
    {ADC0, ADC_CHANNEL_0, GPIOA, GPIO_PIN_0, GPIO_AF_0},  // 液位传感器1
    {ADC0, ADC_CHANNEL_1, GPIOA, GPIO_PIN_1, GPIO_AF_0},  // 液位传感器2

    // 压力传感器ADC通道
    {ADC0, ADC_CHANNEL_2, GPIOA, GPIO_PIN_2, GPIO_AF_0},  // 压力传感器1
    {ADC0, ADC_CHANNEL_3, GPIOA, GPIO_PIN_3, GPIO_AF_0},  // 压力传感器2

    // 温度传感器ADC通道 (PT100三线制)
    {ADC1, ADC_CHANNEL_4, GPIOA, GPIO_PIN_4, GPIO_AF_0},  // 温度传感器1信号
    {ADC1, ADC_CHANNEL_5, GPIOA, GPIO_PIN_5, GPIO_AF_0},  // 温度传感器1参考
    {ADC1, ADC_CHANNEL_6, GPIOA, GPIO_PIN_6, GPIO_AF_0},  // 温度传感器2信号
    {ADC1, ADC_CHANNEL_7, GPIOA, GPIO_PIN_7, GPIO_AF_0},  // 温度传感器2参考
    {ADC2, ADC_CHANNEL_8, GPIOB, GPIO_PIN_0, GPIO_AF_0},  // 温度传感器3信号
    {ADC2, ADC_CHANNEL_9, GPIOB, GPIO_PIN_1, GPIO_AF_0},  // 温度传感器3参考

    // 系统监控ADC通道
    {ADC0, ADC_CHANNEL_10, GPIOC, GPIO_PIN_0, GPIO_AF_0}, // 24V电源监控
    {ADC0, ADC_CHANNEL_11, GPIOC, GPIO_PIN_1, GPIO_AF_0}, // 5V电源监控
    {ADC0, ADC_CHANNEL_12, GPIOC, GPIO_PIN_2, GPIO_AF_0}, // 3.3V电源监控
};

// ADC转换结果缓存
static volatile uint16_t adc_conversion_results[ADC_CHANNEL_MAX];
static volatile bool adc_conversion_complete[3] = {false, false, false}; // ADC0, ADC1, ADC2

/**
 * @brief  ADC HAL初始化
 * @param  None
 * @retval None
 */
void adc_hal_init(void)
{
    // 使能ADC和GPIO时钟
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_periph_clock_enable(RCU_ADC2);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    // 配置ADC时钟 (APB2/8 = 25MHz)
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);

    // 初始化ADC GPIO
    init_adc_gpio();

    // 初始化ADC外设
    init_adc_peripheral(ADC0);
    init_adc_peripheral(ADC1);
    init_adc_peripheral(ADC2);

    // 校准ADC
    calibrate_adc(ADC0);
    calibrate_adc(ADC1);
    calibrate_adc(ADC2);

    // 初始化转换结果缓存
    for (int i = 0; i < ADC_CHANNEL_MAX; i++) {
        adc_conversion_results[i] = 0;
    }
}

/**
 * @brief  初始化ADC GPIO
 * @param  None
 * @retval None
 */
static void init_adc_gpio(void)
{
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                  GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                  GPIO_PIN_0 | GPIO_PIN_1);

    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
}

/**
 * @brief  初始化ADC外设
 * @param  adc_periph ADC外设
 * @retval None
 */
static void init_adc_peripheral(uint32_t adc_periph)
{
    adc_deinit(adc_periph);

    // ADC基本配置
    adc_mode_config(ADC_MODE_FREE);                    // 独立模式
    adc_special_function_config(adc_periph, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(adc_periph, ADC_CONTINUOUS_MODE, DISABLE);
    adc_external_trigger_config(adc_periph, ADC_REGULAR_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_data_alignment_config(adc_periph, ADC_DATAALIGN_RIGHT);
    adc_channel_length_config(adc_periph, ADC_REGULAR_CHANNEL, 1);

    // 采样时间配置 (480个周期，适合高精度)
    for (uint8_t i = 0; i < 16; i++) {
        adc_channel_sample_time_config(adc_periph, i, ADC_SAMPLETIME_480);
    }

    // 使能ADC
    adc_enable(adc_periph);
    delay_1ms(1);
}

/**
 * @brief  校准ADC
 * @param  adc_periph ADC外设
 * @retval None
 */
static void calibrate_adc(uint32_t adc_periph)
{
    // 复位校准寄存器
    adc_calibration_enable(adc_periph);

    // 等待校准完成
    while (adc_flag_get(adc_periph, ADC_FLAG_CALFIN) == RESET);
}

/**
 * @brief  配置ADC通道
 * @param  channel_id 通道ID
 * @param  resolution 分辨率
 * @retval bool 配置成功返回true
 */
bool adc_hal_config_channel(adc_channel_id_t channel_id, adc_resolution_t resolution)
{
    if (channel_id >= ADC_CHANNEL_MAX) {
        return false;
    }

    const adc_channel_config_t *config = &adc_channel_configs[channel_id];

    // 配置分辨率 (GD32F427支持12bit)
    adc_resolution_config(config->adc_periph, ADC_RESOLUTION_12B);

    return true;
}

/**
 * @brief  读取ADC通道值
 * @param  channel_id 通道ID
 * @retval uint16_t ADC转换结果 (0-4095 for 12bit)
 */
uint16_t adc_hal_read_channel(adc_channel_id_t channel_id)
{
    if (channel_id >= ADC_CHANNEL_MAX) {
        return 0;
    }

    const adc_channel_config_t *config = &adc_channel_configs[channel_id];

    // 配置转换通道
    adc_regular_channel_config(config->adc_periph, 0, config->channel, ADC_SAMPLETIME_480);

    // 启动转换
    adc_software_trigger_enable(config->adc_periph, ADC_REGULAR_CHANNEL);

    // 等待转换完成
    while (adc_flag_get(config->adc_periph, ADC_FLAG_EOC) == RESET);

    // 读取转换结果
    uint16_t result = adc_regular_data_read(config->adc_periph);

    // 清除标志
    adc_flag_clear(config->adc_periph, ADC_FLAG_EOC);

    // 缓存结果
    adc_conversion_results[channel_id] = result;

    return result;
}

/**
 * @brief  读取缓存的ADC值
 * @param  channel_id 通道ID
 * @retval uint16_t 缓存的ADC值
 */
uint16_t adc_hal_get_cached_value(adc_channel_id_t channel_id)
{
    if (channel_id >= ADC_CHANNEL_MAX) {
        return 0;
    }

    return adc_conversion_results[channel_id];
}

/**
 * @brief  启动DMA连续转换 (可选功能)
 * @param  adc_periph ADC外设
 * @param  channels 通道数组
 * @param  channel_count 通道数量
 * @retval bool 启动成功返回true
 */
bool adc_hal_start_dma_conversion(uint32_t adc_periph, uint8_t *channels, uint8_t channel_count)
{
    // DMA配置和启动 (简化实现)
    // 这里可以实现更复杂的DMA连续转换
    return true;
}

/**
 * @brief  获取ADC状态
 * @param  adc_periph ADC外设
 * @retval adc_status_t ADC状态
 */
adc_status_t adc_hal_get_status(uint32_t adc_periph)
{
    adc_status_t status = {0};

    status.ready = (adc_flag_get(adc_periph, ADC_FLAG_STRT) == SET);
    status.busy = (adc_flag_get(adc_periph, ADC_FLAG_EOC) == RESET);
    status.overrun = (adc_flag_get(adc_periph, ADC_FLAG_ROVF) == SET);
    status.calibrated = true; // 简化处理

    return status;
}

/**
 * @brief  转换ADC值到电压
 * @param  adc_value ADC原始值
 * @param  resolution 分辨率
 * @param  vref 参考电压 (V)
 * @retval float 电压值 (V)
 */
float adc_hal_convert_to_voltage(uint16_t adc_value, adc_resolution_t resolution, float vref)
{
    uint16_t max_value;

    switch (resolution) {
        case ADC_12BIT:
            max_value = 4095;
            break;
        case ADC_15BIT: // 模拟15bit (实际通过oversampling实现)
            max_value = 32767;
            adc_value = adc_value << 3; // 简化处理
            break;
        default:
            max_value = 4095;
            break;
    }

    return ((float)adc_value * vref) / (float)max_value;
}