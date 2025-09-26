/**
 * @file    mra23d3_driver.c
 * @brief   MRA-23D3固态继电器加热器驱动实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "mra23d3_driver.h"
#include "pwm_hal.h"
#include "gpio_hal.h"
#include "system_config.h"

// 加热器配置参数
typedef struct {
    pwm_channel_id_t pwm_channel;      // PWM通道
    gpio_output_id_t enable_pin;       // 使能引脚
    float max_power;                   // 最大功率 (%)
    float rated_voltage;               // 额定电压 (V)
    float rated_current;               // 额定电流 (A)
    float thermal_time_constant;       // 热时间常数 (s)
} mra23d3_config_t;

// 加热器实例配置
static const mra23d3_config_t heater_configs[3] = {
    // 加热器1 (墨盒1)
    {
        .pwm_channel = PWM_CH_HEATER_1,
        .enable_pin = GPIO_HEATER_1,
        .max_power = 100.0f,
        .rated_voltage = 24.0f,
        .rated_current = 2.0f,
        .thermal_time_constant = 30.0f
    },
    // 加热器2 (墨盒2)
    {
        .pwm_channel = PWM_CH_HEATER_2,
        .enable_pin = GPIO_HEATER_2,
        .max_power = 100.0f,
        .rated_voltage = 24.0f,
        .rated_current = 2.0f,
        .thermal_time_constant = 30.0f
    },
    // 加热器3 (墨盒3)
    {
        .pwm_channel = PWM_CH_HEATER_3,
        .enable_pin = GPIO_HEATER_3,
        .max_power = 80.0f,         // 功率限制
        .rated_voltage = 24.0f,
        .rated_current = 1.5f,
        .thermal_time_constant = 25.0f
    }
};

// 加热器状态
static mra23d3_status_t heater_status[3];

// 过温保护阈值
#define OVERTEMP_PROTECTION_TEMP    85.0f   // 过温保护温度
#define OVERPOWER_PROTECTION_TIME   10000   // 过功率保护时间 (ms)

// 安全计时器
static uint32_t overpower_timers[3] = {0, 0, 0};

/**
 * @brief  MRA-23D3加热器初始化
 * @param  None
 * @retval None
 */
void mra23d3_heater_init(void)
{
    // 初始化PWM通道
    for (int i = 0; i < 3; i++) {
        // 配置PWM通道 (1kHz适合SSR)
        pwm_hal_config_channel(heater_configs[i].pwm_channel, 1000);

        // 配置使能引脚
        gpio_hal_config_output(heater_configs[i].enable_pin);

        // 初始化状态
        heater_status[i].power_percent = 0.0f;
        heater_status[i].enabled = false;
        heater_status[i].fault = false;
        heater_status[i].overtemp = false;
        heater_status[i].overcurrent = false;
        heater_status[i].actual_current = 0.0f;
        heater_status[i].actual_voltage = 0.0f;
        heater_status[i].initialized = true;

        // 关闭加热器
        control_heater_mra23d3(i, 0.0f);

        // 重置安全计时器
        overpower_timers[i] = 0;
    }
}

/**
 * @brief  控制加热器功率
 * @param  heater_id 加热器ID (0-2)
 * @param  power_percent 功率百分比 (0-100%)
 * @retval None
 */
void control_heater_mra23d3(uint8_t heater_id, float power_percent)
{
    if (heater_id >= 3) {
        return;
    }

    const mra23d3_config_t *config = &heater_configs[heater_id];

    // 限制功率范围
    if (power_percent < 0.0f) power_percent = 0.0f;
    if (power_percent > config->max_power) power_percent = config->max_power;

    // 安全检查
    if (heater_status[heater_id].fault ||
        heater_status[heater_id].overtemp ||
        heater_status[heater_id].overcurrent) {
        power_percent = 0.0f;
    }

    // 更新状态
    heater_status[heater_id].power_percent = power_percent;
    heater_status[heater_id].enabled = (power_percent > 0.0f);

    // 设置PWM占空比 (0-10000对应0-100%)
    uint16_t duty_cycle = (uint16_t)(power_percent * 100.0f);
    pwm_hal_set_duty_cycle(config->pwm_channel, duty_cycle);

    // 控制使能引脚
    gpio_hal_write_pin(config->enable_pin, heater_status[heater_id].enabled);

    // 启用/禁用PWM通道
    pwm_hal_enable_channel(config->pwm_channel, heater_status[heater_id].enabled);

    // 更新过功率计时器
    if (power_percent > 90.0f) {
        if (overpower_timers[heater_id] == 0) {
            overpower_timers[heater_id] = xTaskGetTickCount();
        }
    } else {
        overpower_timers[heater_id] = 0;
    }
}

/**
 * @brief  获取加热器状态
 * @param  heater_id 加热器ID (0-2)
 * @retval mra23d3_status_t 加热器状态
 */
mra23d3_status_t mra23d3_get_status(uint8_t heater_id)
{
    mra23d3_status_t status = {0};

    if (heater_id < 3) {
        status = heater_status[heater_id];
    }

    return status;
}

/**
 * @brief  设置加热器功率
 * @param  heater_id 加热器ID (0-2)
 * @param  power_percent 功率百分比 (0-100%)
 * @retval bool 设置成功返回true
 */
bool mra23d3_set_power(uint8_t heater_id, float power_percent)
{
    if (heater_id >= 3) {
        return false;
    }

    control_heater_mra23d3(heater_id, power_percent);
    return true;
}

/**
 * @brief  获取加热器功率
 * @param  heater_id 加热器ID (0-2)
 * @retval float 当前功率百分比
 */
float mra23d3_get_power(uint8_t heater_id)
{
    if (heater_id >= 3) {
        return 0.0f;
    }

    return heater_status[heater_id].power_percent;
}

/**
 * @brief  使能/禁用加热器
 * @param  heater_id 加热器ID (0-2)
 * @param  enable 使能状态
 * @retval bool 操作成功返回true
 */
bool mra23d3_enable(uint8_t heater_id, bool enable)
{
    if (heater_id >= 3) {
        return false;
    }

    if (!enable) {
        // 禁用时关闭功率输出
        control_heater_mra23d3(heater_id, 0.0f);
    } else {
        // 使能时恢复之前的功率设置
        float current_power = heater_status[heater_id].power_percent;
        control_heater_mra23d3(heater_id, current_power);
    }

    return true;
}

/**
 * @brief  检查加热器故障
 * @param  heater_id 加热器ID (0-2)
 * @retval bool 有故障返回true
 */
bool mra23d3_check_fault(uint8_t heater_id)
{
    if (heater_id >= 3) {
        return true;
    }

    // 检查过功率保护
    if (overpower_timers[heater_id] != 0) {
        uint32_t current_time = xTaskGetTickCount();
        if ((current_time - overpower_timers[heater_id]) > pdMS_TO_TICKS(OVERPOWER_PROTECTION_TIME)) {
            heater_status[heater_id].fault = true;
            control_heater_mra23d3(heater_id, 0.0f); // 安全关闭
        }
    }

    return heater_status[heater_id].fault ||
           heater_status[heater_id].overtemp ||
           heater_status[heater_id].overcurrent;
}

/**
 * @brief  重置加热器故障
 * @param  heater_id 加热器ID (0-2)
 * @retval None
 */
void mra23d3_reset_fault(uint8_t heater_id)
{
    if (heater_id < 3) {
        heater_status[heater_id].fault = false;
        heater_status[heater_id].overtemp = false;
        heater_status[heater_id].overcurrent = false;
        overpower_timers[heater_id] = 0;
    }
}

/**
 * @brief  设置过温保护
 * @param  heater_id 加热器ID (0-2)
 * @param  overtemp 过温状态
 * @retval None
 */
void mra23d3_set_overtemp_protection(uint8_t heater_id, bool overtemp)
{
    if (heater_id < 3) {
        heater_status[heater_id].overtemp = overtemp;

        if (overtemp) {
            // 过温时立即关闭加热器
            control_heater_mra23d3(heater_id, 0.0f);
        }
    }
}

/**
 * @brief  紧急关闭所有加热器
 * @param  None
 * @retval None
 */
void mra23d3_emergency_shutdown(void)
{
    for (int i = 0; i < 3; i++) {
        control_heater_mra23d3(i, 0.0f);
        heater_status[i].fault = true;
    }
}

/**
 * @brief  获取加热器最大功率
 * @param  heater_id 加热器ID (0-2)
 * @retval float 最大功率百分比
 */
float mra23d3_get_max_power(uint8_t heater_id)
{
    if (heater_id >= 3) {
        return 0.0f;
    }

    return heater_configs[heater_id].max_power;
}

/**
 * @brief  设置加热器最大功率
 * @param  heater_id 加热器ID (0-2)
 * @param  max_power 最大功率百分比
 * @retval bool 设置成功返回true
 */
bool mra23d3_set_max_power(uint8_t heater_id, float max_power)
{
    if (heater_id >= 3 || max_power < 0.0f || max_power > 100.0f) {
        return false;
    }

    // 更新配置 (注意：这里直接修改const结构体，实际应用中应该有专门的配置管理)
    mra23d3_config_t *config = (mra23d3_config_t *)&heater_configs[heater_id];
    config->max_power = max_power;

    // 如果当前功率超过新的最大功率，则降低功率
    if (heater_status[heater_id].power_percent > max_power) {
        control_heater_mra23d3(heater_id, max_power);
    }

    return true;
}

/**
 * @brief  执行加热器自检
 * @param  heater_id 加热器ID (0-2)
 * @retval bool 自检通过返回true
 */
bool mra23d3_self_test(uint8_t heater_id)
{
    if (heater_id >= 3) {
        return false;
    }

    // 简单的自检：设置小功率输出并检查状态
    float original_power = heater_status[heater_id].power_percent;

    // 设置5%功率进行测试
    control_heater_mra23d3(heater_id, 5.0f);

    // 等待一段时间
    vTaskDelay(pdMS_TO_TICKS(100));

    // 检查PWM是否正常输出
    pwm_status_t pwm_status = pwm_hal_get_status(heater_configs[heater_id].pwm_channel);
    bool test_passed = pwm_status.enabled && (pwm_status.duty_cycle > 0);

    // 恢复原来的功率设置
    control_heater_mra23d3(heater_id, original_power);

    return test_passed;
}