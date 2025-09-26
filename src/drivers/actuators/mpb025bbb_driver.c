/**
 * @file    mpb025bbb_driver.c
 * @brief   MPB025BBB微型齿轮泵驱动实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "mpb025bbb_driver.h"
#include "pwm_hal.h"
#include "gpio_hal.h"
#include "system_config.h"

// 泵配置参数
typedef struct {
    pwm_channel_id_t pwm_channel;      // PWM通道
    gpio_output_id_t enable_pin;       // 使能引脚
    gpio_output_id_t direction_pin;    // 方向引脚
    uint16_t max_speed;                // 最大转速 (RPM)
    uint16_t min_speed;                // 最小转速 (RPM)
    float rated_voltage;               // 额定电压 (V)
    float rated_current;               // 额定电流 (A)
    float flow_rate_ml_per_rpm;        // 流量系数 (ml/min per RPM)
} mpb025bbb_config_t;

// 泵实例配置
static const mpb025bbb_config_t pump_configs[2] = {
    // 泵1 (主供墨泵)
    {
        .pwm_channel = PWM_CH_PUMP_1,
        .enable_pin = GPIO_PUMP_1_EN,
        .direction_pin = GPIO_PUMP_1_DIR,
        .max_speed = 4500,              // 4500 RPM
        .min_speed = 500,               // 500 RPM
        .rated_voltage = 12.0f,
        .rated_current = 0.8f,
        .flow_rate_ml_per_rpm = 0.025f  // 25μl/min per RPM
    },
    // 泵2 (辅助循环泵)
    {
        .pwm_channel = PWM_CH_PUMP_2,
        .enable_pin = GPIO_PUMP_2_EN,
        .direction_pin = GPIO_PUMP_2_DIR,
        .max_speed = 4000,
        .min_speed = 500,
        .rated_voltage = 12.0f,
        .rated_current = 0.6f,
        .flow_rate_ml_per_rpm = 0.020f  // 20μl/min per RPM
    }
};

// 泵状态
static mpb025bbb_status_t pump_status[2];

// 故障检测参数
#define PUMP_STALL_DETECTION_TIME   5000    // 堵转检测时间 (ms)
#define PUMP_STARTUP_TIME          2000     // 启动时间 (ms)

// 状态计时器
static uint32_t pump_timers[2] = {0, 0};

/**
 * @brief  MPB025BBB泵初始化
 * @param  None
 * @retval None
 */
void mpb025bbb_pump_init(void)
{
    // 初始化PWM通道和GPIO
    for (int i = 0; i < 2; i++) {
        // 配置PWM通道 (25kHz适合直流电机)
        pwm_hal_config_channel(pump_configs[i].pwm_channel, 25000);

        // 配置GPIO引脚
        gpio_hal_config_output(pump_configs[i].enable_pin);
        gpio_hal_config_output(pump_configs[i].direction_pin);

        // 初始化状态
        pump_status[i].speed_rpm = 0;
        pump_status[i].target_speed = 0;
        pump_status[i].enabled = false;
        pump_status[i].direction = PUMP_DIRECTION_FORWARD;
        pump_status[i].fault = false;
        pump_status[i].stalled = false;
        pump_status[i].overcurrent = false;
        pump_status[i].actual_current = 0.0f;
        pump_status[i].flow_rate_ml_min = 0.0f;
        pump_status[i].initialized = true;

        // 停止泵
        control_pump_speed_mpb025bbb(i, 0);

        // 重置计时器
        pump_timers[i] = 0;
    }
}

/**
 * @brief  控制泵转速
 * @param  pump_id 泵ID (0-1)
 * @param  target_rpm 目标转速 (RPM)
 * @retval None
 */
void control_pump_speed_mpb025bbb(uint8_t pump_id, uint16_t target_rpm)
{
    if (pump_id >= 2) {
        return;
    }

    const mpb025bbb_config_t *config = &pump_configs[pump_id];

    // 限制转速范围
    if (target_rpm > 0 && target_rpm < config->min_speed) {
        target_rpm = config->min_speed;
    }
    if (target_rpm > config->max_speed) {
        target_rpm = config->max_speed;
    }

    // 安全检查
    if (pump_status[pump_id].fault ||
        pump_status[pump_id].stalled ||
        pump_status[pump_id].overcurrent) {
        target_rpm = 0;
    }

    // 更新状态
    pump_status[pump_id].target_speed = target_rpm;
    pump_status[pump_id].enabled = (target_rpm > 0);

    // 计算PWM占空比 (转速到占空比的线性映射)
    uint16_t duty_cycle = 0;
    if (target_rpm > 0) {
        // 占空比范围：20%-95% 对应 min_speed - max_speed
        float speed_ratio = (float)(target_rpm - config->min_speed) /
                           (float)(config->max_speed - config->min_speed);
        duty_cycle = (uint16_t)(2000 + speed_ratio * 7500); // 20%-95% of 10000
    }

    // 设置PWM占空比
    pwm_hal_set_duty_cycle(config->pwm_channel, duty_cycle);

    // 控制使能引脚
    gpio_hal_write_pin(config->enable_pin, pump_status[pump_id].enabled);

    // 设置方向 (默认正向)
    gpio_hal_write_pin(config->direction_pin,
                      (pump_status[pump_id].direction == PUMP_DIRECTION_FORWARD));

    // 启用/禁用PWM通道
    pwm_hal_enable_channel(config->pwm_channel, pump_status[pump_id].enabled);

    // 更新实际转速 (简化处理，实际应该通过反馈获取)
    pump_status[pump_id].speed_rpm = target_rpm;

    // 计算流量
    pump_status[pump_id].flow_rate_ml_min = target_rpm * config->flow_rate_ml_per_rpm;

    // 更新计时器
    if (target_rpm > 0) {
        if (pump_timers[pump_id] == 0) {
            pump_timers[pump_id] = xTaskGetTickCount();
        }
    } else {
        pump_timers[pump_id] = 0;
    }
}

/**
 * @brief  获取泵状态
 * @param  pump_id 泵ID (0-1)
 * @retval mpb025bbb_status_t 泵状态
 */
mpb025bbb_status_t mpb025bbb_get_status(uint8_t pump_id)
{
    mpb025bbb_status_t status = {0};

    if (pump_id < 2) {
        status = pump_status[pump_id];
    }

    return status;
}

/**
 * @brief  设置泵转速
 * @param  pump_id 泵ID (0-1)
 * @param  speed_rpm 转速 (RPM)
 * @retval bool 设置成功返回true
 */
bool mpb025bbb_set_speed(uint8_t pump_id, uint16_t speed_rpm)
{
    if (pump_id >= 2) {
        return false;
    }

    control_pump_speed_mpb025bbb(pump_id, speed_rpm);
    return true;
}

/**
 * @brief  获取泵转速
 * @param  pump_id 泵ID (0-1)
 * @retval uint16_t 当前转速 (RPM)
 */
uint16_t mpb025bbb_get_speed(uint8_t pump_id)
{
    if (pump_id >= 2) {
        return 0;
    }

    return pump_status[pump_id].speed_rpm;
}

/**
 * @brief  设置泵方向
 * @param  pump_id 泵ID (0-1)
 * @param  direction 方向
 * @retval bool 设置成功返回true
 */
bool mpb025bbb_set_direction(uint8_t pump_id, pump_direction_t direction)
{
    if (pump_id >= 2) {
        return false;
    }

    pump_status[pump_id].direction = direction;

    // 更新方向引脚
    gpio_hal_write_pin(pump_configs[pump_id].direction_pin,
                      (direction == PUMP_DIRECTION_FORWARD));

    return true;
}

/**
 * @brief  使能/禁用泵
 * @param  pump_id 泵ID (0-1)
 * @param  enable 使能状态
 * @retval bool 操作成功返回true
 */
bool mpb025bbb_enable(uint8_t pump_id, bool enable)
{
    if (pump_id >= 2) {
        return false;
    }

    if (!enable) {
        // 禁用时停止泵
        control_pump_speed_mpb025bbb(pump_id, 0);
    } else {
        // 使能时恢复之前的转速设置
        uint16_t target_speed = pump_status[pump_id].target_speed;
        control_pump_speed_mpb025bbb(pump_id, target_speed);
    }

    return true;
}

/**
 * @brief  检查泵故障
 * @param  pump_id 泵ID (0-1)
 * @retval bool 有故障返回true
 */
bool mpb025bbb_check_fault(uint8_t pump_id)
{
    if (pump_id >= 2) {
        return true;
    }

    // 检查堵转保护
    if (pump_status[pump_id].enabled && pump_timers[pump_id] != 0) {
        uint32_t current_time = xTaskGetTickCount();
        uint32_t run_time = current_time - pump_timers[pump_id];

        // 如果运行时间超过启动时间且转速为0，认为堵转
        if (run_time > pdMS_TO_TICKS(PUMP_STARTUP_TIME) &&
            pump_status[pump_id].speed_rpm == 0) {
            pump_status[pump_id].stalled = true;
            pump_status[pump_id].fault = true;
            control_pump_speed_mpb025bbb(pump_id, 0); // 安全停止
        }
    }

    return pump_status[pump_id].fault ||
           pump_status[pump_id].stalled ||
           pump_status[pump_id].overcurrent;
}

/**
 * @brief  重置泵故障
 * @param  pump_id 泵ID (0-1)
 * @retval None
 */
void mpb025bbb_reset_fault(uint8_t pump_id)
{
    if (pump_id < 2) {
        pump_status[pump_id].fault = false;
        pump_status[pump_id].stalled = false;
        pump_status[pump_id].overcurrent = false;
        pump_timers[pump_id] = 0;
    }
}

/**
 * @brief  紧急停止所有泵
 * @param  None
 * @retval None
 */
void mpb025bbb_emergency_stop(void)
{
    for (int i = 0; i < 2; i++) {
        control_pump_speed_mpb025bbb(i, 0);
        pump_status[i].fault = true;
    }
}

/**
 * @brief  获取泵流量
 * @param  pump_id 泵ID (0-1)
 * @retval float 流量 (ml/min)
 */
float mpb025bbb_get_flow_rate(uint8_t pump_id)
{
    if (pump_id >= 2) {
        return 0.0f;
    }

    return pump_status[pump_id].flow_rate_ml_min;
}

/**
 * @brief  执行泵自检
 * @param  pump_id 泵ID (0-1)
 * @retval bool 自检通过返回true
 */
bool mpb025bbb_self_test(uint8_t pump_id)
{
    if (pump_id >= 2) {
        return false;
    }

    // 简单的自检：运行低速并检查状态
    uint16_t original_speed = pump_status[pump_id].target_speed;

    // 设置最小转速进行测试
    control_pump_speed_mpb025bbb(pump_id, pump_configs[pump_id].min_speed);

    // 等待启动
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 检查PWM是否正常输出
    pwm_status_t pwm_status = pwm_hal_get_status(pump_configs[pump_id].pwm_channel);
    bool test_passed = pwm_status.enabled && (pwm_status.duty_cycle > 0);

    // 恢复原来的转速设置
    control_pump_speed_mpb025bbb(pump_id, original_speed);

    return test_passed && !pump_status[pump_id].fault;
}

/**
 * @brief  获取泵最大转速
 * @param  pump_id 泵ID (0-1)
 * @retval uint16_t 最大转速 (RPM)
 */
uint16_t mpb025bbb_get_max_speed(uint8_t pump_id)
{
    if (pump_id >= 2) {
        return 0;
    }

    return pump_configs[pump_id].max_speed;
}

/**
 * @brief  获取泵最小转速
 * @param  pump_id 泵ID (0-1)
 * @retval uint16_t 最小转速 (RPM)
 */
uint16_t mpb025bbb_get_min_speed(uint8_t pump_id)
{
    if (pump_id >= 2) {
        return 0;
    }

    return pump_configs[pump_id].min_speed;
}