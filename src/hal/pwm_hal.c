/**
 * @file    pwm_hal.c
 * @brief   PWM硬件抽象层实现 - 基于GD32F427
 * @version V4.0
 * @date    2025-09-27
 */

#include "pwm_hal.h"
#include "gd32f4xx.h"
#include "system_config.h"

// PWM通道配置表
typedef struct {
    uint32_t timer_periph;      // 定时器外设
    uint16_t channel;           // 定时器通道
    uint32_t gpio_port;         // GPIO端口
    uint32_t gpio_pin;          // GPIO引脚
    uint8_t gpio_af;            // GPIO复用功能
    uint32_t frequency;         // PWM频率
    uint16_t resolution;        // PWM分辨率
} pwm_channel_config_t;

// PWM通道映射表 (基于系统设计)
static const pwm_channel_config_t pwm_channel_configs[PWM_CHANNEL_MAX] = {
    // 泵控制PWM (MPB025BBB调速泵)
    {TIMER0, TIMER_CH_0, GPIOE, GPIO_PIN_9,  GPIO_AF_1, 1000, 1000},  // 泵1调速
    {TIMER0, TIMER_CH_1, GPIOE, GPIO_PIN_11, GPIO_AF_1, 1000, 1000},  // 泵2调速

    // 加热器控制PWM (MRA-23D3固态继电器)
    {TIMER1, TIMER_CH_0, GPIOA, GPIO_PIN_8,  GPIO_AF_1, 10000, 1000}, // 加热器1
    {TIMER1, TIMER_CH_1, GPIOA, GPIO_PIN_9,  GPIO_AF_1, 10000, 1000}, // 加热器2
    {TIMER1, TIMER_CH_2, GPIOA, GPIO_PIN_10, GPIO_AF_1, 10000, 1000}, // 加热器3

    // LED PWM控制 (亮度控制)
    {TIMER2, TIMER_CH_0, GPIOC, GPIO_PIN_6,  GPIO_AF_2, 1000, 100},   // LED1
    {TIMER2, TIMER_CH_1, GPIOC, GPIO_PIN_7,  GPIO_AF_2, 1000, 100},   // LED2
    {TIMER2, TIMER_CH_2, GPIOC, GPIO_PIN_8,  GPIO_AF_2, 1000, 100},   // LED3
    {TIMER2, TIMER_CH_3, GPIOC, GPIO_PIN_9,  GPIO_AF_2, 1000, 100},   // LED4

    // 备用PWM通道
    {TIMER3, TIMER_CH_0, GPIOD, GPIO_PIN_12, GPIO_AF_2, 1000, 1000},  // 备用1
    {TIMER3, TIMER_CH_1, GPIOD, GPIO_PIN_13, GPIO_AF_2, 1000, 1000},  // 备用2
};

// PWM状态缓存
static volatile uint16_t pwm_duty_cache[PWM_CHANNEL_MAX];
static volatile bool pwm_enabled[PWM_CHANNEL_MAX];

/**
 * @brief  PWM HAL初始化
 * @param  None
 * @retval None
 */
void pwm_hal_init(void)
{
    // 使能定时器和GPIO时钟
    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);

    // 初始化PWM GPIO
    init_pwm_gpio();

    // 初始化定时器
    init_timer_pwm(TIMER0, 1000);  // 1kHz for pumps
    init_timer_pwm(TIMER1, 10000); // 10kHz for heaters
    init_timer_pwm(TIMER2, 1000);  // 1kHz for LEDs
    init_timer_pwm(TIMER3, 1000);  // 1kHz for spare

    // 初始化PWM状态
    for (int i = 0; i < PWM_CHANNEL_MAX; i++) {
        pwm_duty_cache[i] = 0;
        pwm_enabled[i] = false;
    }
}

/**
 * @brief  初始化PWM GPIO
 * @param  None
 * @retval None
 */
static void init_pwm_gpio(void)
{
    // TIMER0 PWM GPIO (PE9, PE11)
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9 | GPIO_PIN_11);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9 | GPIO_PIN_11);
    gpio_af_set(GPIOE, GPIO_AF_1, GPIO_PIN_9 | GPIO_PIN_11);

    // TIMER1 PWM GPIO (PA8, PA9, PA10)
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);

    // TIMER2 PWM GPIO (PC6, PC7, PC8, PC9)
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
    gpio_af_set(GPIOC, GPIO_AF_2, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);

    // TIMER3 PWM GPIO (PD12, PD13)
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12 | GPIO_PIN_13);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12 | GPIO_PIN_13);
    gpio_af_set(GPIOD, GPIO_AF_2, GPIO_PIN_12 | GPIO_PIN_13);
}

/**
 * @brief  初始化定时器PWM
 * @param  timer_periph 定时器外设
 * @param  frequency PWM频率
 * @retval None
 */
static void init_timer_pwm(uint32_t timer_periph, uint32_t frequency)
{
    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocintpara;

    timer_deinit(timer_periph);

    // 计算预分频器和周期值 (APB1 = 50MHz, APB2 = 100MHz)
    uint32_t timer_clock = (timer_periph == TIMER0 || timer_periph == TIMER1) ? 100000000 : 50000000;
    uint32_t period = 1000; // PWM分辨率
    uint32_t prescaler = (timer_clock / (frequency * period)) - 1;

    // 定时器基本配置
    timer_initpara.prescaler         = prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period - 1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(timer_periph, &timer_initpara);

    // PWM输出配置
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    // 配置所有通道
    timer_channel_output_config(timer_periph, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(timer_periph, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(timer_periph, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_config(timer_periph, TIMER_CH_3, &timer_ocintpara);

    // 设置PWM模式
    timer_channel_output_mode_config(timer_periph, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(timer_periph, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(timer_periph, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(timer_periph, TIMER_CH_3, TIMER_OC_MODE_PWM0);

    // 初始占空比为0
    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_0, 0);
    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_1, 0);
    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_2, 0);
    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_3, 0);

    // 启动定时器
    timer_enable(timer_periph);
}

/**
 * @brief  配置PWM通道
 * @param  channel_id 通道ID
 * @param  frequency PWM频率
 * @retval bool 配置成功返回true
 */
bool pwm_hal_config_channel(pwm_channel_id_t channel_id, uint32_t frequency)
{
    if (channel_id >= PWM_CHANNEL_MAX) {
        return false;
    }

    const pwm_channel_config_t *config = &pwm_channel_configs[channel_id];

    // 重新配置定时器频率 (简化实现)
    if (frequency != config->frequency) {
        init_timer_pwm(config->timer_periph, frequency);
    }

    return true;
}

/**
 * @brief  设置PWM占空比
 * @param  channel_id 通道ID
 * @param  duty_cycle 占空比 (0-1000)
 * @retval bool 设置成功返回true
 */
bool pwm_hal_set_duty_cycle(pwm_channel_id_t channel_id, uint16_t duty_cycle)
{
    if (channel_id >= PWM_CHANNEL_MAX) {
        return false;
    }

    const pwm_channel_config_t *config = &pwm_channel_configs[channel_id];

    // 限制占空比范围
    if (duty_cycle > config->resolution) {
        duty_cycle = config->resolution;
    }

    // 设置占空比
    timer_channel_output_pulse_value_config(config->timer_periph, config->channel, duty_cycle);

    // 缓存占空比值
    pwm_duty_cache[channel_id] = duty_cycle;
    pwm_enabled[channel_id] = (duty_cycle > 0);

    return true;
}

/**
 * @brief  启用/禁用PWM通道
 * @param  channel_id 通道ID
 * @param  enable 启用标志
 * @retval bool 操作成功返回true
 */
bool pwm_hal_enable_channel(pwm_channel_id_t channel_id, bool enable)
{
    if (channel_id >= PWM_CHANNEL_MAX) {
        return false;
    }

    const pwm_channel_config_t *config = &pwm_channel_configs[channel_id];

    if (enable) {
        // 启用通道输出
        timer_channel_output_state_config(config->timer_periph, config->channel, TIMER_CCX_ENABLE);
        // 恢复占空比
        timer_channel_output_pulse_value_config(config->timer_periph, config->channel, pwm_duty_cache[channel_id]);
    } else {
        // 禁用通道输出
        timer_channel_output_state_config(config->timer_periph, config->channel, TIMER_CCX_DISABLE);
        // 设置占空比为0
        timer_channel_output_pulse_value_config(config->timer_periph, config->channel, 0);
    }

    pwm_enabled[channel_id] = enable;

    return true;
}

/**
 * @brief  获取PWM通道状态
 * @param  channel_id 通道ID
 * @retval pwm_status_t PWM状态
 */
pwm_status_t pwm_hal_get_status(pwm_channel_id_t channel_id)
{
    pwm_status_t status = {0};

    if (channel_id < PWM_CHANNEL_MAX) {
        status.enabled = pwm_enabled[channel_id];
        status.duty_cycle = pwm_duty_cache[channel_id];
        status.frequency = pwm_channel_configs[channel_id].frequency;
        status.fault = false; // 简化处理
    }

    return status;
}

/**
 * @brief  获取PWM占空比
 * @param  channel_id 通道ID
 * @retval uint16_t 当前占空比
 */
uint16_t pwm_hal_get_duty_cycle(pwm_channel_id_t channel_id)
{
    if (channel_id >= PWM_CHANNEL_MAX) {
        return 0;
    }

    return pwm_duty_cache[channel_id];
}

/**
 * @brief  设置PWM频率
 * @param  channel_id 通道ID
 * @param  frequency 频率 (Hz)
 * @retval bool 设置成功返回true
 */
bool pwm_hal_set_frequency(pwm_channel_id_t channel_id, uint32_t frequency)
{
    if (channel_id >= PWM_CHANNEL_MAX || frequency == 0) {
        return false;
    }

    const pwm_channel_config_t *config = &pwm_channel_configs[channel_id];

    // 重新配置定时器 (影响同一定时器的所有通道)
    init_timer_pwm(config->timer_periph, frequency);

    return true;
}

/**
 * @brief  紧急停止所有PWM输出
 * @param  None
 * @retval None
 */
void pwm_hal_emergency_stop(void)
{
    // 立即关闭所有PWM输出
    for (int i = 0; i < PWM_CHANNEL_MAX; i++) {
        pwm_hal_set_duty_cycle(i, 0);
        pwm_hal_enable_channel(i, false);
    }
}