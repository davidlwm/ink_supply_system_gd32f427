/**
 * @file    gpio_hal.c
 * @brief   GPIO硬件抽象层实现 - 基于GD32F427
 * @version V4.0
 * @date    2025-09-27
 */

#include "gpio_hal.h"
#include "gd32f4xx.h"
#include "system_config.h"

// GPIO配置表
typedef struct {
    uint32_t gpio_port;         // GPIO端口
    uint32_t gpio_pin;          // GPIO引脚
    uint32_t gpio_mode;         // GPIO模式
    uint32_t gpio_pupd;         // 上下拉配置
    uint32_t gpio_otype;        // 输出类型
    uint32_t gpio_ospeed;       // 输出速度
} gpio_config_t;

// 数字输出引脚配置表
static const gpio_config_t gpio_output_configs[GPIO_OUTPUT_MAX] = {
    // 加热器控制引脚
    {GPIOB, GPIO_PIN_12, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 加热器1
    {GPIOB, GPIO_PIN_13, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 加热器2
    {GPIOB, GPIO_PIN_14, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 加热器3

    // 电磁阀控制引脚
    {GPIOD, GPIO_PIN_0,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 阀门1
    {GPIOD, GPIO_PIN_1,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 阀门2
    {GPIOD, GPIO_PIN_2,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 阀门3
    {GPIOD, GPIO_PIN_3,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 阀门4
    {GPIOD, GPIO_PIN_4,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 阀门5
    {GPIOD, GPIO_PIN_5,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 阀门6
    {GPIOD, GPIO_PIN_6,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 阀门7
    {GPIOD, GPIO_PIN_7,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 阀门8

    // LED控制引脚
    {GPIOE, GPIO_PIN_0,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // LED1 (电源)
    {GPIOE, GPIO_PIN_1,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // LED2 (网络)
    {GPIOE, GPIO_PIN_2,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // LED3 (运行)
    {GPIOE, GPIO_PIN_3,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // LED4 (通信)
    {GPIOE, GPIO_PIN_4,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // LED5 (故障)

    // 系统控制引脚
    {GPIOF, GPIO_PIN_0,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 系统使能
    {GPIOF, GPIO_PIN_1,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ},  // 看门狗喂狗
};

// 数字输入引脚配置表
static const gpio_config_t gpio_input_configs[GPIO_INPUT_MAX] = {
    // 限位开关输入
    {GPIOG, GPIO_PIN_0,  GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 液位开关1
    {GPIOG, GPIO_PIN_1,  GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 液位开关2
    {GPIOG, GPIO_PIN_2,  GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 液位开关3
    {GPIOG, GPIO_PIN_3,  GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 液位开关4

    // 系统状态输入
    {GPIOG, GPIO_PIN_8,  GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 紧急停止
    {GPIOG, GPIO_PIN_9,  GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 系统复位
    {GPIOG, GPIO_PIN_10, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 外部故障
    {GPIOG, GPIO_PIN_11, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 门锁检测

    // 备用输入
    {GPIOG, GPIO_PIN_12, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 备用输入1
    {GPIOG, GPIO_PIN_13, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, 0, 0},  // 备用输入2
};

// GPIO状态缓存
static volatile bool gpio_output_states[GPIO_OUTPUT_MAX];
static volatile bool gpio_input_states[GPIO_INPUT_MAX];

/**
 * @brief  GPIO HAL初始化
 * @param  None
 * @retval None
 */
void gpio_hal_init(void)
{
    // 使能GPIO时钟
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOG);

    // 初始化输出引脚
    init_gpio_outputs();

    // 初始化输入引脚
    init_gpio_inputs();

    // 初始化状态缓存
    for (int i = 0; i < GPIO_OUTPUT_MAX; i++) {
        gpio_output_states[i] = false;
    }

    for (int i = 0; i < GPIO_INPUT_MAX; i++) {
        gpio_input_states[i] = false;
    }
}

/**
 * @brief  初始化GPIO输出引脚
 * @param  None
 * @retval None
 */
static void init_gpio_outputs(void)
{
    for (int i = 0; i < GPIO_OUTPUT_MAX; i++) {
        const gpio_config_t *config = &gpio_output_configs[i];

        // 配置GPIO模式
        gpio_mode_set(config->gpio_port, config->gpio_mode, config->gpio_pupd, config->gpio_pin);

        // 配置输出选项
        if (config->gpio_mode == GPIO_MODE_OUTPUT) {
            gpio_output_options_set(config->gpio_port, config->gpio_otype, config->gpio_ospeed, config->gpio_pin);
        }

        // 初始状态为低电平
        gpio_bit_reset(config->gpio_port, config->gpio_pin);
    }
}

/**
 * @brief  初始化GPIO输入引脚
 * @param  None
 * @retval None
 */
static void init_gpio_inputs(void)
{
    for (int i = 0; i < GPIO_INPUT_MAX; i++) {
        const gpio_config_t *config = &gpio_input_configs[i];

        // 配置GPIO模式
        gpio_mode_set(config->gpio_port, config->gpio_mode, config->gpio_pupd, config->gpio_pin);
    }
}

/**
 * @brief  配置GPIO输出引脚
 * @param  pin_id 引脚ID
 * @retval bool 配置成功返回true
 */
bool gpio_hal_config_output(gpio_output_id_t pin_id)
{
    if (pin_id >= GPIO_OUTPUT_MAX) {
        return false;
    }

    const gpio_config_t *config = &gpio_output_configs[pin_id];

    gpio_mode_set(config->gpio_port, GPIO_MODE_OUTPUT, config->gpio_pupd, config->gpio_pin);
    gpio_output_options_set(config->gpio_port, config->gpio_otype, config->gpio_ospeed, config->gpio_pin);

    return true;
}

/**
 * @brief  配置GPIO输入引脚
 * @param  pin_id 引脚ID
 * @retval bool 配置成功返回true
 */
bool gpio_hal_config_input(gpio_input_id_t pin_id)
{
    if (pin_id >= GPIO_INPUT_MAX) {
        return false;
    }

    const gpio_config_t *config = &gpio_input_configs[pin_id];

    gpio_mode_set(config->gpio_port, GPIO_MODE_INPUT, config->gpio_pupd, config->gpio_pin);

    return true;
}

/**
 * @brief  写GPIO输出引脚
 * @param  pin_id 引脚ID
 * @param  state 输出状态
 * @retval bool 写入成功返回true
 */
bool gpio_hal_write_pin(gpio_output_id_t pin_id, bool state)
{
    if (pin_id >= GPIO_OUTPUT_MAX) {
        return false;
    }

    const gpio_config_t *config = &gpio_output_configs[pin_id];

    if (state) {
        gpio_bit_set(config->gpio_port, config->gpio_pin);
    } else {
        gpio_bit_reset(config->gpio_port, config->gpio_pin);
    }

    // 更新状态缓存
    gpio_output_states[pin_id] = state;

    return true;
}

/**
 * @brief  读GPIO输入引脚
 * @param  pin_id 引脚ID
 * @retval bool 引脚状态 (true=高电平, false=低电平)
 */
bool gpio_hal_read_pin(gpio_input_id_t pin_id)
{
    if (pin_id >= GPIO_INPUT_MAX) {
        return false;
    }

    const gpio_config_t *config = &gpio_input_configs[pin_id];

    bool state = (gpio_input_bit_get(config->gpio_port, config->gpio_pin) == SET);

    // 更新状态缓存
    gpio_input_states[pin_id] = state;

    return state;
}

/**
 * @brief  切换GPIO输出引脚
 * @param  pin_id 引脚ID
 * @retval bool 切换成功返回true
 */
bool gpio_hal_toggle_pin(gpio_output_id_t pin_id)
{
    if (pin_id >= GPIO_OUTPUT_MAX) {
        return false;
    }

    bool current_state = gpio_output_states[pin_id];
    return gpio_hal_write_pin(pin_id, !current_state);
}

/**
 * @brief  读取GPIO输出引脚状态
 * @param  pin_id 引脚ID
 * @retval bool 输出状态
 */
bool gpio_hal_read_output_pin(gpio_output_id_t pin_id)
{
    if (pin_id >= GPIO_OUTPUT_MAX) {
        return false;
    }

    return gpio_output_states[pin_id];
}

/**
 * @brief  批量设置GPIO输出
 * @param  pin_mask 引脚掩码
 * @param  state 输出状态
 * @retval None
 */
void gpio_hal_write_port(uint32_t pin_mask, bool state)
{
    for (int i = 0; i < GPIO_OUTPUT_MAX; i++) {
        if (pin_mask & (1 << i)) {
            gpio_hal_write_pin(i, state);
        }
    }
}

/**
 * @brief  批量读取GPIO输入
 * @param  None
 * @retval uint32_t 输入状态掩码
 */
uint32_t gpio_hal_read_port(void)
{
    uint32_t port_value = 0;

    for (int i = 0; i < GPIO_INPUT_MAX; i++) {
        if (gpio_hal_read_pin(i)) {
            port_value |= (1 << i);
        }
    }

    return port_value;
}

/**
 * @brief  获取数字输入状态字 (用于EtherCAT)
 * @param  None
 * @retval uint16_t 数字输入状态
 */
uint16_t gpio_hal_get_digital_inputs(void)
{
    uint16_t inputs = 0;

    // 读取所有输入状态
    for (int i = 0; i < GPIO_INPUT_MAX && i < 16; i++) {
        if (gpio_hal_read_pin(i)) {
            inputs |= (1 << i);
        }
    }

    return inputs;
}

/**
 * @brief  设置数字输出状态字 (用于EtherCAT)
 * @param  outputs 数字输出状态
 * @retval None
 */
void gpio_hal_set_digital_outputs(uint16_t outputs)
{
    // 设置阀门输出 (位0-7)
    for (int i = 0; i < 8; i++) {
        gpio_hal_write_pin(GPIO_VALVE_1 + i, (outputs & (1 << i)) != 0);
    }

    // 设置其他控制输出 (位8-15)
    for (int i = 8; i < 16 && (GPIO_HEATER_1 + i - 8) < GPIO_OUTPUT_MAX; i++) {
        gpio_hal_write_pin(GPIO_HEATER_1 + i - 8, (outputs & (1 << i)) != 0);
    }
}

/**
 * @brief  紧急关闭所有输出
 * @param  None
 * @retval None
 */
void gpio_hal_emergency_shutdown(void)
{
    // 立即关闭所有输出
    for (int i = 0; i < GPIO_OUTPUT_MAX; i++) {
        gpio_hal_write_pin(i, false);
    }
}