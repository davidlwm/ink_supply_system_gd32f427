/**
 * @file    uart_hal.c
 * @brief   UART硬件抽象层实现 - 调试+通信
 * @version V4.0
 * @date    2025-09-27
 */

#include "uart_hal.h"
#include "gd32f4xx.h"
#include <string.h>

// UART配置信息
static uart_config_t uart_configs[MAX_UART_CHANNELS];
static bool uart_initialized[MAX_UART_CHANNELS] = {false};

// DMA缓冲区
static uint8_t uart_tx_buffer[MAX_UART_CHANNELS][UART_TX_BUFFER_SIZE];
static uint8_t uart_rx_buffer[MAX_UART_CHANNELS][UART_RX_BUFFER_SIZE];
static volatile uint16_t uart_tx_head[MAX_UART_CHANNELS] = {0};
static volatile uint16_t uart_tx_tail[MAX_UART_CHANNELS] = {0};
static volatile uint16_t uart_rx_head[MAX_UART_CHANNELS] = {0};
static volatile uint16_t uart_rx_tail[MAX_UART_CHANNELS] = {0};

// UART外设映射
static const uint32_t uart_periph_map[MAX_UART_CHANNELS] = {
    USART0,     // UART0 - 调试串口
    USART1,     // UART1 - 通信串口1
    USART2,     // UART2 - 通信串口2
    UART3       // UART3 - 通信串口3
};

// GPIO配置映射
static const uart_gpio_config_t uart_gpio_configs[MAX_UART_CHANNELS] = {
    // UART0 - PA9/PA10
    {
        .tx_port = GPIOA, .tx_pin = GPIO_PIN_9,
        .rx_port = GPIOA, .rx_pin = GPIO_PIN_10,
        .gpio_af = GPIO_AF_7, .rcu_gpio = RCU_GPIOA, .rcu_uart = RCU_USART0
    },
    // UART1 - PA2/PA3
    {
        .tx_port = GPIOA, .tx_pin = GPIO_PIN_2,
        .rx_port = GPIOA, .rx_pin = GPIO_PIN_3,
        .gpio_af = GPIO_AF_7, .rcu_gpio = RCU_GPIOA, .rcu_uart = RCU_USART1
    },
    // UART2 - PB10/PB11
    {
        .tx_port = GPIOB, .tx_pin = GPIO_PIN_10,
        .rx_port = GPIOB, .rx_pin = GPIO_PIN_11,
        .gpio_af = GPIO_AF_7, .rcu_gpio = RCU_GPIOB, .rcu_uart = RCU_USART2
    },
    // UART3 - PC10/PC11
    {
        .tx_port = GPIOC, .tx_pin = GPIO_PIN_10,
        .rx_port = GPIOC, .rx_pin = GPIO_PIN_11,
        .gpio_af = GPIO_AF_8, .rcu_gpio = RCU_GPIOC, .rcu_uart = RCU_UART3
    }
};

/**
 * @brief  UART HAL初始化
 * @param  channel: UART通道
 * @param  config: 配置参数
 * @retval uart_result_t 初始化结果
 */
uart_result_t uart_hal_init(uart_channel_t channel, const uart_config_t* config)
{
    if (channel >= MAX_UART_CHANNELS || config == NULL) {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (uart_initialized[channel]) {
        return UART_ERROR_ALREADY_INITIALIZED;
    }

    // 保存配置
    memcpy(&uart_configs[channel], config, sizeof(uart_config_t));

    // 使能时钟
    const uart_gpio_config_t* gpio_cfg = &uart_gpio_configs[channel];
    rcu_periph_clock_enable(gpio_cfg->rcu_gpio);
    rcu_periph_clock_enable(gpio_cfg->rcu_uart);

    // 配置GPIO
    gpio_af_set(gpio_cfg->tx_port, gpio_cfg->gpio_af, gpio_cfg->tx_pin);
    gpio_af_set(gpio_cfg->rx_port, gpio_cfg->gpio_af, gpio_cfg->rx_pin);

    gpio_mode_set(gpio_cfg->tx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, gpio_cfg->tx_pin);
    gpio_output_options_set(gpio_cfg->tx_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, gpio_cfg->tx_pin);

    gpio_mode_set(gpio_cfg->rx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, gpio_cfg->rx_pin);

    // 配置UART参数
    uint32_t uart_periph = uart_periph_map[channel];

    usart_deinit(uart_periph);
    usart_baudrate_set(uart_periph, config->baudrate);
    usart_word_length_set(uart_periph, config->word_length);
    usart_stop_bit_set(uart_periph, config->stop_bits);
    usart_parity_config(uart_periph, config->parity);
    usart_hardware_flow_cts_config(uart_periph, config->flow_control ? USART_CTS_ENABLE : USART_CTS_DISABLE);
    usart_hardware_flow_rts_config(uart_periph, config->flow_control ? USART_RTS_ENABLE : USART_RTS_DISABLE);
    usart_receive_config(uart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(uart_periph, USART_TRANSMIT_ENABLE);

    // 配置中断
    if (config->use_interrupt) {
        usart_interrupt_enable(uart_periph, USART_INT_RBNE);
        usart_interrupt_enable(uart_periph, USART_INT_TBE);

        // 配置NVIC
        switch (channel) {
            case UART_CHANNEL_0:
                nvic_irq_enable(USART0_IRQn, 1, 0);
                break;
            case UART_CHANNEL_1:
                nvic_irq_enable(USART1_IRQn, 1, 0);
                break;
            case UART_CHANNEL_2:
                nvic_irq_enable(USART2_IRQn, 1, 0);
                break;
            case UART_CHANNEL_3:
                nvic_irq_enable(UART3_IRQn, 1, 0);
                break;
        }
    }

    // 使能UART
    usart_enable(uart_periph);

    // 初始化缓冲区
    uart_tx_head[channel] = 0;
    uart_tx_tail[channel] = 0;
    uart_rx_head[channel] = 0;
    uart_rx_tail[channel] = 0;

    uart_initialized[channel] = true;

    return UART_SUCCESS;
}

/**
 * @brief  UART发送数据 (阻塞)
 * @param  channel: UART通道
 * @param  data: 数据指针
 * @param  length: 数据长度
 * @param  timeout: 超时时间(毫秒)
 * @retval uart_result_t 发送结果
 */
uart_result_t uart_hal_send(uart_channel_t channel, const uint8_t* data, uint16_t length, uint32_t timeout)
{
    if (channel >= MAX_UART_CHANNELS || data == NULL || length == 0) {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!uart_initialized[channel]) {
        return UART_ERROR_NOT_INITIALIZED;
    }

    uint32_t uart_periph = uart_periph_map[channel];
    uint32_t start_time = get_systick_ms(); // 需要实现系统时钟函数

    for (uint16_t i = 0; i < length; i++) {
        // 等待发送缓冲区空
        while (RESET == usart_flag_get(uart_periph, USART_FLAG_TBE)) {
            if (timeout > 0 && (get_systick_ms() - start_time) > timeout) {
                return UART_ERROR_TIMEOUT;
            }
        }

        usart_data_transmit(uart_periph, data[i]);
    }

    // 等待发送完成
    while (RESET == usart_flag_get(uart_periph, USART_FLAG_TC)) {
        if (timeout > 0 && (get_systick_ms() - start_time) > timeout) {
            return UART_ERROR_TIMEOUT;
        }
    }

    return UART_SUCCESS;
}

/**
 * @brief  UART接收数据 (阻塞)
 * @param  channel: UART通道
 * @param  data: 数据指针
 * @param  length: 期望长度
 * @param  received: 实际接收长度
 * @param  timeout: 超时时间(毫秒)
 * @retval uart_result_t 接收结果
 */
uart_result_t uart_hal_receive(uart_channel_t channel, uint8_t* data, uint16_t length, uint16_t* received, uint32_t timeout)
{
    if (channel >= MAX_UART_CHANNELS || data == NULL || length == 0 || received == NULL) {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!uart_initialized[channel]) {
        return UART_ERROR_NOT_INITIALIZED;
    }

    uint32_t uart_periph = uart_periph_map[channel];
    uint32_t start_time = get_systick_ms();
    *received = 0;

    for (uint16_t i = 0; i < length; i++) {
        // 等待接收数据
        while (RESET == usart_flag_get(uart_periph, USART_FLAG_RBNE)) {
            if (timeout > 0 && (get_systick_ms() - start_time) > timeout) {
                return (*received > 0) ? UART_SUCCESS : UART_ERROR_TIMEOUT;
            }
        }

        data[i] = usart_data_receive(uart_periph);
        (*received)++;
    }

    return UART_SUCCESS;
}

/**
 * @brief  UART发送数据 (非阻塞)
 * @param  channel: UART通道
 * @param  data: 数据指针
 * @param  length: 数据长度
 * @retval uart_result_t 发送结果
 */
uart_result_t uart_hal_send_async(uart_channel_t channel, const uint8_t* data, uint16_t length)
{
    if (channel >= MAX_UART_CHANNELS || data == NULL || length == 0) {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!uart_initialized[channel] || !uart_configs[channel].use_interrupt) {
        return UART_ERROR_NOT_INITIALIZED;
    }

    // 将数据写入发送缓冲区
    for (uint16_t i = 0; i < length; i++) {
        uint16_t next_head = (uart_tx_head[channel] + 1) % UART_TX_BUFFER_SIZE;

        if (next_head == uart_tx_tail[channel]) {
            return UART_ERROR_BUFFER_FULL; // 缓冲区满
        }

        uart_tx_buffer[channel][uart_tx_head[channel]] = data[i];
        uart_tx_head[channel] = next_head;
    }

    // 启动发送中断
    uint32_t uart_periph = uart_periph_map[channel];
    usart_interrupt_enable(uart_periph, USART_INT_TBE);

    return UART_SUCCESS;
}

/**
 * @brief  UART接收数据 (非阻塞)
 * @param  channel: UART通道
 * @param  data: 数据指针
 * @param  length: 缓冲区大小
 * @param  received: 实际接收长度
 * @retval uart_result_t 接收结果
 */
uart_result_t uart_hal_receive_async(uart_channel_t channel, uint8_t* data, uint16_t length, uint16_t* received)
{
    if (channel >= MAX_UART_CHANNELS || data == NULL || length == 0 || received == NULL) {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!uart_initialized[channel] || !uart_configs[channel].use_interrupt) {
        return UART_ERROR_NOT_INITIALIZED;
    }

    *received = 0;

    // 从接收缓冲区读取数据
    while (uart_rx_tail[channel] != uart_rx_head[channel] && *received < length) {
        data[*received] = uart_rx_buffer[channel][uart_rx_tail[channel]];
        uart_rx_tail[channel] = (uart_rx_tail[channel] + 1) % UART_RX_BUFFER_SIZE;
        (*received)++;
    }

    return UART_SUCCESS;
}

/**
 * @brief  获取可用接收数据长度
 * @param  channel: UART通道
 * @retval uint16_t 可用数据长度
 */
uint16_t uart_hal_get_rx_count(uart_channel_t channel)
{
    if (channel >= MAX_UART_CHANNELS || !uart_initialized[channel]) {
        return 0;
    }

    uint16_t head = uart_rx_head[channel];
    uint16_t tail = uart_rx_tail[channel];

    if (head >= tail) {
        return head - tail;
    } else {
        return (UART_RX_BUFFER_SIZE - tail) + head;
    }
}

/**
 * @brief  获取发送缓冲区空闲空间
 * @param  channel: UART通道
 * @retval uint16_t 空闲空间大小
 */
uint16_t uart_hal_get_tx_free(uart_channel_t channel)
{
    if (channel >= MAX_UART_CHANNELS || !uart_initialized[channel]) {
        return 0;
    }

    uint16_t head = uart_tx_head[channel];
    uint16_t tail = uart_tx_tail[channel];

    if (head >= tail) {
        return (UART_TX_BUFFER_SIZE - 1) - (head - tail);
    } else {
        return (tail - head) - 1;
    }
}

/**
 * @brief  清空接收缓冲区
 * @param  channel: UART通道
 * @retval uart_result_t 清空结果
 */
uart_result_t uart_hal_flush_rx(uart_channel_t channel)
{
    if (channel >= MAX_UART_CHANNELS || !uart_initialized[channel]) {
        return UART_ERROR_INVALID_PARAMETER;
    }

    uart_rx_head[channel] = 0;
    uart_rx_tail[channel] = 0;

    return UART_SUCCESS;
}

/**
 * @brief  清空发送缓冲区
 * @param  channel: UART通道
 * @retval uart_result_t 清空结果
 */
uart_result_t uart_hal_flush_tx(uart_channel_t channel)
{
    if (channel >= MAX_UART_CHANNELS || !uart_initialized[channel]) {
        return UART_ERROR_INVALID_PARAMETER;
    }

    uart_tx_head[channel] = 0;
    uart_tx_tail[channel] = 0;

    return UART_SUCCESS;
}

/**
 * @brief  UART中断处理函数 (需要在中断向量中调用)
 * @param  channel: UART通道
 * @retval None
 */
void uart_hal_irq_handler(uart_channel_t channel)
{
    if (channel >= MAX_UART_CHANNELS || !uart_initialized[channel]) {
        return;
    }

    uint32_t uart_periph = uart_periph_map[channel];

    // 处理接收中断
    if (RESET != usart_interrupt_flag_get(uart_periph, USART_INT_FLAG_RBNE)) {
        uint8_t data = usart_data_receive(uart_periph);

        uint16_t next_head = (uart_rx_head[channel] + 1) % UART_RX_BUFFER_SIZE;
        if (next_head != uart_rx_tail[channel]) {
            uart_rx_buffer[channel][uart_rx_head[channel]] = data;
            uart_rx_head[channel] = next_head;
        }
        // 如果缓冲区满，丢弃数据
    }

    // 处理发送中断
    if (RESET != usart_interrupt_flag_get(uart_periph, USART_INT_FLAG_TBE)) {
        if (uart_tx_tail[channel] != uart_tx_head[channel]) {
            usart_data_transmit(uart_periph, uart_tx_buffer[channel][uart_tx_tail[channel]]);
            uart_tx_tail[channel] = (uart_tx_tail[channel] + 1) % UART_TX_BUFFER_SIZE;
        } else {
            // 发送缓冲区空，禁用发送中断
            usart_interrupt_disable(uart_periph, USART_INT_TBE);
        }
    }
}

/**
 * @brief  UART反初始化
 * @param  channel: UART通道
 * @retval uart_result_t 反初始化结果
 */
uart_result_t uart_hal_deinit(uart_channel_t channel)
{
    if (channel >= MAX_UART_CHANNELS) {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!uart_initialized[channel]) {
        return UART_ERROR_NOT_INITIALIZED;
    }

    uint32_t uart_periph = uart_periph_map[channel];

    // 禁用中断
    usart_interrupt_disable(uart_periph, USART_INT_RBNE);
    usart_interrupt_disable(uart_periph, USART_INT_TBE);

    // 禁用UART
    usart_disable(uart_periph);

    // 反初始化UART
    usart_deinit(uart_periph);

    uart_initialized[channel] = false;

    return UART_SUCCESS;
}

/**
 * @brief  获取系统毫秒时钟 (需要用户实现)
 * @param  None
 * @retval uint32_t 毫秒时钟
 */
__weak uint32_t get_systick_ms(void)
{
    // 这里需要用户提供系统毫秒时钟实现
    // 可以使用SysTick或其他定时器
    return 0;
}