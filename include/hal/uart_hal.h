/**
 * @file    uart_hal.h
 * @brief   UART硬件抽象层头文件 - 调试+通信
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __UART_HAL_H
#define __UART_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"

// 最大UART通道数
#define MAX_UART_CHANNELS       4

// 缓冲区大小
#define UART_TX_BUFFER_SIZE     256
#define UART_RX_BUFFER_SIZE     256

// UART通道定义
typedef enum {
    UART_CHANNEL_0 = 0,         // USART0 - 调试串口
    UART_CHANNEL_1,             // USART1 - 通信串口1
    UART_CHANNEL_2,             // USART2 - 通信串口2
    UART_CHANNEL_3              // UART3  - 通信串口3
} uart_channel_t;

// UART结果定义
typedef enum {
    UART_SUCCESS = 0,
    UART_ERROR_INVALID_PARAMETER,
    UART_ERROR_NOT_INITIALIZED,
    UART_ERROR_ALREADY_INITIALIZED,
    UART_ERROR_TIMEOUT,
    UART_ERROR_BUFFER_FULL,
    UART_ERROR_HARDWARE_FAULT
} uart_result_t;

// UART配置结构
typedef struct {
    uint32_t baudrate;              // 波特率
    uint32_t word_length;           // 数据位长度
    uint32_t stop_bits;             // 停止位
    uint32_t parity;                // 校验位
    bool flow_control;              // 硬件流控
    bool use_interrupt;             // 使用中断模式
} uart_config_t;

// UART GPIO配置结构 (内部使用)
typedef struct {
    uint32_t tx_port;
    uint32_t tx_pin;
    uint32_t rx_port;
    uint32_t rx_pin;
    uint32_t gpio_af;
    rcu_periph_enum rcu_gpio;
    rcu_periph_enum rcu_uart;
} uart_gpio_config_t;

// UART初始化和反初始化
uart_result_t uart_hal_init(uart_channel_t channel, const uart_config_t* config);
uart_result_t uart_hal_deinit(uart_channel_t channel);

// 阻塞模式收发
uart_result_t uart_hal_send(uart_channel_t channel, const uint8_t* data, uint16_t length, uint32_t timeout);
uart_result_t uart_hal_receive(uart_channel_t channel, uint8_t* data, uint16_t length, uint16_t* received, uint32_t timeout);

// 非阻塞模式收发
uart_result_t uart_hal_send_async(uart_channel_t channel, const uint8_t* data, uint16_t length);
uart_result_t uart_hal_receive_async(uart_channel_t channel, uint8_t* data, uint16_t length, uint16_t* received);

// 缓冲区管理
uint16_t uart_hal_get_rx_count(uart_channel_t channel);
uint16_t uart_hal_get_tx_free(uart_channel_t channel);
uart_result_t uart_hal_flush_rx(uart_channel_t channel);
uart_result_t uart_hal_flush_tx(uart_channel_t channel);

// 中断处理 (需要在中断向量中调用)
void uart_hal_irq_handler(uart_channel_t channel);

// 系统时钟函数 (需要用户实现)
uint32_t get_systick_ms(void);

// 预定义配置
#define UART_CONFIG_DEBUG_DEFAULT   {115200, USART_WL_8BIT, USART_STB_1BIT, USART_PM_NONE, false, true}
#define UART_CONFIG_COMM_DEFAULT    {9600, USART_WL_8BIT, USART_STB_1BIT, USART_PM_NONE, false, true}
#define UART_CONFIG_RS485_DEFAULT   {9600, USART_WL_8BIT, USART_STB_1BIT, USART_PM_NONE, true, true}

#endif /* __UART_HAL_H */