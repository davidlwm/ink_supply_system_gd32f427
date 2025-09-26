/**
 * @file    spi_hal.h
 * @brief   SPI硬件抽象层头文件 - LCD显示
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __SPI_HAL_H
#define __SPI_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"

// 最大SPI通道数
#define MAX_SPI_CHANNELS        3

// SPI通道定义
typedef enum {
    SPI_CHANNEL_0 = 0,          // SPI0 - LCD显示
    SPI_CHANNEL_1,              // SPI1 - 扩展SPI1
    SPI_CHANNEL_2               // SPI2 - 扩展SPI2
} spi_channel_t;

// SPI结果定义
typedef enum {
    SPI_SUCCESS = 0,
    SPI_ERROR_INVALID_PARAMETER,
    SPI_ERROR_NOT_INITIALIZED,
    SPI_ERROR_ALREADY_INITIALIZED,
    SPI_ERROR_TIMEOUT,
    SPI_ERROR_INVALID_OPERATION,
    SPI_ERROR_HARDWARE_FAULT
} spi_result_t;

// SPI CS控制方式
typedef enum {
    SPI_CS_HARDWARE = 0,        // 硬件CS控制
    SPI_CS_SOFTWARE             // 软件CS控制
} spi_cs_control_t;

// SPI配置结构
typedef struct {
    uint32_t mode;              // 传输模式
    uint32_t data_size;         // 数据帧大小
    uint32_t clock_polarity;    // 时钟极性
    uint32_t clock_phase;       // 时钟相位
    uint32_t prescaler;         // 时钟分频
    uint32_t endian;            // 字节序
    spi_cs_control_t cs_control; // CS控制方式
} spi_config_t;

// SPI GPIO配置结构 (内部使用)
typedef struct {
    uint32_t sck_port;
    uint32_t sck_pin;
    uint32_t miso_port;
    uint32_t miso_pin;
    uint32_t mosi_port;
    uint32_t mosi_pin;
    uint32_t cs_port;
    uint32_t cs_pin;
    uint32_t gpio_af;
    rcu_periph_enum rcu_gpio;
    rcu_periph_enum rcu_spi;
} spi_gpio_config_t;

// SPI初始化和反初始化
spi_result_t spi_hal_init(spi_channel_t channel, const spi_config_t* config);
spi_result_t spi_hal_deinit(spi_channel_t channel);

// SPI数据传输
spi_result_t spi_hal_transfer(spi_channel_t channel, const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, uint32_t timeout);
spi_result_t spi_hal_send(spi_channel_t channel, const uint8_t* data, uint16_t length, uint32_t timeout);
spi_result_t spi_hal_receive(spi_channel_t channel, uint8_t* data, uint16_t length, uint32_t timeout);

// CS控制
spi_result_t spi_hal_cs_low(spi_channel_t channel);
spi_result_t spi_hal_cs_high(spi_channel_t channel);

// 单字节读写
uint8_t spi_hal_read_write_byte(spi_channel_t channel, uint8_t data);

// 预定义配置
#define SPI_CONFIG_LCD_DEFAULT  { \
    .mode = SPI_TRANSMODE_FULLDUPLEX, \
    .data_size = SPI_FRAMESIZE_8BIT, \
    .clock_polarity = SPI_CK_PL_LOW, \
    .clock_phase = SPI_CK_PH_1EDGE, \
    .prescaler = SPI_PSC_8, \
    .endian = SPI_ENDIAN_MSB, \
    .cs_control = SPI_CS_SOFTWARE \
}

#define SPI_CONFIG_FAST_DEFAULT { \
    .mode = SPI_TRANSMODE_FULLDUPLEX, \
    .data_size = SPI_FRAMESIZE_8BIT, \
    .clock_polarity = SPI_CK_PL_LOW, \
    .clock_phase = SPI_CK_PH_1EDGE, \
    .prescaler = SPI_PSC_2, \
    .endian = SPI_ENDIAN_MSB, \
    .cs_control = SPI_CS_SOFTWARE \
}

#endif /* __SPI_HAL_H */