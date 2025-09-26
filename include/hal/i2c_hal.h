/**
 * @file i2c_hal.h
 * @brief I2C硬件抽象层头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统I2C硬件抽象层，基于GD32F4xx HAL库
 */

#ifndef I2C_HAL_H
#define I2C_HAL_H

#include "hal/hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C传输模式 */
typedef enum {
    I2C_MODE_STANDARD = 0,      /* 标准模式 100kHz */
    I2C_MODE_FAST = 1,          /* 快速模式 400kHz */
    I2C_MODE_FAST_PLUS = 2      /* 快速+模式 1MHz */
} i2c_mode_t;

/* I2C寻址模式 */
typedef enum {
    I2C_ADDRESSING_7BIT = 0,    /* 7位寻址 */
    I2C_ADDRESSING_10BIT = 1    /* 10位寻址 */
} i2c_addressing_mode_t;

/* I2C传输方向 */
typedef enum {
    I2C_DIRECTION_TRANSMIT = 0, /* 发送 */
    I2C_DIRECTION_RECEIVE = 1   /* 接收 */
} i2c_direction_t;

/* I2C配置 */
typedef struct {
    hal_config_base_t base;         /* 基础配置 */
    uint32_t instance;              /* I2C实例 */
    uint32_t clock_speed;           /* 时钟速度 */
    i2c_mode_t mode;                /* 传输模式 */
    i2c_addressing_mode_t addr_mode; /* 寻址模式 */
    uint16_t own_address;           /* 自身地址 */
    bool general_call_enable;       /* 广播呼叫使能 */
    bool no_stretch_enable;         /* 禁止时钟延展 */
    uint8_t analog_filter;          /* 模拟滤波器 */
    uint8_t digital_filter;         /* 数字滤波器 */
    bool dma_enable;                /* DMA使能 */
    hal_callback_t master_tx_complete_callback; /* 主机发送完成回调 */
    hal_callback_t master_rx_complete_callback; /* 主机接收完成回调 */
    hal_callback_t slave_tx_complete_callback;  /* 从机发送完成回调 */
    hal_callback_t slave_rx_complete_callback;  /* 从机接收完成回调 */
    hal_callback_t error_callback;  /* 错误回调 */
} i2c_config_t;

/* I2C句柄 */
typedef struct {
    hal_handle_base_t base;     /* 基础句柄 */
    i2c_config_t* config;       /* I2C配置 */
    uint8_t* tx_buffer;         /* 发送缓冲区 */
    uint8_t* rx_buffer;         /* 接收缓冲区 */
    uint16_t tx_size;           /* 发送大小 */
    uint16_t rx_size;           /* 接收大小 */
    volatile uint16_t tx_count; /* 发送计数 */
    volatile uint16_t rx_count; /* 接收计数 */
    volatile bool transfer_complete; /* 传输完成标志 */
    uint32_t error_flags;       /* 错误标志 */
} i2c_handle_t;

/* 基础I2C操作接口 */
hal_result_t i2c_hal_init(i2c_handle_t* handle, const i2c_config_t* config);
hal_result_t i2c_hal_deinit(i2c_handle_t* handle);
hal_result_t i2c_hal_master_transmit(i2c_handle_t* handle, uint16_t dev_address, const uint8_t* data, uint16_t size, uint32_t timeout);
hal_result_t i2c_hal_master_receive(i2c_handle_t* handle, uint16_t dev_address, uint8_t* data, uint16_t size, uint32_t timeout);
hal_result_t i2c_hal_slave_transmit(i2c_handle_t* handle, const uint8_t* data, uint16_t size, uint32_t timeout);
hal_result_t i2c_hal_slave_receive(i2c_handle_t* handle, uint8_t* data, uint16_t size, uint32_t timeout);

/* 中断模式接口 */
hal_result_t i2c_hal_master_transmit_it(i2c_handle_t* handle, uint16_t dev_address, const uint8_t* data, uint16_t size);
hal_result_t i2c_hal_master_receive_it(i2c_handle_t* handle, uint16_t dev_address, uint8_t* data, uint16_t size);
hal_result_t i2c_hal_slave_transmit_it(i2c_handle_t* handle, const uint8_t* data, uint16_t size);
hal_result_t i2c_hal_slave_receive_it(i2c_handle_t* handle, uint8_t* data, uint16_t size);

/* DMA模式接口 */
hal_result_t i2c_hal_master_transmit_dma(i2c_handle_t* handle, uint16_t dev_address, const uint8_t* data, uint16_t size);
hal_result_t i2c_hal_master_receive_dma(i2c_handle_t* handle, uint16_t dev_address, uint8_t* data, uint16_t size);

/* 寄存器操作接口 */
hal_result_t i2c_hal_mem_write(i2c_handle_t* handle, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, const uint8_t* data, uint16_t size, uint32_t timeout);
hal_result_t i2c_hal_mem_read(i2c_handle_t* handle, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t* data, uint16_t size, uint32_t timeout);

/* 设备检测接口 */
hal_result_t i2c_hal_is_device_ready(i2c_handle_t* handle, uint16_t dev_address, uint32_t trials, uint32_t timeout);
hal_result_t i2c_hal_scan_devices(i2c_handle_t* handle, uint8_t* devices, uint8_t* count);

/* 供墨系统专用接口 */
hal_result_t i2c_hal_system_init(void);
hal_result_t i2c_hal_system_deinit(void);

/* 回调函数类型 */
typedef void (*i2c_master_tx_complete_callback_t)(i2c_handle_t* handle);
typedef void (*i2c_master_rx_complete_callback_t)(i2c_handle_t* handle);
typedef void (*i2c_error_callback_t)(i2c_handle_t* handle, hal_result_t error);

/* 回调注册接口 */
hal_result_t i2c_hal_register_master_tx_callback(i2c_handle_t* handle, i2c_master_tx_complete_callback_t callback);
hal_result_t i2c_hal_register_master_rx_callback(i2c_handle_t* handle, i2c_master_rx_complete_callback_t callback);
hal_result_t i2c_hal_register_error_callback(i2c_handle_t* handle, i2c_error_callback_t callback);
hal_result_t i2c_hal_unregister_callbacks(i2c_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif /* I2C_HAL_H */