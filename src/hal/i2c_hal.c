/**
 * @file i2c_hal.c
 * @brief I2C硬件抽象层实现
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统I2C硬件抽象层实现
 */

#include "hal/i2c_hal.h"
#include "hal/hal_common.h"
#include <string.h>

/* 私有变量 */
static bool g_i2c_system_initialized = false;

/**
 * @brief I2C句柄初始化
 * @param handle I2C句柄指针
 * @param config I2C配置指针
 * @return hal_result_t 操作结果
 */
hal_result_t i2c_hal_init(i2c_handle_t* handle, const i2c_config_t* config)
{
    HAL_CHECK_PARAM(handle);
    HAL_CHECK_PARAM(config);

    /* 初始化基础句柄 */
    hal_result_t result = hal_handle_init(&handle->base, &config->base);
    if (result != HAL_OK) {
        return result;
    }

    /* 设置I2C配置 */
    handle->config = (i2c_config_t*)config;
    handle->tx_buffer = NULL;
    handle->rx_buffer = NULL;
    handle->tx_size = 0;
    handle->rx_size = 0;
    handle->tx_count = 0;
    handle->rx_count = 0;
    handle->transfer_complete = false;
    handle->error_flags = 0;

    /* 配置I2C硬件 */
    rcu_periph_clock_enable(RCU_I2C0 + (config->instance & 0x0F));

    /* 配置I2C参数 */
    i2c_clock_config(config->instance, config->clock_speed, I2C_DTCY_2);
    i2c_mode_addr_config(config->instance, I2C_I2CMODE_ENABLE,
                        config->addr_mode == I2C_ADDRESSING_7BIT ? I2C_ADDFORMAT_7BITS : I2C_ADDFORMAT_10BITS,
                        config->own_address);
    i2c_enable(config->instance);

    return HAL_OK;
}

/**
 * @brief I2C句柄反初始化
 * @param handle I2C句柄指针
 * @return hal_result_t 操作结果
 */
hal_result_t i2c_hal_deinit(i2c_handle_t* handle)
{
    HAL_CHECK_HANDLE(handle);

    /* 禁用I2C */
    i2c_disable(handle->config->instance);

    /* 反初始化基础句柄 */
    return hal_handle_deinit(&handle->base);
}

/**
 * @brief I2C主机发送数据
 * @param handle I2C句柄指针
 * @param dev_address 设备地址
 * @param data 数据指针
 * @param size 数据大小
 * @param timeout 超时时间
 * @return hal_result_t 操作结果
 */
hal_result_t i2c_hal_master_transmit(i2c_handle_t* handle, uint16_t dev_address, const uint8_t* data, uint16_t size, uint32_t timeout)
{
    HAL_CHECK_HANDLE(handle);
    HAL_CHECK_PARAM(data);

    if (size == 0) {
        return HAL_INVALID_PARAM;
    }

    HAL_LOCK(handle);

    handle->base.state = HAL_STATE_BUSY;
    handle->error_flags = 0;

    /* 启动传输 */
    i2c_start_on_bus(handle->config->instance);

    /* 等待START条件 */
    uint32_t start_time = xTaskGetTickCount();
    while (!i2c_flag_get(handle->config->instance, I2C_FLAG_SBSEND)) {
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout)) {
            i2c_stop_on_bus(handle->config->instance);
            handle->base.state = HAL_STATE_READY;
            HAL_UNLOCK(handle);
            return HAL_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* 发送设备地址 */
    i2c_master_addressing(handle->config->instance, dev_address, I2C_TRANSMITTER);

    /* 等待地址确认 */
    start_time = xTaskGetTickCount();
    while (!i2c_flag_get(handle->config->instance, I2C_FLAG_ADDSEND)) {
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout)) {
            i2c_stop_on_bus(handle->config->instance);
            handle->base.state = HAL_STATE_READY;
            HAL_UNLOCK(handle);
            return HAL_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* 清除地址标志 */
    i2c_flag_clear(handle->config->instance, I2C_FLAG_ADDSEND);

    /* 发送数据 */
    for (uint16_t i = 0; i < size; i++) {
        i2c_data_transmit(handle->config->instance, data[i]);

        start_time = xTaskGetTickCount();
        while (!i2c_flag_get(handle->config->instance, I2C_FLAG_TBE)) {
            if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout)) {
                i2c_stop_on_bus(handle->config->instance);
                handle->base.state = HAL_STATE_READY;
                HAL_UNLOCK(handle);
                return HAL_TIMEOUT;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    /* 发送STOP条件 */
    i2c_stop_on_bus(handle->config->instance);

    handle->base.state = HAL_STATE_READY;
    HAL_UPDATE_STATS_SUCCESS(handle);
    HAL_UNLOCK(handle);

    return HAL_OK;
}

/**
 * @brief I2C主机接收数据
 * @param handle I2C句柄指针
 * @param dev_address 设备地址
 * @param data 数据缓冲区
 * @param size 数据大小
 * @param timeout 超时时间
 * @return hal_result_t 操作结果
 */
hal_result_t i2c_hal_master_receive(i2c_handle_t* handle, uint16_t dev_address, uint8_t* data, uint16_t size, uint32_t timeout)
{
    HAL_CHECK_HANDLE(handle);
    HAL_CHECK_PARAM(data);

    if (size == 0) {
        return HAL_INVALID_PARAM;
    }

    HAL_LOCK(handle);

    handle->base.state = HAL_STATE_BUSY;
    handle->error_flags = 0;

    /* 启动传输 */
    i2c_start_on_bus(handle->config->instance);

    /* 等待START条件 */
    uint32_t start_time = xTaskGetTickCount();
    while (!i2c_flag_get(handle->config->instance, I2C_FLAG_SBSEND)) {
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout)) {
            i2c_stop_on_bus(handle->config->instance);
            handle->base.state = HAL_STATE_READY;
            HAL_UNLOCK(handle);
            return HAL_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* 发送设备地址 */
    i2c_master_addressing(handle->config->instance, dev_address, I2C_RECEIVER);

    /* 等待地址确认 */
    start_time = xTaskGetTickCount();
    while (!i2c_flag_get(handle->config->instance, I2C_FLAG_ADDSEND)) {
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout)) {
            i2c_stop_on_bus(handle->config->instance);
            handle->base.state = HAL_STATE_READY;
            HAL_UNLOCK(handle);
            return HAL_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* 清除地址标志 */
    i2c_flag_clear(handle->config->instance, I2C_FLAG_ADDSEND);

    /* 接收数据 */
    for (uint16_t i = 0; i < size; i++) {
        if (i == size - 1) {
            /* 最后一个字节发送NACK */
            i2c_ack_config(handle->config->instance, I2C_ACK_DISABLE);
            i2c_stop_on_bus(handle->config->instance);
        }

        start_time = xTaskGetTickCount();
        while (!i2c_flag_get(handle->config->instance, I2C_FLAG_RBNE)) {
            if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout)) {
                i2c_stop_on_bus(handle->config->instance);
                handle->base.state = HAL_STATE_READY;
                HAL_UNLOCK(handle);
                return HAL_TIMEOUT;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        data[i] = i2c_data_receive(handle->config->instance);
    }

    /* 重新使能ACK */
    i2c_ack_config(handle->config->instance, I2C_ACK_ENABLE);

    handle->base.state = HAL_STATE_READY;
    HAL_UPDATE_STATS_SUCCESS(handle);
    HAL_UNLOCK(handle);

    return HAL_OK;
}

/**
 * @brief I2C设备检测
 * @param handle I2C句柄指针
 * @param dev_address 设备地址
 * @param trials 尝试次数
 * @param timeout 超时时间
 * @return hal_result_t 操作结果
 */
hal_result_t i2c_hal_is_device_ready(i2c_handle_t* handle, uint16_t dev_address, uint32_t trials, uint32_t timeout)
{
    HAL_CHECK_HANDLE(handle);

    for (uint32_t i = 0; i < trials; i++) {
        HAL_LOCK(handle);

        i2c_start_on_bus(handle->config->instance);

        uint32_t start_time = xTaskGetTickCount();
        while (!i2c_flag_get(handle->config->instance, I2C_FLAG_SBSEND)) {
            if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout)) {
                HAL_UNLOCK(handle);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        i2c_master_addressing(handle->config->instance, dev_address, I2C_TRANSMITTER);

        start_time = xTaskGetTickCount();
        while (!i2c_flag_get(handle->config->instance, I2C_FLAG_ADDSEND) &&
               !i2c_flag_get(handle->config->instance, I2C_FLAG_AERR)) {
            if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        bool device_ready = i2c_flag_get(handle->config->instance, I2C_FLAG_ADDSEND);

        i2c_stop_on_bus(handle->config->instance);
        i2c_flag_clear(handle->config->instance, I2C_FLAG_ADDSEND | I2C_FLAG_AERR);

        HAL_UNLOCK(handle);

        if (device_ready) {
            return HAL_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); /* 等待10ms再重试 */
    }

    return HAL_ERROR;
}

/**
 * @brief I2C系统初始化
 * @return hal_result_t 操作结果
 */
hal_result_t i2c_hal_system_init(void)
{
    if (g_i2c_system_initialized) {
        return HAL_OK;
    }

    /* 使能I2C时钟 */
    rcu_periph_clock_enable(RCU_GPIOB);

    /* 配置I2C GPIO */
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_6 | GPIO_PIN_7);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6 | GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

    g_i2c_system_initialized = true;
    return HAL_OK;
}

/**
 * @brief I2C系统反初始化
 * @return hal_result_t 操作结果
 */
hal_result_t i2c_hal_system_deinit(void)
{
    g_i2c_system_initialized = false;
    return HAL_OK;
}