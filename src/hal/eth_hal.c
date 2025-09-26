/**
 * @file eth_hal.c
 * @brief Ethernet Hardware Abstraction Layer Implementation
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 以太网硬件抽象层实现，基于GD32F4xx HAL库
 */

#include "hal/eth_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

/* 私有宏定义 */
#define ETH_PHY_ADDRESS         0x01
#define ETH_TIMEOUT_MS          5000
#define ETH_LINK_CHECK_MS       1000
#define ETH_BUFFER_ALIGNMENT    4

/* 私有变量 */
static eth_network_config_t g_eth_config;
static eth_status_t g_eth_status;
static eth_rx_callback_t g_rx_callback = NULL;
static eth_link_callback_t g_link_callback = NULL;
static SemaphoreHandle_t g_eth_mutex = NULL;
static TaskHandle_t g_link_monitor_task = NULL;
static bool g_eth_initialized = false;
static bool g_eth_started = false;

/* 接收缓冲区 */
static uint8_t g_rx_buffers[ETH_RX_BUFFER_COUNT][ETH_MAX_PACKET_SIZE] __attribute__((aligned(ETH_BUFFER_ALIGNMENT)));
static uint8_t g_tx_buffers[ETH_TX_BUFFER_COUNT][ETH_MAX_PACKET_SIZE] __attribute__((aligned(ETH_BUFFER_ALIGNMENT)));
static volatile uint8_t g_rx_buffer_index = 0;
static volatile uint8_t g_tx_buffer_index = 0;

/* 私有函数声明 */
static eth_result_t eth_hal_phy_init(void);
static eth_result_t eth_hal_mac_init(void);
static void eth_hal_link_monitor_task(void* parameter);
static eth_result_t eth_hal_update_link_status(void);
static void eth_hal_rx_interrupt_handler(void);

/**
 * @brief 以太网HAL初始化
 * @param config 网络配置参数
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_init(const eth_network_config_t* config)
{
    if (config == NULL) {
        return ETH_INVALID_PARAM;
    }

    if (g_eth_initialized) {
        return ETH_OK;
    }

    /* 创建互斥锁 */
    g_eth_mutex = xSemaphoreCreateMutex();
    if (g_eth_mutex == NULL) {
        return ETH_ERROR;
    }

    /* 复制配置 */
    memcpy(&g_eth_config, config, sizeof(eth_network_config_t));

    /* 初始化状态 */
    memset(&g_eth_status, 0, sizeof(eth_status_t));
    g_eth_status.link_status = ETH_LINK_DOWN;

    /* 使能以太网时钟 */
    rcu_periph_clock_enable(RCU_ENET);
    rcu_periph_clock_enable(RCU_ENETTX);
    rcu_periph_clock_enable(RCU_ENETRX);

    /* 配置以太网引脚 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* 初始化PHY */
    if (eth_hal_phy_init() != ETH_OK) {
        vSemaphoreDelete(g_eth_mutex);
        return ETH_ERROR;
    }

    /* 初始化MAC */
    if (eth_hal_mac_init() != ETH_OK) {
        vSemaphoreDelete(g_eth_mutex);
        return ETH_ERROR;
    }

    /* 创建链路监控任务 */
    if (xTaskCreate(eth_hal_link_monitor_task, "EthLink", 256, NULL, 3, &g_link_monitor_task) != pdPASS) {
        vSemaphoreDelete(g_eth_mutex);
        return ETH_ERROR;
    }

    g_eth_initialized = true;
    return ETH_OK;
}

/**
 * @brief 以太网HAL反初始化
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_deinit(void)
{
    if (!g_eth_initialized) {
        return ETH_OK;
    }

    /* 停止以太网 */
    eth_hal_stop();

    /* 删除链路监控任务 */
    if (g_link_monitor_task != NULL) {
        vTaskDelete(g_link_monitor_task);
        g_link_monitor_task = NULL;
    }

    /* 禁用以太网 */
    enet_deinit();

    /* 禁用时钟 */
    rcu_periph_clock_disable(RCU_ENET);
    rcu_periph_clock_disable(RCU_ENETTX);
    rcu_periph_clock_disable(RCU_ENETRX);

    /* 删除互斥锁 */
    if (g_eth_mutex != NULL) {
        vSemaphoreDelete(g_eth_mutex);
        g_eth_mutex = NULL;
    }

    g_eth_initialized = false;
    return ETH_OK;
}

/**
 * @brief 启动以太网
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_start(void)
{
    if (!g_eth_initialized) {
        return ETH_ERROR;
    }

    if (xSemaphoreTake(g_eth_mutex, pdMS_TO_TICKS(ETH_TIMEOUT_MS)) != pdTRUE) {
        return ETH_TIMEOUT;
    }

    if (!g_eth_started) {
        /* 使能以太网接收和发送 */
        enet_enable();

        /* 启动DMA */
        enet_dma_function_enable(ENET_DMA_RX);
        enet_dma_function_enable(ENET_DMA_TX);

        g_eth_started = true;
    }

    xSemaphoreGive(g_eth_mutex);
    return ETH_OK;
}

/**
 * @brief 停止以太网
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_stop(void)
{
    if (!g_eth_initialized) {
        return ETH_ERROR;
    }

    if (xSemaphoreTake(g_eth_mutex, pdMS_TO_TICKS(ETH_TIMEOUT_MS)) != pdTRUE) {
        return ETH_TIMEOUT;
    }

    if (g_eth_started) {
        /* 禁用DMA */
        enet_dma_function_disable(ENET_DMA_RX);
        enet_dma_function_disable(ENET_DMA_TX);

        /* 禁用以太网 */
        enet_disable();

        g_eth_started = false;
    }

    xSemaphoreGive(g_eth_mutex);
    return ETH_OK;
}

/**
 * @brief 设置MAC地址
 * @param mac_addr MAC地址
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_set_mac_addr(const uint8_t* mac_addr)
{
    if (mac_addr == NULL) {
        return ETH_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_eth_mutex, pdMS_TO_TICKS(ETH_TIMEOUT_MS)) != pdTRUE) {
        return ETH_TIMEOUT;
    }

    memcpy(g_eth_config.mac_addr, mac_addr, ETH_MAC_ADDR_LEN);

    /* 设置硬件MAC地址 */
    enet_mac_address_set(ENET_MAC_ADDRESS0, (uint8_t*)mac_addr);

    xSemaphoreGive(g_eth_mutex);
    return ETH_OK;
}

/**
 * @brief 获取MAC地址
 * @param mac_addr MAC地址缓冲区
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_get_mac_addr(uint8_t* mac_addr)
{
    if (mac_addr == NULL) {
        return ETH_INVALID_PARAM;
    }

    memcpy(mac_addr, g_eth_config.mac_addr, ETH_MAC_ADDR_LEN);
    return ETH_OK;
}

/**
 * @brief 发送数据包
 * @param data 数据指针
 * @param length 数据长度
 * @param timeout 超时时间(ms)
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_send_packet(const uint8_t* data, uint16_t length, uint32_t timeout)
{
    if (data == NULL || length == 0 || length > ETH_MAX_PACKET_SIZE) {
        return ETH_INVALID_PARAM;
    }

    if (!g_eth_started || g_eth_status.link_status != ETH_LINK_UP) {
        return ETH_NO_LINK;
    }

    if (xSemaphoreTake(g_eth_mutex, pdMS_TO_TICKS(timeout)) != pdTRUE) {
        return ETH_TIMEOUT;
    }

    /* 复制数据到发送缓冲区 */
    uint8_t* tx_buffer = g_tx_buffers[g_tx_buffer_index];
    memcpy(tx_buffer, data, length);

    /* 发送数据包 */
    ErrStatus result = enet_frame_send(tx_buffer, length);

    if (result == SUCCESS) {
        g_eth_status.tx_packets++;
        g_tx_buffer_index = (g_tx_buffer_index + 1) % ETH_TX_BUFFER_COUNT;
        xSemaphoreGive(g_eth_mutex);
        return ETH_OK;
    } else {
        g_eth_status.tx_errors++;
        xSemaphoreGive(g_eth_mutex);
        return ETH_ERROR;
    }
}

/**
 * @brief 接收数据包
 * @param data 数据缓冲区
 * @param length 数据长度指针
 * @param timeout 超时时间(ms)
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_receive_packet(uint8_t* data, uint16_t* length, uint32_t timeout)
{
    if (data == NULL || length == NULL) {
        return ETH_INVALID_PARAM;
    }

    if (!g_eth_started) {
        return ETH_ERROR;
    }

    uint32_t start_time = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout)) {
        if (enet_frame_receive() == SUCCESS) {
            uint32_t frame_length = enet_frame_length_get();

            if (frame_length <= *length) {
                /* 复制接收到的数据 */
                uint8_t* rx_buffer = g_rx_buffers[g_rx_buffer_index];
                memcpy(data, rx_buffer, frame_length);
                *length = frame_length;

                g_eth_status.rx_packets++;
                g_rx_buffer_index = (g_rx_buffer_index + 1) % ETH_RX_BUFFER_COUNT;
                return ETH_OK;
            } else {
                g_eth_status.rx_errors++;
                return ETH_BUFFER_FULL;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ETH_TIMEOUT;
}

/**
 * @brief 获取链路状态
 * @return eth_link_status_t 链路状态
 */
eth_link_status_t eth_hal_get_link_status(void)
{
    return g_eth_status.link_status;
}

/**
 * @brief 获取统计信息
 * @param status 状态结构体指针
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_get_status(eth_status_t* status)
{
    if (status == NULL) {
        return ETH_INVALID_PARAM;
    }

    memcpy(status, &g_eth_status, sizeof(eth_status_t));
    return ETH_OK;
}

/**
 * @brief 注册接收回调函数
 * @param callback 回调函数指针
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_register_rx_callback(eth_rx_callback_t callback)
{
    g_rx_callback = callback;
    return ETH_OK;
}

/**
 * @brief 注册链路状态回调函数
 * @param callback 回调函数指针
 * @return eth_result_t 操作结果
 */
eth_result_t eth_hal_register_link_callback(eth_link_callback_t callback)
{
    g_link_callback = callback;
    return ETH_OK;
}

/* 私有函数实现 */

/**
 * @brief PHY初始化
 * @return eth_result_t 操作结果
 */
static eth_result_t eth_hal_phy_init(void)
{
    /* 配置PHY引脚 */
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_13);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);

    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_200MHZ, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_200MHZ, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_200MHZ, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);

    /* 复位PHY */
    enet_phy_write_read(ENET_PHY_WRITE, ETH_PHY_ADDRESS, PHY_REG_BCR, PHY_RESET);

    /* 等待复位完成 */
    uint32_t timeout = ETH_TIMEOUT_MS;
    uint16_t phy_value;
    do {
        enet_phy_write_read(ENET_PHY_READ, ETH_PHY_ADDRESS, PHY_REG_BCR, &phy_value);
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout--;
    } while ((phy_value & PHY_RESET) && timeout > 0);

    if (timeout == 0) {
        return ETH_TIMEOUT;
    }

    return ETH_OK;
}

/**
 * @brief MAC初始化
 * @return eth_result_t 操作结果
 */
static eth_result_t eth_hal_mac_init(void)
{
    /* 配置以太网参数 */
    enet_initpara_config(ENET_AUTO_NEGOTIATION, ENET_NO_AUTOCHECKSUM, ENET_NO_WATCHDOG);

    /* 初始化以太网 */
    if (enet_init() != SUCCESS) {
        return ETH_ERROR;
    }

    /* 设置MAC地址 */
    enet_mac_address_set(ENET_MAC_ADDRESS0, g_eth_config.mac_addr);

    /* 配置DMA描述符 */
    enet_descriptors_chain_init(ENET_DMA_TX);
    enet_descriptors_chain_init(ENET_DMA_RX);

    /* 配置接收缓冲区 */
    for (int i = 0; i < ETH_RX_BUFFER_COUNT; i++) {
        enet_desc_receive_active_set(g_rx_buffers[i], ETH_MAX_PACKET_SIZE);
    }

    return ETH_OK;
}

/**
 * @brief 链路监控任务
 * @param parameter 任务参数
 */
static void eth_hal_link_monitor_task(void* parameter)
{
    (void)parameter;

    while (1) {
        eth_hal_update_link_status();
        vTaskDelay(pdMS_TO_TICKS(ETH_LINK_CHECK_MS));
    }
}

/**
 * @brief 更新链路状态
 * @return eth_result_t 操作结果
 */
static eth_result_t eth_hal_update_link_status(void)
{
    uint16_t phy_value;
    eth_link_status_t old_status = g_eth_status.link_status;

    /* 读取PHY状态寄存器 */
    enet_phy_write_read(ENET_PHY_READ, ETH_PHY_ADDRESS, PHY_REG_BSR, &phy_value);

    if (phy_value & PHY_LINKED_STATUS) {
        g_eth_status.link_status = ETH_LINK_UP;

        /* 读取速度和双工模式 */
        enet_phy_write_read(ENET_PHY_READ, ETH_PHY_ADDRESS, PHY_REG_SR, &phy_value);

        if (phy_value & PHY_SPEED_STATUS) {
            g_eth_status.speed = ETH_SPEED_10M;
        } else {
            g_eth_status.speed = ETH_SPEED_100M;
        }

        if (phy_value & PHY_DUPLEX_STATUS) {
            g_eth_status.duplex = ETH_DUPLEX_FULL;
        } else {
            g_eth_status.duplex = ETH_DUPLEX_HALF;
        }
    } else {
        g_eth_status.link_status = ETH_LINK_DOWN;
    }

    /* 如果链路状态发生变化，调用回调函数 */
    if (old_status != g_eth_status.link_status && g_link_callback != NULL) {
        g_link_callback(g_eth_status.link_status);
    }

    return ETH_OK;
}