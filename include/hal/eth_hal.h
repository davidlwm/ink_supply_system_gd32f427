/**
 * @file eth_hal.h
 * @brief Ethernet Hardware Abstraction Layer
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 以太网硬件抽象层，基于GD32F4xx HAL库
 *              提供TCP/UDP通信、网络配置管理功能
 */

#ifndef ETH_HAL_H
#define ETH_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"
#include "gd32f4xx_enet.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 宏定义 */
#define ETH_MAX_PACKET_SIZE     1518
#define ETH_MIN_PACKET_SIZE     64
#define ETH_MAC_ADDR_LEN        6
#define ETH_IP_ADDR_LEN         4
#define ETH_RX_BUFFER_COUNT     4
#define ETH_TX_BUFFER_COUNT     2

/* 错误码定义 */
typedef enum {
    ETH_OK = 0,
    ETH_ERROR = 1,
    ETH_BUSY = 2,
    ETH_TIMEOUT = 3,
    ETH_INVALID_PARAM = 4,
    ETH_NO_LINK = 5,
    ETH_BUFFER_FULL = 6
} eth_result_t;

/* 以太网链路状态 */
typedef enum {
    ETH_LINK_DOWN = 0,
    ETH_LINK_UP = 1
} eth_link_status_t;

/* 以太网速度 */
typedef enum {
    ETH_SPEED_10M = 0,
    ETH_SPEED_100M = 1
} eth_speed_t;

/* 以太网双工模式 */
typedef enum {
    ETH_DUPLEX_HALF = 0,
    ETH_DUPLEX_FULL = 1
} eth_duplex_t;

/* 网络配置结构体 */
typedef struct {
    uint8_t mac_addr[ETH_MAC_ADDR_LEN];     /* MAC地址 */
    uint8_t ip_addr[ETH_IP_ADDR_LEN];       /* IP地址 */
    uint8_t netmask[ETH_IP_ADDR_LEN];       /* 子网掩码 */
    uint8_t gateway[ETH_IP_ADDR_LEN];       /* 网关地址 */
    bool dhcp_enabled;                       /* DHCP使能 */
} eth_network_config_t;

/* 以太网状态结构体 */
typedef struct {
    eth_link_status_t link_status;          /* 链路状态 */
    eth_speed_t speed;                      /* 链路速度 */
    eth_duplex_t duplex;                    /* 双工模式 */
    uint32_t rx_packets;                    /* 接收包计数 */
    uint32_t tx_packets;                    /* 发送包计数 */
    uint32_t rx_errors;                     /* 接收错误计数 */
    uint32_t tx_errors;                     /* 发送错误计数 */
} eth_status_t;

/* 数据包结构体 */
typedef struct {
    uint8_t* data;                          /* 数据指针 */
    uint16_t length;                        /* 数据长度 */
    uint32_t timestamp;                     /* 时间戳 */
} eth_packet_t;

/* 以太网回调函数类型 */
typedef void (*eth_rx_callback_t)(eth_packet_t* packet);
typedef void (*eth_link_callback_t)(eth_link_status_t status);

/* 初始化和配置 */
eth_result_t eth_hal_init(const eth_network_config_t* config);
eth_result_t eth_hal_deinit(void);
eth_result_t eth_hal_start(void);
eth_result_t eth_hal_stop(void);
eth_result_t eth_hal_reset(void);

/* 网络配置 */
eth_result_t eth_hal_set_mac_addr(const uint8_t* mac_addr);
eth_result_t eth_hal_get_mac_addr(uint8_t* mac_addr);
eth_result_t eth_hal_set_ip_config(const uint8_t* ip_addr, const uint8_t* netmask, const uint8_t* gateway);
eth_result_t eth_hal_get_ip_config(uint8_t* ip_addr, uint8_t* netmask, uint8_t* gateway);
eth_result_t eth_hal_enable_dhcp(bool enable);

/* 数据传输 */
eth_result_t eth_hal_send_packet(const uint8_t* data, uint16_t length, uint32_t timeout);
eth_result_t eth_hal_receive_packet(uint8_t* data, uint16_t* length, uint32_t timeout);
bool eth_hal_is_packet_available(void);
uint16_t eth_hal_get_packet_length(void);

/* 状态监控 */
eth_result_t eth_hal_get_status(eth_status_t* status);
eth_link_status_t eth_hal_get_link_status(void);
eth_speed_t eth_hal_get_link_speed(void);
eth_duplex_t eth_hal_get_duplex_mode(void);

/* 统计信息 */
uint32_t eth_hal_get_rx_count(void);
uint32_t eth_hal_get_tx_count(void);
uint32_t eth_hal_get_rx_error_count(void);
uint32_t eth_hal_get_tx_error_count(void);
eth_result_t eth_hal_clear_statistics(void);

/* 回调函数 */
eth_result_t eth_hal_register_rx_callback(eth_rx_callback_t callback);
eth_result_t eth_hal_register_link_callback(eth_link_callback_t callback);
eth_result_t eth_hal_unregister_rx_callback(void);
eth_result_t eth_hal_unregister_link_callback(void);

/* 电源管理 */
eth_result_t eth_hal_enter_sleep_mode(void);
eth_result_t eth_hal_exit_sleep_mode(void);
bool eth_hal_is_sleep_mode(void);

/* 诊断功能 */
eth_result_t eth_hal_loopback_test(void);
eth_result_t eth_hal_cable_test(void);
eth_result_t eth_hal_phy_register_read(uint16_t reg_addr, uint16_t* data);
eth_result_t eth_hal_phy_register_write(uint16_t reg_addr, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif /* ETH_HAL_H */