/**
 * @file    comm_task.h
 * @brief   通信任务头文件 - 管理TCP/IP和EtherCAT双协议
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#include <stdint.h>
#include <stdbool.h>

// 通信结果类型
typedef enum {
    COMM_SUCCESS = 0,
    COMM_ERROR_INVALID_PARAMETER,
    COMM_ERROR_TIMEOUT,
    COMM_ERROR_MEMORY_ALLOCATION,
    COMM_ERROR_TASK_CREATE,
    COMM_ERROR_NETWORK_INIT,
    COMM_ERROR_TCP_INIT,
    COMM_ERROR_ETHERCAT_INIT
} comm_result_t;

// 通信系统状态信息
typedef struct {
    bool tcp_server_running;       // TCP服务器运行状态
    bool ethercat_running;         // EtherCAT运行状态
    bool network_initialized;     // 网络初始化状态
    uint32_t tcp_client_count;    // TCP客户端数量
    uint32_t ethercat_cycle_count; // EtherCAT循环计数
    uint32_t total_tx_bytes;      // 总发送字节数
    uint32_t total_rx_bytes;      // 总接收字节数
} comm_system_status_info_t;

// 通信配置参数
typedef struct {
    // TCP/IP配置
    struct {
        uint32_t ip_address;       // IP地址
        uint32_t netmask;         // 子网掩码
        uint32_t gateway;         // 网关
        uint16_t tcp_port;        // TCP端口
        uint8_t max_clients;      // 最大客户端数
    } tcp_config;

    // EtherCAT配置
    struct {
        uint16_t vendor_id;       // 厂商ID
        uint32_t product_code;    // 产品代码
        uint16_t cycle_time_ms;   // 循环时间(ms)
        bool enable_dc;           // 使能分布式时钟
    } ethercat_config;
} comm_config_t;

// 任务配置参数
#define COMM_TASK_PERIOD_MS         50      // 通信任务周期50ms
#define TCP_SERVER_STACK_SIZE       2048    // TCP服务器栈大小
#define TCP_SERVER_TASK_PRIORITY    3       // TCP服务器任务优先级
#define ETHERCAT_STACK_SIZE         2048    // EtherCAT任务栈大小
#define ETHERCAT_PRIORITY           8       // EtherCAT任务优先级

/**
 * @brief  通信系统初始化
 * @param  None
 * @retval comm_result_t 初始化结果
 */
comm_result_t comm_manager_init(void);

/**
 * @brief  通信主任务
 * @param  pvParameters 任务参数
 * @retval None
 */
void comm_task(void *pvParameters);

/**
 * @brief  获取通信系统状态
 * @param  status 状态结构指针
 * @retval comm_result_t 获取结果
 */
comm_result_t comm_get_system_status(comm_system_status_info_t *status);

/**
 * @brief  设置通信配置
 * @param  config 配置结构指针
 * @retval comm_result_t 设置结果
 */
comm_result_t comm_set_config(const comm_config_t *config);

#endif /* __COMM_TASK_H__ */