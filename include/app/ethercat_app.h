/**
 * @file    ethercat_app.h
 * @brief   EtherCAT应用头文件 - 保持v1版本EtherCAT功能
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __ETHERCAT_APP_H__
#define __ETHERCAT_APP_H__

#include <stdint.h>
#include <stdbool.h>

// EtherCAT结果类型
typedef enum {
    ETHERCAT_SUCCESS = 0,
    ETHERCAT_ERROR_INVALID_PARAMETER,
    ETHERCAT_ERROR_TIMEOUT,
    ETHERCAT_ERROR_HARDWARE_INIT,
    ETHERCAT_ERROR_MEMORY_ALLOCATION,
    ETHERCAT_ERROR_COMMUNICATION,
    ETHERCAT_ERROR_SYNC_TIMEOUT
} ethercat_result_t;

// EtherCAT控制命令定义
typedef enum {
    ETHERCAT_CMD_NONE = 0x0000,
    ETHERCAT_CMD_SYSTEM_RESET = 0x0001,
    ETHERCAT_CMD_EMERGENCY_STOP = 0x0002,
    ETHERCAT_CMD_CLEAR_FAULTS = 0x0003,
    ETHERCAT_CMD_CALIBRATE_SENSORS = 0x0004,
    ETHERCAT_CMD_SAVE_CONFIG = 0x0005,
    ETHERCAT_CMD_LOAD_CONFIG = 0x0006,
    ETHERCAT_CMD_START_OPERATION = 0x0007,
    ETHERCAT_CMD_STOP_OPERATION = 0x0008
} ethercat_command_t;

// EtherCAT状态信息
typedef struct {
    bool initialized;              // 初始化状态
    bool link_up;                 // 链路状态
    uint8_t al_state;            // 应用层状态
    uint32_t cycle_count;        // 循环计数
    uint32_t error_count;        // 错误计数
    uint32_t max_cycle_time_us;  // 最大循环时间(微秒)
} ethercat_status_info_t;

// EtherCAT配置参数
typedef struct {
    uint16_t vendor_id;          // 厂商ID
    uint32_t product_code;       // 产品代码
    uint32_t revision_number;    // 版本号
    uint32_t serial_number;      // 序列号
    uint16_t sync0_cycle_time;   // 同步周期(ms)
    bool enable_dc;              // 使能分布式时钟
} ethercat_config_t;

/**
 * @brief  EtherCAT应用初始化
 * @param  None
 * @retval ethercat_result_t 初始化结果
 */
ethercat_result_t ethercat_app_init(void);

/**
 * @brief  EtherCAT应用任务
 * @param  pvParameters 任务参数
 * @retval None
 */
void ethercat_app_task(void *pvParameters);

/**
 * @brief  获取EtherCAT状态
 * @param  status 状态结构指针
 * @retval ethercat_result_t 获取结果
 */
ethercat_result_t ethercat_get_status(ethercat_status_info_t *status);

/**
 * @brief  设置EtherCAT配置
 * @param  config 配置结构指针
 * @retval ethercat_result_t 设置结果
 */
ethercat_result_t ethercat_set_config(const ethercat_config_t *config);

// EtherCAT协议栈回调函数 (由SSC协议栈调用)
void APPL_Application(void);
void APPL_InputMapping(uint16_t *pData, uint16_t Size);
void APPL_OutputMapping(uint16_t *pData, uint16_t Size);

#endif /* __ETHERCAT_APP_H__ */