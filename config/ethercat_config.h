/**
 * @file    ethercat_config.h
 * @brief   EtherCAT协议栈配置文件 - 针对供墨系统优化
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __ETHERCAT_CONFIG_H__
#define __ETHERCAT_CONFIG_H__

#include "gd32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------- EtherCAT基础配置 ---------- */

/**
 * ESC_FREQUENCY: EtherCAT从站控制器频率
 * 设置为25MHz以匹配GD32F427的SPI时钟配置
 */
#define ESC_FREQUENCY               25000000

/**
 * ECAT_TIMER_INC_P_MS: EtherCAT定时器增量 (每毫秒)
 * 基于ESC频率计算：25MHz / 1000 = 25000
 */
#define ECAT_TIMER_INC_P_MS         25000

/**
 * ESC_MBX_DEFAULT_SIZE: 默认邮箱大小
 * 配置为128字节，适合工业控制应用
 */
#define ESC_MBX_DEFAULT_SIZE        128

/**
 * ESC_SM_WD: 同步管理器看门狗使能
 */
#define ESC_SM_WD                   1

/* ---------- 过程数据配置 ---------- */

/**
 * MAX_PD_INPUT_SIZE: 最大输入过程数据大小 (字节)
 * 传感器数据：7个传感器 * 4字节/传感器 = 28字节
 * 状态数据：8字节
 * 总计：36字节，向上对齐到64字节
 */
#define MAX_PD_INPUT_SIZE           64

/**
 * MAX_PD_OUTPUT_SIZE: 最大输出过程数据大小 (字节)
 * 执行器控制：13个执行器 * 4字节/执行器 = 52字节
 * 控制参数：12字节
 * 总计：64字节
 */
#define MAX_PD_OUTPUT_SIZE          64

/**
 * PD_CYCLE_TIME_US: 过程数据循环时间 (微秒)
 * 设置为1000us (1ms) 以匹配工业标准
 */
#define PD_CYCLE_TIME_US            1000

/* ---------- 同步管理器配置 ---------- */

/**
 * SM0 - 邮箱写入 (主站到从站)
 */
#define SM0_START_ADDR              0x1000
#define SM0_LENGTH                  ESC_MBX_DEFAULT_SIZE
#define SM0_CONTROL                 0x26

/**
 * SM1 - 邮箱读出 (从站到主站)
 */
#define SM1_START_ADDR              0x1080
#define SM1_LENGTH                  ESC_MBX_DEFAULT_SIZE
#define SM1_CONTROL                 0x22

/**
 * SM2 - 过程数据输出 (主站到从站)
 */
#define SM2_START_ADDR              0x1100
#define SM2_LENGTH                  MAX_PD_OUTPUT_SIZE
#define SM2_CONTROL                 0x64

/**
 * SM3 - 过程数据输入 (从站到主站)
 */
#define SM3_START_ADDR              0x1180
#define SM3_LENGTH                  MAX_PD_INPUT_SIZE
#define SM3_CONTROL                 0x20

/* ---------- 对象字典配置 ---------- */

/**
 * VENDOR_ID: 供应商ID
 * 使用测试用途的供应商ID
 */
#define VENDOR_ID                   0x12345678

/**
 * PRODUCT_CODE: 产品代码
 */
#define PRODUCT_CODE                0x00000001

/**
 * REVISION_NUMBER: 修订版本号
 */
#define REVISION_NUMBER             0x00040000

/**
 * SERIAL_NUMBER: 序列号
 */
#define SERIAL_NUMBER               0x00000001

/* ---------- 应用层配置 ---------- */

/**
 * AL_EVENT_ENABLED: 应用层事件掩码
 * 使能所有标准事件
 */
#define AL_EVENT_ENABLED            0xFF

/**
 * BOOTSTRAP_SUPPORTED: 引导程序支持
 */
#define BOOTSTRAP_SUPPORTED         0

/**
 * FOE_SUPPORTED: 文件访问协议支持
 */
#define FOE_SUPPORTED               0

/**
 * SOE_SUPPORTED: 服务访问协议支持
 */
#define SOE_SUPPORTED               0

/**
 * VOE_SUPPORTED: 供应商特定协议支持
 */
#define VOE_SUPPORTED               0

/**
 * EOE_SUPPORTED: 以太网协议支持
 */
#define EOE_SUPPORTED               0

/* ---------- 实时性能配置 ---------- */

/**
 * DC_SUPPORTED: 分布式时钟支持
 */
#define DC_SUPPORTED                1

/**
 * DC_RANGE: 分布式时钟范围 (纳秒)
 */
#define DC_RANGE                    125000

/**
 * MAX_SYNC_MAN: 最大同步管理器数量
 */
#define MAX_SYNC_MAN                8

/**
 * MAX_RXPDO_ENTRIES: 最大接收PDO条目数
 */
#define MAX_RXPDO_ENTRIES           16

/**
 * MAX_TXPDO_ENTRIES: 最大发送PDO条目数
 */
#define MAX_TXPDO_ENTRIES           16

/* ---------- 错误处理配置 ---------- */

/**
 * ERROR_ACK_SUPPORTED: 错误确认支持
 */
#define ERROR_ACK_SUPPORTED         1

/**
 * EMERGENCY_SUPPORTED: 紧急消息支持
 */
#define EMERGENCY_SUPPORTED         1

/**
 * SDO_TIMEOUT: SDO超时时间 (毫秒)
 */
#define SDO_TIMEOUT                 1000

/* ---------- 缓冲区配置 ---------- */

/**
 * MBX_BUFFER_SIZE: 邮箱缓冲区大小
 */
#define MBX_BUFFER_SIZE             (ESC_MBX_DEFAULT_SIZE * 2)

/**
 * PD_BUFFER_SIZE: 过程数据缓冲区大小
 */
#define PD_BUFFER_SIZE              (MAX_PD_INPUT_SIZE + MAX_PD_OUTPUT_SIZE)

/* ---------- 硬件接口配置 ---------- */

/**
 * ESC_SPI_INTERFACE: ESC SPI接口选择
 * 使用SPI1接口连接EtherCAT ESC芯片
 */
#define ESC_SPI_INTERFACE           SPI1

/**
 * ESC_IRQ_PIN: ESC中断引脚
 */
#define ESC_IRQ_PIN                 GPIO_PIN_10

/**
 * ESC_IRQ_PORT: ESC中断端口
 */
#define ESC_IRQ_PORT                GPIOA

/**
 * ESC_IRQ_EXTI_LINE: ESC EXTI线
 */
#define ESC_IRQ_EXTI_LINE           EXTI_10

/**
 * ESC_IRQ_SOURCE_PORT: ESC中断源端口
 */
#define ESC_IRQ_SOURCE_PORT         EXTI_SOURCE_GPIOA

/**
 * ESC_IRQ_SOURCE_PIN: ESC中断源引脚
 */
#define ESC_IRQ_SOURCE_PIN          EXTI_SOURCE_PIN10

/**
 * ESC_IRQ_PRIORITY: ESC中断优先级
 */
#define ESC_IRQ_PRIORITY            2

/* ---------- 调试配置 ---------- */

/**
 * ECAT_DEBUG_ENABLE: EtherCAT调试使能
 */
#ifdef DEBUG
#define ECAT_DEBUG_ENABLE           1
#else
#define ECAT_DEBUG_ENABLE           0
#endif

/**
 * ECAT_TRACE_ENABLE: EtherCAT跟踪使能
 */
#define ECAT_TRACE_ENABLE           0

/* ---------- 应用特定配置 ---------- */

/**
 * 供墨系统特定的EtherCAT配置
 */

// 输入数据映射 (从站发送给主站)
typedef struct {
    // 传感器数据
    float liquid_level_1;        // 液位传感器1 (%)
    float liquid_level_2;        // 液位传感器2 (%)
    float pressure_1;            // 压力传感器1 (kPa)
    float pressure_2;            // 压力传感器2 (kPa)
    float temperature_1;         // 温度传感器1 (°C)
    float temperature_2;         // 温度传感器2 (°C)
    float temperature_3;         // 温度传感器3 (°C)

    // 状态数据
    uint16_t system_status;      // 系统状态
    uint16_t fault_code;         // 故障代码
    uint16_t safety_status;      // 安全状态
    uint16_t reserved;           // 保留
} ethercat_input_data_t;

// 输出数据映射 (主站发送给从站)
typedef struct {
    // 执行器控制
    float heater_1_power;        // 加热器1功率 (%)
    float heater_2_power;        // 加热器2功率 (%)
    float heater_3_power;        // 加热器3功率 (%)
    uint16_t pump_1_speed;       // 泵1转速 (RPM)
    uint16_t pump_2_speed;       // 泵2转速 (RPM)
    uint16_t valve_states;       // 阀门状态 (位掩码)

    // 控制参数
    float temperature_setpoint_1; // 温度设定点1
    float temperature_setpoint_2; // 温度设定点2
    float temperature_setpoint_3; // 温度设定点3
    float pressure_setpoint_1;    // 压力设定点1
    float pressure_setpoint_2;    // 压力设定点2
    uint16_t control_mode;        // 控制模式
    uint16_t system_command;      // 系统命令
    uint16_t reserved;            // 保留
} ethercat_output_data_t;

/* ---------- 函数声明 ---------- */

/**
 * EtherCAT配置初始化
 */
void ethercat_config_init(void);

/**
 * 获取输入数据指针
 */
ethercat_input_data_t* ethercat_get_input_data(void);

/**
 * 获取输出数据指针
 */
ethercat_output_data_t* ethercat_get_output_data(void);

#endif /* __ETHERCAT_CONFIG_H__ */