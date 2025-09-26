/**
 * @file    config_task.h
 * @brief   配置管理任务头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __CONFIG_TASK_H
#define __CONFIG_TASK_H

#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

// 配置结果定义
typedef enum {
    CONFIG_SUCCESS = 0,
    CONFIG_ERROR_INVALID_PARAMETER,
    CONFIG_ERROR_FLASH_WRITE,
    CONFIG_ERROR_FLASH_READ,
    CONFIG_ERROR_CHECKSUM,
    CONFIG_ERROR_VERSION_MISMATCH,
    CONFIG_ERROR_MEMORY_ALLOCATION
} config_result_t;

// 配置数据类型定义
typedef enum {
    CONFIG_TYPE_SYSTEM = 0,
    CONFIG_TYPE_SENSOR,
    CONFIG_TYPE_ACTUATOR,
    CONFIG_TYPE_CONTROL,
    CONFIG_TYPE_COMMUNICATION,
    CONFIG_TYPE_SAFETY,
    CONFIG_TYPE_HMI,
    CONFIG_TYPE_CALIBRATION
} config_type_t;

// 系统配置结构
typedef struct {
    uint32_t system_id;
    char device_name[32];
    uint8_t hardware_version;
    uint8_t software_version;
    uint32_t serial_number;
    bool auto_start;
    uint8_t default_mode;
    uint32_t watchdog_timeout;
} system_config_t;

// 传感器配置结构
typedef struct {
    float calibration_offset[7];   // 传感器校准偏移
    float calibration_scale[7];    // 传感器校准比例
    uint32_t sample_period[7];     // 采样周期
    bool enable_filter[7];         // 滤波使能
    float filter_coefficient[7];   // 滤波系数
} sensor_config_t;

// 执行器配置结构
typedef struct {
    float power_limit[3];          // 加热器功率限制
    uint16_t speed_limit[2];       // 泵速度限制
    uint32_t valve_delay[8];       // 阀门动作延时
    bool safety_enable[13];        // 安全保护使能
} actuator_config_t;

// 控制配置结构
typedef struct {
    float temperature_pid_kp[3];
    float temperature_pid_ki[3];
    float temperature_pid_kd[3];
    float pressure_pid_kp[2];
    float pressure_pid_ki[2];
    float pressure_pid_kd[2];
    float temperature_target[3];
    float pressure_target[2];
    float level_target[2];
    uint32_t control_period;
} control_config_t;

// 通信配置结构
typedef struct {
    uint32_t ethernet_ip;
    uint32_t ethernet_mask;
    uint32_t ethernet_gateway;
    uint16_t tcp_port;
    uint16_t ethercat_station_id;
    uint32_t comm_timeout;
    bool enable_tcp;
    bool enable_ethercat;
} communication_config_t;

// 配置数据头结构
typedef struct {
    uint32_t magic_number;         // 魔数标识
    uint16_t version;              // 配置版本
    uint16_t data_size;            // 数据大小
    uint32_t checksum;             // 校验和
    uint32_t timestamp;            // 时间戳
} config_header_t;

// 完整配置结构
typedef struct {
    config_header_t header;
    system_config_t system;
    sensor_config_t sensor;
    actuator_config_t actuator;
    control_config_t control;
    communication_config_t communication;
} device_config_t;

// 任务函数
void config_task(void *pvParameters);
config_result_t config_manager_init(void);

// 配置管理函数
config_result_t config_load_default(void);
config_result_t config_load_from_flash(void);
config_result_t config_save_to_flash(void);
config_result_t config_backup_create(void);
config_result_t config_backup_restore(void);

// 配置读写函数
config_result_t config_read(config_type_t type, void *data, uint16_t size);
config_result_t config_write(config_type_t type, const void *data, uint16_t size);
config_result_t config_get_system(system_config_t *config);
config_result_t config_set_system(const system_config_t *config);
config_result_t config_get_sensor(sensor_config_t *config);
config_result_t config_set_sensor(const sensor_config_t *config);
config_result_t config_get_actuator(actuator_config_t *config);
config_result_t config_set_actuator(const actuator_config_t *config);
config_result_t config_get_control(control_config_t *config);
config_result_t config_set_control(const control_config_t *config);

// 配置验证函数
config_result_t config_validate(config_type_t type, const void *data);
config_result_t config_check_integrity(void);
uint32_t config_calculate_checksum(const void *data, uint16_t size);

// 工厂重置函数
config_result_t config_factory_reset(void);
config_result_t config_export_to_file(const char *filename);
config_result_t config_import_from_file(const char *filename);

// 内部处理函数
static void config_task_init(void);
static void config_periodic_save(void);
static config_result_t config_flash_write_block(uint32_t address, const void *data, uint16_t size);
static config_result_t config_flash_read_block(uint32_t address, void *data, uint16_t size);
static void config_apply_loaded_settings(void);

// Flash存储地址定义
#define CONFIG_FLASH_BASE_ADDR      0x08060000  // 配置存储基地址
#define CONFIG_BACKUP_ADDR          0x08070000  // 备份存储地址
#define CONFIG_MAGIC_NUMBER         0x12345678  // 配置魔数
#define CONFIG_VERSION              0x0100      // 配置版本1.0

#endif /* __CONFIG_TASK_H */