/**
 * @file    config_task.c
 * @brief   配置管理任务实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "app/config_task.h"
#include "app/system_config.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// 标准库
#include <string.h>
#include <stdio.h>

// 全局配置数据
static device_config_t g_device_config = {0};
static device_config_t g_backup_config = {0};
static SemaphoreHandle_t g_config_mutex = NULL;
static bool g_config_loaded = false;
static bool g_config_modified = false;

// 自动保存计时器
static uint32_t g_last_save_time = 0;
static uint32_t g_auto_save_interval = 300000; // 5分钟自动保存

// 默认配置
static const device_config_t default_device_config = {
    .header = {
        .magic_number = CONFIG_MAGIC_NUMBER,
        .version = CONFIG_VERSION,
        .data_size = sizeof(device_config_t) - sizeof(config_header_t),
        .checksum = 0,
        .timestamp = 0
    },
    .system = {
        .system_id = 0x12345678,
        .device_name = "InkSupplySystem",
        .hardware_version = 1,
        .software_version = 4,
        .serial_number = 1000001,
        .auto_start = true,
        .default_mode = 1, // AUTOMATIC
        .watchdog_timeout = 5000
    },
    .sensor = {
        .calibration_offset = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .calibration_scale = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
        .sample_period = {50, 50, 100, 100, 100, 100, 100},
        .enable_filter = {true, true, true, true, true, true, true},
        .filter_coefficient = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}
    },
    .actuator = {
        .power_limit = {100.0f, 100.0f, 80.0f},
        .speed_limit = {4500, 4000},
        .valve_delay = {100, 100, 100, 100, 100, 100, 100, 100},
        .safety_enable = {true, true, true, true, true, true, true, true, true, true, true, true, true}
    },
    .control = {
        .temperature_pid_kp = {2.0f, 2.0f, 1.8f},
        .temperature_pid_ki = {0.1f, 0.1f, 0.08f},
        .temperature_pid_kd = {0.05f, 0.05f, 0.04f},
        .pressure_pid_kp = {1.5f, 1.5f},
        .pressure_pid_ki = {0.05f, 0.05f},
        .pressure_pid_kd = {0.02f, 0.02f},
        .temperature_target = {60.0f, 65.0f, 70.0f},
        .pressure_target = {50.0f, 45.0f},
        .level_target = {80.0f, 75.0f},
        .control_period = 10
    },
    .communication = {
        .ethernet_ip = 0xC0A80164,      // 192.168.1.100
        .ethernet_mask = 0xFFFFFF00,    // 255.255.255.0
        .ethernet_gateway = 0xC0A80101, // 192.168.1.1
        .tcp_port = 8080,
        .ethercat_station_id = 1,
        .comm_timeout = 5000,
        .enable_tcp = true,
        .enable_ethercat = true
    }
};

/**
 * @brief  配置管理器初始化
 * @param  None
 * @retval config_result_t 初始化结果
 */
config_result_t config_manager_init(void)
{
    // 创建互斥锁
    g_config_mutex = xSemaphoreCreateMutex();
    if (g_config_mutex == NULL) {
        return CONFIG_ERROR_MEMORY_ALLOCATION;
    }

    // 初始化配置状态
    g_config_loaded = false;
    g_config_modified = false;
    g_last_save_time = xTaskGetTickCount();

    // 尝试从Flash加载配置
    config_result_t result = config_load_from_flash();
    if (result != CONFIG_SUCCESS) {
        // 加载失败，使用默认配置
        result = config_load_default();
        if (result == CONFIG_SUCCESS) {
            // 保存默认配置到Flash
            config_save_to_flash();
        }
    }

    return result;
}

/**
 * @brief  配置任务主函数
 * @param  pvParameters 任务参数
 * @retval None
 */
void config_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    uint32_t task_counter = 0;

    // 等待系统初始化完成
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 初始化配置管理器
    if (config_manager_init() != CONFIG_SUCCESS) {
        vTaskDelete(NULL);
        return;
    }

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 严格1秒周期执行
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONFIG_TASK_PERIOD_MS));

        task_counter++;

        // 1. 检查配置完整性 (每分钟)
        if (task_counter % 60 == 0) {
            config_check_integrity();
        }

        // 2. 自动保存检查 (每10秒)
        if (task_counter % 10 == 0) {
            config_periodic_save();
        }

        // 3. 创建备份 (每小时)
        if (task_counter % 3600 == 0) {
            config_backup_create();
        }
    }
}

/**
 * @brief  加载默认配置
 * @param  None
 * @retval config_result_t 加载结果
 */
config_result_t config_load_default(void)
{
    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CONFIG_ERROR_MEMORY_ALLOCATION;
    }

    // 复制默认配置
    memcpy(&g_device_config, &default_device_config, sizeof(device_config_t));

    // 更新时间戳
    g_device_config.header.timestamp = xTaskGetTickCount();

    // 计算校验和
    g_device_config.header.checksum = config_calculate_checksum(
        (uint8_t*)&g_device_config + sizeof(config_header_t),
        g_device_config.header.data_size
    );

    g_config_loaded = true;
    g_config_modified = true;

    xSemaphoreGive(g_config_mutex);

    return CONFIG_SUCCESS;
}

/**
 * @brief  从Flash加载配置
 * @param  None
 * @retval config_result_t 加载结果
 */
config_result_t config_load_from_flash(void)
{
    config_result_t result;

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CONFIG_ERROR_MEMORY_ALLOCATION;
    }

    // 从Flash读取配置数据
    result = config_flash_read_block(CONFIG_FLASH_BASE_ADDR, &g_device_config, sizeof(device_config_t));
    if (result != CONFIG_SUCCESS) {
        xSemaphoreGive(g_config_mutex);
        return result;
    }

    // 验证魔数
    if (g_device_config.header.magic_number != CONFIG_MAGIC_NUMBER) {
        xSemaphoreGive(g_config_mutex);
        return CONFIG_ERROR_VERSION_MISMATCH;
    }

    // 验证版本
    if (g_device_config.header.version != CONFIG_VERSION) {
        xSemaphoreGive(g_config_mutex);
        return CONFIG_ERROR_VERSION_MISMATCH;
    }

    // 验证校验和
    uint32_t calculated_checksum = config_calculate_checksum(
        (uint8_t*)&g_device_config + sizeof(config_header_t),
        g_device_config.header.data_size
    );

    if (calculated_checksum != g_device_config.header.checksum) {
        xSemaphoreGive(g_config_mutex);
        return CONFIG_ERROR_CHECKSUM;
    }

    g_config_loaded = true;
    g_config_modified = false;

    xSemaphoreGive(g_config_mutex);

    // 应用加载的配置
    config_apply_loaded_settings();

    return CONFIG_SUCCESS;
}

/**
 * @brief  保存配置到Flash
 * @param  None
 * @retval config_result_t 保存结果
 */
config_result_t config_save_to_flash(void)
{
    config_result_t result;

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CONFIG_ERROR_MEMORY_ALLOCATION;
    }

    // 更新时间戳
    g_device_config.header.timestamp = xTaskGetTickCount();

    // 重新计算校验和
    g_device_config.header.checksum = config_calculate_checksum(
        (uint8_t*)&g_device_config + sizeof(config_header_t),
        g_device_config.header.data_size
    );

    // 写入Flash
    result = config_flash_write_block(CONFIG_FLASH_BASE_ADDR, &g_device_config, sizeof(device_config_t));
    if (result == CONFIG_SUCCESS) {
        g_config_modified = false;
        g_last_save_time = xTaskGetTickCount();
    }

    xSemaphoreGive(g_config_mutex);

    return result;
}

/**
 * @brief  创建配置备份
 * @param  None
 * @retval config_result_t 备份结果
 */
config_result_t config_backup_create(void)
{
    config_result_t result;

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CONFIG_ERROR_MEMORY_ALLOCATION;
    }

    // 复制当前配置到备份
    memcpy(&g_backup_config, &g_device_config, sizeof(device_config_t));

    // 写入备份区域
    result = config_flash_write_block(CONFIG_BACKUP_ADDR, &g_backup_config, sizeof(device_config_t));

    xSemaphoreGive(g_config_mutex);

    return result;
}

/**
 * @brief  恢复配置备份
 * @param  None
 * @retval config_result_t 恢复结果
 */
config_result_t config_backup_restore(void)
{
    config_result_t result;

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return CONFIG_ERROR_MEMORY_ALLOCATION;
    }

    // 从备份区域读取配置
    result = config_flash_read_block(CONFIG_BACKUP_ADDR, &g_backup_config, sizeof(device_config_t));
    if (result != CONFIG_SUCCESS) {
        xSemaphoreGive(g_config_mutex);
        return result;
    }

    // 验证备份数据
    if (g_backup_config.header.magic_number != CONFIG_MAGIC_NUMBER) {
        xSemaphoreGive(g_config_mutex);
        return CONFIG_ERROR_VERSION_MISMATCH;
    }

    // 恢复配置
    memcpy(&g_device_config, &g_backup_config, sizeof(device_config_t));
    g_config_modified = true;

    xSemaphoreGive(g_config_mutex);

    // 保存恢复的配置
    return config_save_to_flash();
}

/**
 * @brief  获取系统配置
 * @param  config 配置结构指针
 * @retval config_result_t 获取结果
 */
config_result_t config_get_system(system_config_t *config)
{
    if (config == NULL) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(config, &g_device_config.system, sizeof(system_config_t));
        xSemaphoreGive(g_config_mutex);
        return CONFIG_SUCCESS;
    }

    return CONFIG_ERROR_MEMORY_ALLOCATION;
}

/**
 * @brief  设置系统配置
 * @param  config 配置结构指针
 * @retval config_result_t 设置结果
 */
config_result_t config_set_system(const system_config_t *config)
{
    if (config == NULL) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&g_device_config.system, config, sizeof(system_config_t));
        g_config_modified = true;
        xSemaphoreGive(g_config_mutex);
        return CONFIG_SUCCESS;
    }

    return CONFIG_ERROR_MEMORY_ALLOCATION;
}

/**
 * @brief  获取传感器配置
 * @param  config 配置结构指针
 * @retval config_result_t 获取结果
 */
config_result_t config_get_sensor(sensor_config_t *config)
{
    if (config == NULL) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(config, &g_device_config.sensor, sizeof(sensor_config_t));
        xSemaphoreGive(g_config_mutex);
        return CONFIG_SUCCESS;
    }

    return CONFIG_ERROR_MEMORY_ALLOCATION;
}

/**
 * @brief  设置传感器配置
 * @param  config 配置结构指针
 * @retval config_result_t 设置结果
 */
config_result_t config_set_sensor(const sensor_config_t *config)
{
    if (config == NULL) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    // 验证传感器配置参数
    if (config_validate(CONFIG_TYPE_SENSOR, config) != CONFIG_SUCCESS) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&g_device_config.sensor, config, sizeof(sensor_config_t));
        g_config_modified = true;
        xSemaphoreGive(g_config_mutex);
        return CONFIG_SUCCESS;
    }

    return CONFIG_ERROR_MEMORY_ALLOCATION;
}

/**
 * @brief  获取控制配置
 * @param  config 配置结构指针
 * @retval config_result_t 获取结果
 */
config_result_t config_get_control(control_config_t *config)
{
    if (config == NULL) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(config, &g_device_config.control, sizeof(control_config_t));
        xSemaphoreGive(g_config_mutex);
        return CONFIG_SUCCESS;
    }

    return CONFIG_ERROR_MEMORY_ALLOCATION;
}

/**
 * @brief  设置控制配置
 * @param  config 配置结构指针
 * @retval config_result_t 设置结果
 */
config_result_t config_set_control(const control_config_t *config)
{
    if (config == NULL) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    // 验证控制配置参数
    if (config_validate(CONFIG_TYPE_CONTROL, config) != CONFIG_SUCCESS) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&g_device_config.control, config, sizeof(control_config_t));
        g_config_modified = true;
        xSemaphoreGive(g_config_mutex);
        return CONFIG_SUCCESS;
    }

    return CONFIG_ERROR_MEMORY_ALLOCATION;
}

/**
 * @brief  计算校验和
 * @param  data 数据指针
 * @param  size 数据大小
 * @retval uint32_t 校验和
 */
uint32_t config_calculate_checksum(const void *data, uint16_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t checksum = 0;

    for (uint16_t i = 0; i < size; i++) {
        checksum += bytes[i];
    }

    return checksum;
}

/**
 * @brief  验证配置数据
 * @param  type 配置类型
 * @param  data 配置数据
 * @retval config_result_t 验证结果
 */
config_result_t config_validate(config_type_t type, const void *data)
{
    if (data == NULL) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    switch (type) {
        case CONFIG_TYPE_SENSOR:
            {
                const sensor_config_t *sensor = (const sensor_config_t *)data;
                // 验证传感器配置参数范围
                for (int i = 0; i < 7; i++) {
                    if (sensor->calibration_scale[i] <= 0.0f) {
                        return CONFIG_ERROR_INVALID_PARAMETER;
                    }
                    if (sensor->sample_period[i] < 10 || sensor->sample_period[i] > 10000) {
                        return CONFIG_ERROR_INVALID_PARAMETER;
                    }
                }
            }
            break;

        case CONFIG_TYPE_CONTROL:
            {
                const control_config_t *control = (const control_config_t *)data;
                // 验证PID参数范围
                for (int i = 0; i < 3; i++) {
                    if (control->temperature_pid_kp[i] < 0.0f || control->temperature_pid_kp[i] > 100.0f) {
                        return CONFIG_ERROR_INVALID_PARAMETER;
                    }
                }
                for (int i = 0; i < 2; i++) {
                    if (control->pressure_pid_kp[i] < 0.0f || control->pressure_pid_kp[i] > 100.0f) {
                        return CONFIG_ERROR_INVALID_PARAMETER;
                    }
                }
            }
            break;

        default:
            break;
    }

    return CONFIG_SUCCESS;
}

/**
 * @brief  检查配置完整性
 * @param  None
 * @retval config_result_t 检查结果
 */
config_result_t config_check_integrity(void)
{
    if (!g_config_loaded) {
        return CONFIG_ERROR_INVALID_PARAMETER;
    }

    // 重新计算校验和并比较
    uint32_t calculated_checksum = config_calculate_checksum(
        (uint8_t*)&g_device_config + sizeof(config_header_t),
        g_device_config.header.data_size
    );

    if (calculated_checksum != g_device_config.header.checksum) {
        return CONFIG_ERROR_CHECKSUM;
    }

    return CONFIG_SUCCESS;
}

/**
 * @brief  工厂重置
 * @param  None
 * @retval config_result_t 重置结果
 */
config_result_t config_factory_reset(void)
{
    config_result_t result;

    // 加载默认配置
    result = config_load_default();
    if (result != CONFIG_SUCCESS) {
        return result;
    }

    // 保存到Flash
    result = config_save_to_flash();
    if (result != CONFIG_SUCCESS) {
        return result;
    }

    // 创建备份
    return config_backup_create();
}

/**
 * @brief  周期性保存
 * @param  None
 * @retval None
 */
static void config_periodic_save(void)
{
    if (g_config_modified) {
        uint32_t current_time = xTaskGetTickCount();
        if ((current_time - g_last_save_time) >= pdMS_TO_TICKS(g_auto_save_interval)) {
            config_save_to_flash();
        }
    }
}

/**
 * @brief  Flash写入块
 * @param  address 地址
 * @param  data 数据
 * @param  size 大小
 * @retval config_result_t 写入结果
 */
static config_result_t config_flash_write_block(uint32_t address, const void *data, uint16_t size)
{
    // 这里应该实现实际的Flash写入操作
    // 为了演示，这里返回成功
    (void)address;
    (void)data;
    (void)size;
    return CONFIG_SUCCESS;
}

/**
 * @brief  Flash读取块
 * @param  address 地址
 * @param  data 数据
 * @param  size 大小
 * @retval config_result_t 读取结果
 */
static config_result_t config_flash_read_block(uint32_t address, void *data, uint16_t size)
{
    // 这里应该实现实际的Flash读取操作
    // 为了演示，这里返回失败以使用默认配置
    (void)address;
    (void)data;
    (void)size;
    return CONFIG_ERROR_FLASH_READ;
}

/**
 * @brief  应用加载的配置
 * @param  None
 * @retval None
 */
static void config_apply_loaded_settings(void)
{
    // 将加载的配置应用到各个系统模块
    // 例如：更新传感器参数、控制参数等
}

/**
 * @brief  任务初始化
 * @param  None
 * @retval None
 */
static void config_task_init(void)
{
    // 配置任务初始化
}