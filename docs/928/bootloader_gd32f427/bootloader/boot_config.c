/**
 ******************************************************************************
 * @file    boot_config.c
 * @brief   Bootloader配置管理实现
 * @version V1.0
 * @date    2025-09-30
 ******************************************************************************
 */

#include "boot_config.h"
#include "flash_layout.h"
#include "flash_hal.h"
#include "crc32.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief  初始化配置为默认值
 * @param  config: 配置结构体指针
 * @retval None
 */
void boot_config_reset(boot_config_t *config)
{
    memset(config, 0, sizeof(boot_config_t));

    /* 设置魔术字和版本 */
    config->magic = BOOT_CONFIG_MAGIC;
    config->config_version = BOOT_CONFIG_VERSION;

    /* 设置默认启动标志 */
    config->boot_flag = BOOT_FLAG_NORMAL_RUN;
    config->active_bank = 0;  /* Bank A */

    /* 初始化Bank A信息（假设已有固件） */
    config->bank_a_info.version = MAKE_VERSION(1, 0, 0);
    config->bank_a_info.size = 0;
    config->bank_a_info.crc32 = 0;
    config->bank_a_info.timestamp = 0;
    strncpy(config->bank_a_info.description, "Factory Firmware",
            sizeof(config->bank_a_info.description));

    /* 初始化Bank B信息 */
    config->bank_b_info.version = 0;
    config->bank_b_info.size = 0;
    config->bank_b_info.crc32 = 0;
    config->bank_b_info.timestamp = 0;
    memset(config->bank_b_info.description, 0, sizeof(config->bank_b_info.description));

    /* 初始化统计信息 */
    config->update_count = 0;
    config->boot_count = 0;
    config->last_update_time = 0;

    /* 回退设置 */
    config->rollback_count = 0;
    config->max_rollback = DEFAULT_MAX_ROLLBACK;
    config->consecutive_boot_fail = 0;

    /* 安全特性 */
    config->secure_boot_enable = 0;  /* 默认关闭 */
    config->min_fw_version = MAKE_VERSION(1, 0, 0);

    /* 应急模式配置 */
    config->emergency_mode_timeout = DEFAULT_EMERGENCY_TIMEOUT;
    config->emergency_key_gpio = 0;  /* 需根据硬件配置 */

    /* 计算CRC32 */
    config->config_crc32 = crc32_calculate((uint8_t*)config,
                                           sizeof(boot_config_t) - sizeof(uint32_t));
}

/**
 * @brief  校验配置完整性
 * @param  config: 配置结构体指针
 * @retval true: 有效, false: 无效
 */
bool boot_config_verify(const boot_config_t *config)
{
    /* 检查魔术字 */
    if (config->magic != BOOT_CONFIG_MAGIC) {
        return false;
    }

    /* 检查配置版本 */
    if (config->config_version != BOOT_CONFIG_VERSION) {
        /* 可以支持版本升级兼容 */
    }

    /* 校验CRC32 */
    uint32_t calc_crc = crc32_calculate((const uint8_t*)config,
                                        sizeof(boot_config_t) - sizeof(uint32_t));
    if (calc_crc != config->config_crc32) {
        return false;
    }

    return true;
}

/**
 * @brief  从Flash加载配置
 * @param  config: 配置结构体指针
 * @retval true: 成功, false: 失败
 */
bool boot_config_load(boot_config_t *config)
{
    flash_status_t status;

    /* 从配置扇区读取 */
    status = flash_read(CONFIG_SECTOR_ADDR, (uint8_t*)config, sizeof(boot_config_t));
    if (status != FLASH_OK) {
        return false;
    }

    /* 校验配置 */
    if (!boot_config_verify(config)) {
        /* 配置无效，尝试从备份恢复 */
        if (boot_config_restore(config)) {
            /* 备份有效，保存到主配置区 */
            boot_config_save(config);
            return true;
        }
        return false;
    }

    return true;
}

/**
 * @brief  保存配置到Flash
 * @param  config: 配置结构体指针
 * @retval true: 成功, false: 失败
 */
bool boot_config_save(const boot_config_t *config)
{
    flash_status_t status;
    boot_config_t temp_config;

    /* 拷贝配置并计算CRC */
    memcpy(&temp_config, config, sizeof(boot_config_t));
    temp_config.config_crc32 = crc32_calculate((uint8_t*)&temp_config,
                                               sizeof(boot_config_t) - sizeof(uint32_t));

    /* 擦除配置扇区 */
    status = flash_erase_sector(CONFIG_SECTOR_ADDR);
    if (status != FLASH_OK) {
        return false;
    }

    /* 写入配置 */
    status = flash_write(CONFIG_SECTOR_ADDR, (const uint8_t*)&temp_config,
                        sizeof(boot_config_t));
    if (status != FLASH_OK) {
        return false;
    }

    /* 校验写入是否成功 */
    if (!flash_verify(CONFIG_SECTOR_ADDR, (const uint8_t*)&temp_config,
                     sizeof(boot_config_t))) {
        return false;
    }

    return true;
}

/**
 * @brief  备份配置到备份扇区
 * @param  config: 配置结构体指针
 * @retval true: 成功, false: 失败
 */
bool boot_config_backup(const boot_config_t *config)
{
    flash_status_t status;
    boot_config_t temp_config;

    /* 拷贝配置并计算CRC */
    memcpy(&temp_config, config, sizeof(boot_config_t));
    temp_config.config_crc32 = crc32_calculate((uint8_t*)&temp_config,
                                               sizeof(boot_config_t) - sizeof(uint32_t));

    /* 擦除备份扇区 */
    status = flash_erase_sector(BACKUP_CONFIG_ADDR);
    if (status != FLASH_OK) {
        return false;
    }

    /* 写入备份 */
    status = flash_write(BACKUP_CONFIG_ADDR, (const uint8_t*)&temp_config,
                        sizeof(boot_config_t));
    if (status != FLASH_OK) {
        return false;
    }

    return true;
}

/**
 * @brief  从备份扇区恢复配置
 * @param  config: 配置结构体指针
 * @retval true: 成功, false: 失败
 */
bool boot_config_restore(boot_config_t *config)
{
    flash_status_t status;

    /* 从备份扇区读取 */
    status = flash_read(BACKUP_CONFIG_ADDR, (uint8_t*)config, sizeof(boot_config_t));
    if (status != FLASH_OK) {
        return false;
    }

    /* 校验备份配置 */
    return boot_config_verify(config);
}

/**
 * @brief  打印配置信息（调试用）
 * @param  config: 配置结构体指针
 * @retval None
 */
void boot_config_print(const boot_config_t *config)
{
    printf("\r\n========== Boot Configuration ==========\r\n");
    printf("Magic:            0x%08X\r\n", config->magic);
    printf("Config Version:   0x%04X\r\n", config->config_version);
    printf("Boot Flag:        %d\r\n", config->boot_flag);
    printf("Active Bank:      %d\r\n", config->active_bank);

    printf("\r\n--- Bank A Info ---\r\n");
    printf("Version:          %d.%d.%d\r\n",
           GET_VERSION_MAJOR(config->bank_a_info.version),
           GET_VERSION_MINOR(config->bank_a_info.version),
           GET_VERSION_PATCH(config->bank_a_info.version));
    printf("Size:             %d bytes\r\n", config->bank_a_info.size);
    printf("CRC32:            0x%08X\r\n", config->bank_a_info.crc32);
    printf("Description:      %s\r\n", config->bank_a_info.description);

    printf("\r\n--- Bank B Info ---\r\n");
    printf("Version:          %d.%d.%d\r\n",
           GET_VERSION_MAJOR(config->bank_b_info.version),
           GET_VERSION_MINOR(config->bank_b_info.version),
           GET_VERSION_PATCH(config->bank_b_info.version));
    printf("Size:             %d bytes\r\n", config->bank_b_info.size);
    printf("CRC32:            0x%08X\r\n", config->bank_b_info.crc32);

    printf("\r\n--- Statistics ---\r\n");
    printf("Update Count:     %d\r\n", config->update_count);
    printf("Boot Count:       %d\r\n", config->boot_count);
    printf("Rollback Count:   %d\r\n", config->rollback_count);
    printf("Boot Fail Count:  %d\r\n", config->consecutive_boot_fail);

    printf("========================================\r\n\r\n");
}