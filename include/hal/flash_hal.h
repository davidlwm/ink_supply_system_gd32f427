/**
 * @file flash_hal.h
 * @brief Flash Hardware Abstraction Layer
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description Flash硬件抽象层，基于GD32F4xx HAL库
 *              提供配置存储、固件升级、数据持久化功能
 */

#ifndef FLASH_HAL_H
#define FLASH_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"
#include "gd32f4xx_fmc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 宏定义 */
#define FLASH_SECTOR_SIZE           0x20000     /* 128KB扇区大小 */
#define FLASH_PAGE_SIZE             0x800       /* 2KB页大小 */
#define FLASH_WORD_SIZE             4           /* 32位字大小 */
#define FLASH_MAX_PROGRAM_SIZE      256         /* 最大编程大小 */
#define FLASH_ERASE_TIMEOUT_MS      10000       /* 擦除超时时间 */
#define FLASH_PROGRAM_TIMEOUT_MS    1000        /* 编程超时时间 */

/* Flash扇区定义 */
#define FLASH_SECTOR_0              FMC_SECTOR_0    /* 16KB */
#define FLASH_SECTOR_1              FMC_SECTOR_1    /* 16KB */
#define FLASH_SECTOR_2              FMC_SECTOR_2    /* 16KB */
#define FLASH_SECTOR_3              FMC_SECTOR_3    /* 16KB */
#define FLASH_SECTOR_4              FMC_SECTOR_4    /* 64KB */
#define FLASH_SECTOR_5              FMC_SECTOR_5    /* 128KB */
#define FLASH_SECTOR_6              FMC_SECTOR_6    /* 128KB */
#define FLASH_SECTOR_7              FMC_SECTOR_7    /* 128KB */

/* Flash地址定义 */
#define FLASH_BASE_ADDR             0x08000000
#define FLASH_CONFIG_SECTOR         FLASH_SECTOR_6  /* 配置存储扇区 */
#define FLASH_CONFIG_ADDR           0x08040000      /* 配置存储地址 */
#define FLASH_BACKUP_SECTOR         FLASH_SECTOR_7  /* 备份存储扇区 */
#define FLASH_BACKUP_ADDR           0x08060000      /* 备份存储地址 */

/* 错误码定义 */
typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR = 1,
    FLASH_BUSY = 2,
    FLASH_TIMEOUT = 3,
    FLASH_INVALID_PARAM = 4,
    FLASH_WRITE_PROTECTED = 5,
    FLASH_VERIFY_FAILED = 6,
    FLASH_ERASE_FAILED = 7,
    FLASH_PROGRAM_FAILED = 8,
    FLASH_ALIGNMENT_ERROR = 9
} flash_result_t;

/* Flash保护模式 */
typedef enum {
    FLASH_PROTECTION_NONE = 0,
    FLASH_PROTECTION_READ = 1,
    FLASH_PROTECTION_WRITE = 2,
    FLASH_PROTECTION_FULL = 3
} flash_protection_t;

/* Flash状态 */
typedef enum {
    FLASH_STATE_READY = 0,
    FLASH_STATE_BUSY = 1,
    FLASH_STATE_ERROR = 2
} flash_state_t;

/* 配置数据结构体 */
typedef struct {
    uint32_t magic;                 /* 魔术字 0x12345678 */
    uint32_t version;               /* 配置版本 */
    uint32_t crc32;                 /* CRC32校验 */
    uint32_t length;                /* 数据长度 */
    uint8_t data[4080];             /* 配置数据 (4KB - 16字节头部) */
} flash_config_t;

/* Flash信息结构体 */
typedef struct {
    uint32_t total_size;            /* 总容量 */
    uint32_t sector_count;          /* 扇区数量 */
    uint32_t page_size;             /* 页大小 */
    uint32_t sector_size;           /* 扇区大小 */
    flash_state_t state;            /* Flash状态 */
    uint32_t error_count;           /* 错误计数 */
} flash_info_t;

/* Flash操作回调函数类型 */
typedef void (*flash_erase_callback_t)(uint32_t sector, flash_result_t result);
typedef void (*flash_program_callback_t)(uint32_t address, uint32_t length, flash_result_t result);

/* 初始化和配置 */
flash_result_t flash_hal_init(void);
flash_result_t flash_hal_deinit(void);
flash_result_t flash_hal_lock(void);
flash_result_t flash_hal_unlock(void);
bool flash_hal_is_locked(void);

/* 擦除操作 */
flash_result_t flash_hal_erase_sector(uint32_t sector);
flash_result_t flash_hal_erase_sectors(uint32_t start_sector, uint32_t end_sector);
flash_result_t flash_hal_erase_mass(void);
flash_result_t flash_hal_erase_address_range(uint32_t start_addr, uint32_t end_addr);

/* 编程操作 */
flash_result_t flash_hal_program_word(uint32_t address, uint32_t data);
flash_result_t flash_hal_program_halfword(uint32_t address, uint16_t data);
flash_result_t flash_hal_program_byte(uint32_t address, uint8_t data);
flash_result_t flash_hal_program_buffer(uint32_t address, const uint8_t* data, uint32_t length);

/* 读取操作 */
flash_result_t flash_hal_read_word(uint32_t address, uint32_t* data);
flash_result_t flash_hal_read_halfword(uint32_t address, uint16_t* data);
flash_result_t flash_hal_read_byte(uint32_t address, uint8_t* data);
flash_result_t flash_hal_read_buffer(uint32_t address, uint8_t* data, uint32_t length);

/* 验证操作 */
flash_result_t flash_hal_verify_word(uint32_t address, uint32_t data);
flash_result_t flash_hal_verify_buffer(uint32_t address, const uint8_t* data, uint32_t length);
flash_result_t flash_hal_blank_check_sector(uint32_t sector);
flash_result_t flash_hal_blank_check_range(uint32_t start_addr, uint32_t end_addr);

/* 保护操作 */
flash_result_t flash_hal_set_protection(uint32_t sector, flash_protection_t protection);
flash_result_t flash_hal_get_protection(uint32_t sector, flash_protection_t* protection);
flash_result_t flash_hal_disable_write_protection(uint32_t sectors);
flash_result_t flash_hal_enable_write_protection(uint32_t sectors);

/* 状态和信息 */
flash_result_t flash_hal_get_info(flash_info_t* info);
flash_state_t flash_hal_get_state(void);
bool flash_hal_is_busy(void);
flash_result_t flash_hal_wait_ready(uint32_t timeout_ms);
uint32_t flash_hal_get_error_count(void);
flash_result_t flash_hal_clear_error_count(void);

/* 配置管理 */
flash_result_t flash_hal_save_config(const void* config, uint32_t length);
flash_result_t flash_hal_load_config(void* config, uint32_t max_length, uint32_t* actual_length);
flash_result_t flash_hal_erase_config(void);
bool flash_hal_is_config_valid(void);
flash_result_t flash_hal_backup_config(void);
flash_result_t flash_hal_restore_config(void);

/* CRC计算 */
uint32_t flash_hal_calculate_crc32(uint32_t address, uint32_t length);
flash_result_t flash_hal_verify_crc32(uint32_t address, uint32_t length, uint32_t expected_crc);

/* 地址转换 */
uint32_t flash_hal_sector_to_address(uint32_t sector);
uint32_t flash_hal_address_to_sector(uint32_t address);
bool flash_hal_is_valid_address(uint32_t address);
bool flash_hal_is_aligned_address(uint32_t address, uint32_t alignment);

/* 回调函数 */
flash_result_t flash_hal_register_erase_callback(flash_erase_callback_t callback);
flash_result_t flash_hal_register_program_callback(flash_program_callback_t callback);
flash_result_t flash_hal_unregister_erase_callback(void);
flash_result_t flash_hal_unregister_program_callback(void);

/* 诊断功能 */
flash_result_t flash_hal_self_test(void);
flash_result_t flash_hal_read_device_id(uint32_t* device_id);
flash_result_t flash_hal_read_unique_id(uint8_t* unique_id, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_HAL_H */