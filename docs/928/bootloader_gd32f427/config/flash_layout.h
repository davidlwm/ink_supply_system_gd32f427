/**
 ******************************************************************************
 * @file    flash_layout.h
 * @brief   Flash分区布局定义 - GD32F427VGT6 (1MB Flash)
 * @version V1.0
 * @date    2025-09-30
 ******************************************************************************
 * Flash分区方案：
 * +--------------------+
 * | 0x08000000         |  64KB    | Bootloader       | 引导程序（永久）
 * | - 0x0800FFFF       |          |                  | 不可被应用程序擦除
 * +--------------------+
 * | 0x08010000         |  16KB    | Config Sector    | 配置参数区
 * | - 0x08013FFF       |          |                  | 存储运行标志/版本信息
 * +--------------------+
 * | 0x08014000         | 432KB    | APP Bank A       | 应用程序区A
 * | - 0x0807FFFF       |          |                  | 当前运行固件
 * +--------------------+
 * | 0x08080000         | 432KB    | APP Bank B       | 应用程序区B
 * | - 0x080EBFFF       |          |                  | 新固件下载区
 * +--------------------+
 * | 0x080EC000         |  16KB    | Backup Config    | 配置备份区
 * | - 0x080EFFFF       |          |                  | 参数备份
 * +--------------------+
 * | 0x080F0000         |  64KB    | Reserved         | 预留区域
 * | - 0x080FFFFF       |          |                  | （未来扩展）
 * +--------------------+
 * 总计: 1024KB
 ******************************************************************************
 */

#ifndef __FLASH_LAYOUT_H
#define __FLASH_LAYOUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Flash基地址 */
#define FLASH_BASE_ADDR             0x08000000U

/* ========== Bootloader区域 (64KB) ========== */
#define BOOTLOADER_START_ADDR       0x08000000U
#define BOOTLOADER_SIZE             0x00010000U  /* 64KB */
#define BOOTLOADER_END_ADDR         (BOOTLOADER_START_ADDR + BOOTLOADER_SIZE - 1)

/* ========== 配置扇区 (16KB) ========== */
#define CONFIG_SECTOR_ADDR          0x08010000U
#define CONFIG_SECTOR_SIZE          0x00004000U  /* 16KB */
#define CONFIG_SECTOR_END_ADDR      (CONFIG_SECTOR_ADDR + CONFIG_SECTOR_SIZE - 1)

/* ========== 应用程序Bank A (432KB) ========== */
#define APP_BANK_A_START_ADDR       0x08014000U
#define APP_BANK_A_SIZE             0x0006C000U  /* 432KB */
#define APP_BANK_A_END_ADDR         (APP_BANK_A_START_ADDR + APP_BANK_A_SIZE - 1)

/* ========== 应用程序Bank B (432KB) ========== */
#define APP_BANK_B_START_ADDR       0x08080000U
#define APP_BANK_B_SIZE             0x0006C000U  /* 432KB */
#define APP_BANK_B_END_ADDR         (APP_BANK_B_START_ADDR + APP_BANK_B_SIZE - 1)

/* ========== 备份配置区 (16KB) ========== */
#define BACKUP_CONFIG_ADDR          0x080EC000U
#define BACKUP_CONFIG_SIZE          0x00004000U  /* 16KB */
#define BACKUP_CONFIG_END_ADDR      (BACKUP_CONFIG_ADDR + BACKUP_CONFIG_SIZE - 1)

/* ========== 预留区域 (64KB) ========== */
#define RESERVED_AREA_ADDR          0x080F0000U
#define RESERVED_AREA_SIZE          0x00010000U  /* 64KB */
#define RESERVED_AREA_END_ADDR      (RESERVED_AREA_ADDR + RESERVED_AREA_SIZE - 1)

/* ========== 应用程序最大尺寸 ========== */
#define APP_MAX_SIZE                APP_BANK_A_SIZE

/* ========== Flash页大小 (GD32F427) ========== */
/* GD32F427扇区大小分布：
 * 扇区0-3:   16KB  (0x08000000 - 0x0800FFFF)
 * 扇区4:     64KB  (0x08010000 - 0x0801FFFF)
 * 扇区5-11:  128KB (0x08020000 - 0x080FFFFF)
 */
#define FLASH_SECTOR_SIZE_16KB      0x00004000U  /* 16KB */
#define FLASH_SECTOR_SIZE_64KB      0x00010000U  /* 64KB */
#define FLASH_SECTOR_SIZE_128KB     0x00020000U  /* 128KB */

/* ========== Flash总大小 ========== */
#define FLASH_TOTAL_SIZE            0x00100000U  /* 1MB */
#define FLASH_END_ADDR              (FLASH_BASE_ADDR + FLASH_TOTAL_SIZE - 1)

/* ========== 应用程序向量表偏移 ========== */
#define APP_VECTOR_TABLE_OFFSET     (APP_BANK_A_START_ADDR - FLASH_BASE_ADDR)

/* ========== 安全标识 ========== */
/* 应用程序在向量表偏移0x1C0处存放固件标识 "APP_" */
#define APP_MAGIC_OFFSET            0x000001C0U
#define APP_MAGIC_VALUE             0x5F505041U  /* "APP_" in little-endian */

/* ========== GD32F427 SRAM范围 ========== */
#define SRAM_BASE_ADDR              0x20000000U
#define SRAM_SIZE                   0x00030000U  /* 192KB */
#define SRAM_END_ADDR               (SRAM_BASE_ADDR + SRAM_SIZE - 1)

#define CCMRAM_BASE_ADDR            0x10000000U
#define CCMRAM_SIZE                 0x00010000U  /* 64KB */
#define CCMRAM_END_ADDR             (CCMRAM_BASE_ADDR + CCMRAM_SIZE - 1)

/* ========== 宏辅助函数 ========== */
/* 检查地址是否在Bootloader区域 */
#define IS_BOOTLOADER_ADDR(addr)    (((addr) >= BOOTLOADER_START_ADDR) && ((addr) <= BOOTLOADER_END_ADDR))

/* 检查地址是否在Bank A */
#define IS_BANK_A_ADDR(addr)        (((addr) >= APP_BANK_A_START_ADDR) && ((addr) <= APP_BANK_A_END_ADDR))

/* 检查地址是否在Bank B */
#define IS_BANK_B_ADDR(addr)        (((addr) >= APP_BANK_B_START_ADDR) && ((addr) <= APP_BANK_B_END_ADDR))

/* 检查地址是否在配置扇区 */
#define IS_CONFIG_SECTOR_ADDR(addr) (((addr) >= CONFIG_SECTOR_ADDR) && ((addr) <= CONFIG_SECTOR_END_ADDR))

/* 检查地址是否有效 */
#define IS_VALID_FLASH_ADDR(addr)   (((addr) >= FLASH_BASE_ADDR) && ((addr) <= FLASH_END_ADDR))

/* 检查栈指针是否合法 */
#define IS_VALID_STACK_POINTER(sp)  (((sp) >= SRAM_BASE_ADDR) && ((sp) <= SRAM_END_ADDR))

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_LAYOUT_H */