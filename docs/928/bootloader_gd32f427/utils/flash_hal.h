/**
 ******************************************************************************
 * @file    flash_hal.h
 * @brief   Flash HAL抽象层头文件
 * @version V1.0
 * @date    2025-09-30
 ******************************************************************************
 */

#ifndef __FLASH_HAL_H
#define __FLASH_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Flash操作返回状态 */
typedef enum {
    FLASH_OK = 0,           /* 操作成功 */
    FLASH_ERROR,            /* 一般错误 */
    FLASH_BUSY,             /* Flash忙 */
    FLASH_TIMEOUT,          /* 超时 */
    FLASH_ADDR_INVALID,     /* 地址无效 */
    FLASH_WRITE_PROTECTED,  /* 写保护 */
    FLASH_ERASE_ERROR,      /* 擦除错误 */
    FLASH_PROGRAM_ERROR     /* 编程错误 */
} flash_status_t;

/**
 * @brief  Flash HAL初始化
 * @param  None
 * @retval None
 */
void flash_hal_init(void);

/**
 * @brief  解锁Flash（允许写入/擦除）
 * @param  None
 * @retval FLASH_OK: 成功, 其他: 失败
 */
flash_status_t flash_unlock(void);

/**
 * @brief  锁定Flash（禁止写入/擦除）
 * @param  None
 * @retval FLASH_OK: 成功, 其他: 失败
 */
flash_status_t flash_lock(void);

/**
 * @brief  擦除Flash扇区
 * @param  sector_addr: 扇区起始地址（必须对齐到扇区边界）
 * @retval FLASH_OK: 成功, 其他: 失败
 */
flash_status_t flash_erase_sector(uint32_t sector_addr);

/**
 * @brief  擦除Flash区域（自动计算扇区数量）
 * @param  start_addr: 起始地址
 * @param  size: 擦除大小（字节）
 * @retval FLASH_OK: 成功, 其他: 失败
 */
flash_status_t flash_erase_region(uint32_t start_addr, uint32_t size);

/**
 * @brief  写入Flash数据（支持任意长度，自动对齐）
 * @param  addr: 目标地址
 * @param  data: 数据指针
 * @param  len: 数据长度（字节）
 * @retval FLASH_OK: 成功, 其他: 失败
 */
flash_status_t flash_write(uint32_t addr, const uint8_t *data, size_t len);

/**
 * @brief  从Flash读取数据
 * @param  addr: 源地址
 * @param  data: 数据缓冲区指针
 * @param  len: 读取长度（字节）
 * @retval FLASH_OK: 成功, 其他: 失败
 */
flash_status_t flash_read(uint32_t addr, uint8_t *data, size_t len);

/**
 * @brief  校验Flash数据
 * @param  addr: Flash地址
 * @param  data: 参考数据
 * @param  len: 长度（字节）
 * @retval true: 一致, false: 不一致
 */
bool flash_verify(uint32_t addr, const uint8_t *data, size_t len);

/**
 * @brief  获取扇区起始地址
 * @param  addr: Flash地址
 * @retval 扇区起始地址
 */
uint32_t flash_get_sector_start(uint32_t addr);

/**
 * @brief  获取扇区大小
 * @param  addr: Flash地址
 * @retval 扇区大小（字节）
 */
uint32_t flash_get_sector_size(uint32_t addr);

/**
 * @brief  检查地址是否对齐
 * @param  addr: Flash地址
 * @param  alignment: 对齐字节数（如4）
 * @retval true: 对齐, false: 未对齐
 */
bool flash_is_aligned(uint32_t addr, uint32_t alignment);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_HAL_H */