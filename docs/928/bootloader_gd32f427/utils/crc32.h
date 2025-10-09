/**
 ******************************************************************************
 * @file    crc32.h
 * @brief   CRC32计算模块头文件
 * @version V1.0
 * @date    2025-09-30
 ******************************************************************************
 */

#ifndef __CRC32_H
#define __CRC32_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* CRC32初始值和异或值 */
#define CRC32_INIT_VALUE    0xFFFFFFFFU
#define CRC32_XOR_VALUE     0xFFFFFFFFU

/**
 * @brief  计算数据的CRC32校验值
 * @param  data: 数据指针
 * @param  len: 数据长度（字节）
 * @retval CRC32值
 */
uint32_t crc32_calculate(const uint8_t *data, size_t len);

/**
 * @brief  增量计算CRC32（支持分块计算）
 * @param  crc: 当前CRC值（首次调用传入CRC32_INIT_VALUE）
 * @param  data: 数据指针
 * @param  len: 数据长度（字节）
 * @retval 更新后的CRC32值
 */
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len);

/**
 * @brief  完成CRC32计算（对结果进行最终异或）
 * @param  crc: crc32_update返回的CRC值
 * @retval 最终CRC32值
 */
uint32_t crc32_finalize(uint32_t crc);

#ifdef __cplusplus
}
#endif

#endif /* __CRC32_H */