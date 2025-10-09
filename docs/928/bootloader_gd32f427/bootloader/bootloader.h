/**
 ******************************************************************************
 * @file    bootloader.h
 * @brief   Bootloader主程序头文件
 * @version V1.0
 * @date    2025-09-30
 ******************************************************************************
 */

#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Bootloader版本信息 */
#define BOOTLOADER_VERSION_MAJOR    1
#define BOOTLOADER_VERSION_MINOR    0
#define BOOTLOADER_VERSION_PATCH    0
#define BOOTLOADER_VERSION_STRING   "v1.0.0"

/* LED指示状态 */
typedef enum {
    LED_BOOTLOADER_ACTIVE = 0,  /* Bootloader运行中 */
    LED_UPDATE_IN_PROGRESS,     /* 升级进行中 */
    LED_UPDATE_SUCCESS,         /* 升级成功 */
    LED_ERASE_FLASH,            /* 擦除Flash */
    LED_COPY_FIRMWARE,          /* 拷贝固件 */
    LED_JUMP_TO_APP,            /* 跳转应用程序 */
    LED_ERROR,                  /* 错误 */
    LED_EMERGENCY               /* 应急模式 */
} led_state_t;

/**
 * @brief  Bootloader主函数
 * @param  None
 * @retval None（永不返回）
 */
void bootloader_main(void);

/**
 * @brief  硬件初始化
 * @param  None
 * @retval None
 */
void boot_hw_init(void);

/**
 * @brief  检查应急按键
 * @param  None
 * @retval true: 按键按下, false: 按键未按下
 */
bool boot_check_emergency_key(void);

/**
 * @brief  校验应用程序有效性
 * @param  app_addr: 应用程序起始地址
 * @retval true: 有效, false: 无效
 */
bool boot_verify_application(uint32_t app_addr);

/**
 * @brief  跳转到应用程序
 * @param  app_addr: 应用程序起始地址
 * @retval None（不会返回）
 */
void boot_jump_to_application(uint32_t app_addr) __attribute__((noreturn));

/**
 * @brief  LED设置
 * @param  state: LED状态
 * @retval None
 */
void boot_led_set(led_state_t state);

/**
 * @brief  LED翻转
 * @param  state: LED状态
 * @retval None
 */
void boot_led_toggle(led_state_t state);

/**
 * @brief  简单延时
 * @param  ms: 毫秒数
 * @retval None
 */
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H */