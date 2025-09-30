/**
 ******************************************************************************
 * @file    main.c
 * @brief   Bootloader主入口（简化版）
 * @version V1.0
 * @date    2025-09-30
 ******************************************************************************
 * GD32F427 Bootloader
 * 功能：
 * - 双Bank固件升级
 * - CRC32校验
 * - 应急恢复模式
 ******************************************************************************
 */

#include "gd32f4xx.h"
#include "bootloader.h"
#include <stdio.h>

/* Bootloader版本信息段（可选） */
__attribute__((section(".bootloader_info"), used))
const struct {
    uint32_t magic;         /* 0x424F4F54 "BOOT" */
    uint32_t version;       /* v1.0.0 */
    uint32_t build_date;    /* 编译日期时间戳 */
    char description[32];   /* 描述 */
} bootloader_info = {
    .magic = 0x424F4F54U,
    .version = 0x010000U,   /* 1.0.0 */
    .build_date = 0,        /* 可通过编译脚本注入 */
    .description = "GD32F427 Bootloader v1.0"
};

/**
 * @brief  系统时钟配置（简化版，使用默认配置）
 * @param  None
 * @retval None
 */
static void system_clock_config(void)
{
    /* 使用启动代码的默认时钟配置 */
    /* 如需精确配置，请添加GD32标准库或HAL库支持 */
}

/**
 * @brief  主函数
 * @param  None
 * @retval None（永不返回）
 */
int main(void)
{
    /* 配置系统时钟 */
    system_clock_config();

    /* 进入Bootloader主程序 */
    bootloader_main();

    /* 不应该到达这里 */
    while (1);
}

/**
 * @brief  断言失败处理
 * @param  file: 源文件名
 * @param  line: 行号
 * @retval None
 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    printf("Assert failed: file %s on line %d\r\n", file, line);
    while (1);
}
#endif

/* 重定向printf到UART（可选） */
#ifdef BOOTLOADER_DEBUG

/* 需要实现的函数 */
int fputc(int ch, FILE *f)
{
    /* 这里需要实现UART发送一个字节 */
    /* 示例（需要先初始化USART0）：
     * usart_data_transmit(USART0, (uint8_t)ch);
     * while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
     */
    return ch;
}

int fgetc(FILE *f)
{
    /* 这里需要实现UART接收一个字节 */
    /* 示例：
     * while(RESET == usart_flag_get(USART0, USART_FLAG_RBNE));
     * return (int)usart_data_receive(USART0);
     */
    return 0;
}

#endif