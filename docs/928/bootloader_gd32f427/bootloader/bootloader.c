/**
 ******************************************************************************
 * @file    bootloader.c
 * @brief   Bootloader主程序实现
 * @version V1.0
 * @date    2025-09-30
 ******************************************************************************
 * 功能说明：
 * 1. 上电后初始化最小系统
 * 2. 读取配置扇区，判断启动模式
 * 3. 如果有升级标志，从Bank B拷贝固件到Bank A
 * 4. 校验应用程序完整性
 * 5. 跳转到应用程序
 * 6. 如果失败，进入应急模式
 ******************************************************************************
 */

#include "bootloader.h"
#include "boot_config.h"
#include "flash_layout.h"
#include "flash_hal.h"
#include "crc32.h"
#include "gd32f4xx.h"
#include <stdio.h>
#include <string.h>

/* 全局变量 */
static boot_config_t g_boot_config;

/* 前向声明 */
static bool boot_handle_update(void);
static bool boot_copy_firmware(uint32_t src_addr, uint32_t dst_addr, uint32_t size);
static bool boot_verify_firmware(uint32_t addr, uint32_t size, uint32_t expected_crc);
static void boot_enter_emergency_mode(void) __attribute__((noreturn));

/**
 * @brief  Bootloader主函数
 * @param  None
 * @retval None
 */
void bootloader_main(void)
{
    /* 1. 硬件初始化 */
    boot_hw_init();

    /* 2. LED指示进入Bootloader */
    boot_led_set(LED_BOOTLOADER_ACTIVE);
    delay_ms(100);

    /* 3. 初始化Flash HAL */
    flash_hal_init();

    /* 4. 读取配置扇区 */
    if (!boot_config_load(&g_boot_config)) {
        /* 配置损坏，使用默认配置 */
        boot_config_reset(&g_boot_config);
        boot_config_save(&g_boot_config);
    }

    /* 5. 增加启动计数 */
    g_boot_config.boot_count++;

    /* 6. 打印Bootloader信息（可选，需要UART初始化） */
    #ifdef BOOTLOADER_DEBUG
    printf("\r\n\r\n");
    printf("========================================\r\n");
    printf(" GD32F427 Bootloader %s\r\n", BOOTLOADER_VERSION_STRING);
    printf("========================================\r\n");
    printf("Boot Count: %d\r\n", g_boot_config.boot_count);
    printf("Boot Flag:  %d\r\n", g_boot_config.boot_flag);
    printf("\r\n");
    #endif

    /* 7. 检查是否按住应急按键（进入应急模式） */
    if (boot_check_emergency_key()) {
        boot_enter_emergency_mode();
        /* 不会返回 */
    }

    /* 8. 处理升级流程 */
    if (g_boot_config.boot_flag == BOOT_FLAG_UPDATE_PENDING) {
        if (!boot_handle_update()) {
            /* 升级失败，增加失败计数 */
            g_boot_config.consecutive_boot_fail++;
            g_boot_config.boot_flag = BOOT_FLAG_UPDATE_FAILED;
            boot_config_save(&g_boot_config);

            /* 连续失败3次，进入应急模式 */
            if (g_boot_config.consecutive_boot_fail >= MAX_BOOT_FAIL_COUNT) {
                boot_enter_emergency_mode();
            }
        } else {
            /* 升级成功，重置失败计数 */
            g_boot_config.consecutive_boot_fail = 0;
        }
    }

    /* 9. 校验应用程序 */
    if (!boot_verify_application(APP_BANK_A_START_ADDR)) {
        /* 应用程序损坏 */
        boot_led_set(LED_ERROR);
        g_boot_config.consecutive_boot_fail++;
        boot_config_save(&g_boot_config);

        #ifdef BOOTLOADER_DEBUG
        printf("ERROR: Application verification failed!\r\n");
        #endif

        /* 连续失败，进入应急模式 */
        if (g_boot_config.consecutive_boot_fail >= MAX_BOOT_FAIL_COUNT) {
            boot_enter_emergency_mode();
        }

        /* 等待复位 */
        while (1) {
            boot_led_toggle(LED_ERROR);
            delay_ms(200);
        }
    }

    /* 10. 保存配置 */
    boot_config_save(&g_boot_config);

    /* 11. 跳转到应用程序 */
    boot_led_set(LED_JUMP_TO_APP);
    delay_ms(100);

    #ifdef BOOTLOADER_DEBUG
    printf("Jumping to application @ 0x%08X...\r\n\r\n", APP_BANK_A_START_ADDR);
    #endif

    boot_jump_to_application(APP_BANK_A_START_ADDR);

    /* 不应该到达这里 */
    while (1) {
        boot_led_toggle(LED_ERROR);
        delay_ms(100);
    }
}

/**
 * @brief  处理升级流程
 * @param  None
 * @retval true: 成功, false: 失败
 */
static bool boot_handle_update(void)
{
    boot_led_set(LED_UPDATE_IN_PROGRESS);

    #ifdef BOOTLOADER_DEBUG
    printf("=== Firmware Update Start ===\r\n");
    #endif

    /* 设置升级进行中标志 */
    g_boot_config.boot_flag = BOOT_FLAG_UPDATE_IN_PROGRESS;
    boot_config_save(&g_boot_config);

    /* 1. 校验Bank B的新固件 */
    #ifdef BOOTLOADER_DEBUG
    printf("Verifying new firmware...\r\n");
    #endif

    if (!boot_verify_firmware(APP_BANK_B_START_ADDR,
                               g_boot_config.bank_b_info.size,
                               g_boot_config.bank_b_info.crc32)) {
        /* 新固件校验失败 */
        boot_led_set(LED_ERROR);
        g_boot_config.boot_flag = BOOT_FLAG_UPDATE_FAILED;
        g_boot_config.rollback_count++;

        #ifdef BOOTLOADER_DEBUG
        printf("ERROR: New firmware verification failed!\r\n");
        #endif

        return false;
    }

    /* 2. 擦除Bank A */
    #ifdef BOOTLOADER_DEBUG
    printf("Erasing Bank A...\r\n");
    #endif

    boot_led_set(LED_ERASE_FLASH);
    if (flash_erase_region(APP_BANK_A_START_ADDR, APP_BANK_A_SIZE) != FLASH_OK) {
        g_boot_config.boot_flag = BOOT_FLAG_UPDATE_FAILED;

        #ifdef BOOTLOADER_DEBUG
        printf("ERROR: Bank A erase failed!\r\n");
        #endif

        return false;
    }

    /* 3. 从Bank B拷贝到Bank A */
    #ifdef BOOTLOADER_DEBUG
    printf("Copying firmware from Bank B to Bank A...\r\n");
    #endif

    boot_led_set(LED_COPY_FIRMWARE);
    if (!boot_copy_firmware(APP_BANK_B_START_ADDR,
                            APP_BANK_A_START_ADDR,
                            g_boot_config.bank_b_info.size)) {
        g_boot_config.boot_flag = BOOT_FLAG_UPDATE_FAILED;

        #ifdef BOOTLOADER_DEBUG
        printf("ERROR: Firmware copy failed!\r\n");
        #endif

        return false;
    }

    /* 4. 再次校验Bank A */
    #ifdef BOOTLOADER_DEBUG
    printf("Verifying Bank A...\r\n");
    #endif

    if (!boot_verify_firmware(APP_BANK_A_START_ADDR,
                              g_boot_config.bank_b_info.size,
                              g_boot_config.bank_b_info.crc32)) {
        g_boot_config.boot_flag = BOOT_FLAG_UPDATE_FAILED;

        #ifdef BOOTLOADER_DEBUG
        printf("ERROR: Bank A verification failed!\r\n");
        #endif

        return false;
    }

    /* 5. 升级成功 */
    boot_led_set(LED_UPDATE_SUCCESS);
    g_boot_config.boot_flag = BOOT_FLAG_NORMAL_RUN;
    g_boot_config.active_bank = 0;  /* Bank A */
    g_boot_config.bank_a_info = g_boot_config.bank_b_info;
    g_boot_config.update_count++;
    g_boot_config.last_update_time = 0; /* 需要RTC支持 */

    #ifdef BOOTLOADER_DEBUG
    printf("Firmware update SUCCESS!\r\n");
    printf("New version: %d.%d.%d\r\n",
           GET_VERSION_MAJOR(g_boot_config.bank_a_info.version),
           GET_VERSION_MINOR(g_boot_config.bank_a_info.version),
           GET_VERSION_PATCH(g_boot_config.bank_a_info.version));
    #endif

    delay_ms(500);
    return true;
}

/**
 * @brief  拷贝固件
 * @param  src_addr: 源地址
 * @param  dst_addr: 目标地址
 * @param  size: 大小
 * @retval true: 成功, false: 失败
 */
static bool boot_copy_firmware(uint32_t src_addr, uint32_t dst_addr, uint32_t size)
{
    uint8_t buffer[256];
    uint32_t offset = 0;

    while (offset < size) {
        uint32_t chunk_size = (size - offset) > 256 ? 256 : (size - offset);

        /* 读取源数据 */
        if (flash_read(src_addr + offset, buffer, chunk_size) != FLASH_OK) {
            return false;
        }

        /* 写入目标 */
        if (flash_write(dst_addr + offset, buffer, chunk_size) != FLASH_OK) {
            return false;
        }

        offset += chunk_size;
    }

    return true;
}

/**
 * @brief  校验固件
 * @param  addr: 固件地址
 * @param  size: 固件大小
 * @param  expected_crc: 期望的CRC32值
 * @retval true: 成功, false: 失败
 */
static bool boot_verify_firmware(uint32_t addr, uint32_t size, uint32_t expected_crc)
{
    if (size == 0 || size > APP_MAX_SIZE) {
        return false;
    }

    if (expected_crc == 0) {
        return false;
    }

    uint32_t calc_crc = crc32_calculate((uint8_t*)addr, size);
    return (calc_crc == expected_crc);
}

/**
 * @brief  校验应用程序有效性
 * @param  app_addr: 应用程序起始地址
 * @retval true: 有效, false: 无效
 */
bool boot_verify_application(uint32_t app_addr)
{
    /* 1. 检查栈指针合法性 */
    uint32_t sp = *(volatile uint32_t*)app_addr;
    if (!IS_VALID_STACK_POINTER(sp)) {
        return false;
    }

    /* 2. 检查复位向量合法性 */
    uint32_t pc = *(volatile uint32_t*)(app_addr + 4);
    if (pc < app_addr || pc > (app_addr + APP_MAX_SIZE)) {
        return false;
    }

    /* 3. 检查固件魔术字（可选） */
    uint32_t app_magic = *(volatile uint32_t*)(app_addr + APP_MAGIC_OFFSET);
    if (app_magic != APP_MAGIC_VALUE) {
        /* 警告：固件可能不是标准格式 */
        #ifdef BOOTLOADER_DEBUG
        printf("Warning: APP magic not found\r\n");
        #endif
    }

    /* 4. 校验CRC32（如果配置中有记录） */
    if (g_boot_config.bank_a_info.crc32 != 0 && g_boot_config.bank_a_info.size != 0) {
        uint32_t calc_crc = crc32_calculate((uint8_t*)app_addr,
                                            g_boot_config.bank_a_info.size);
        if (calc_crc != g_boot_config.bank_a_info.crc32) {
            return false;
        }
    }

    return true;
}

/**
 * @brief  跳转到应用程序
 * @param  app_addr: 应用程序起始地址
 * @retval None（不会返回）
 */
void boot_jump_to_application(uint32_t app_addr)
{
    /* 关闭所有中断 */
    __disable_irq();

    /* 关闭SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /* 清除所有中断标志 */
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* 设置向量表偏移 */
    SCB->VTOR = app_addr;

    /* 读取应用程序栈指针和复位向量 */
    uint32_t app_sp = *(volatile uint32_t*)app_addr;
    uint32_t app_pc = *(volatile uint32_t*)(app_addr + 4);

    /* 设置主栈指针 */
    __set_MSP(app_sp);

    /* 跳转到应用程序 */
    void (*app_reset_handler)(void) = (void (*)(void))app_pc;
    app_reset_handler();

    /* 不应该返回 */
    while (1);
}

/**
 * @brief  应急模式
 * @param  None
 * @retval None（不会返回）
 */
static void boot_enter_emergency_mode(void)
{
    g_boot_config.boot_flag = BOOT_FLAG_EMERGENCY_MODE;
    boot_config_save(&g_boot_config);

    #ifdef BOOTLOADER_DEBUG
    printf("\r\n");
    printf("========================================\r\n");
    printf(" === EMERGENCY MODE ===\r\n");
    printf("========================================\r\n");
    printf("Bootloader Version: %s\r\n", BOOTLOADER_VERSION_STRING);
    printf("Waiting for firmware upload...\r\n");
    printf("\r\n");
    #endif

    /* 进入永久循环，等待固件上传 */
    /* 实际应用中，这里应启动UART/TCP服务器等待固件 */
    while (1) {
        boot_led_toggle(LED_EMERGENCY);
        delay_ms(500);
    }
}

/**
 * @brief  硬件初始化（最小系统）
 * @param  None
 * @retval None
 */
void boot_hw_init(void)
{
    /* 系统时钟已由启动代码初始化 */

    /* 初始化LED GPIO（根据硬件调整） */
    /* 这里需要根据实际硬件配置LED引脚 */
    /* 示例：使能GPIOA时钟 */
    /* RCC->AHB1ENR |= (1 << 0);  */ /* 使能GPIOA时钟 */

    /* 可选：初始化UART用于调试输出 */
    #ifdef BOOTLOADER_DEBUG
    /* 初始化UART0作为调试串口 */
    /* ... UART初始化代码 ... */
    #endif
}

/**
 * @brief  检查应急按键
 * @param  None
 * @retval true: 按下, false: 未按下
 */
bool boot_check_emergency_key(void)
{
    /* 这里需要根据实际硬件配置按键检测 */
    /* 示例：检查PA0按键是否按下 */

    /* 暂时返回false */
    return false;
}

/**
 * @brief  LED设置
 * @param  state: LED状态
 * @retval None
 */
void boot_led_set(led_state_t state)
{
    /* 根据实际硬件实现LED控制 */
    /* 示例：不同状态使用不同LED或闪烁模式 */
}

/**
 * @brief  LED翻转
 * @param  state: LED状态
 * @retval None
 */
void boot_led_toggle(led_state_t state)
{
    /* 根据实际硬件实现LED翻转 */
}

/**
 * @brief  简单延时
 * @param  ms: 毫秒数
 * @retval None
 */
void delay_ms(uint32_t ms)
{
    /* 简单的延时实现（基于系统时钟） */
    /* SystemCoreClock = 200MHz */
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < (SystemCoreClock / 8000); j++);
    }
}