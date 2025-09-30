/**
 ******************************************************************************
 * @file    flash_hal_simple.c
 * @brief   Flash HAL简化实现（直接寄存器操作）
 * @version V1.0
 * @date    2025-09-30
 ******************************************************************************
 * 说明：
 * 这是一个简化版实现，使用CMSIS直接操作Flash寄存器
 * 完整版本需要添加GD32标准库支持
 ******************************************************************************
 */

#include "flash_hal.h"
#include "flash_layout.h"
#include <string.h>

/* Flash操作超时时间 */
#define FLASH_TIMEOUT_MS        5000

/* Flash寄存器基地址 */
#define FLASH_BASE              0x40023C00U
#define FLASH                   ((FLASH_TypeDef *) FLASH_BASE)

/* Flash寄存器结构（简化） */
typedef struct {
    volatile uint32_t ACR;      /* 0x00 */
    volatile uint32_t KEYR;     /* 0x04 */
    volatile uint32_t OPTKEYR;  /* 0x08 */
    volatile uint32_t SR;       /* 0x0C */
    volatile uint32_t CR;       /* 0x10 */
    volatile uint32_t OPTCR;    /* 0x14 */
    volatile uint32_t OPTCR1;   /* 0x18 */
} FLASH_TypeDef;

/* Flash解锁键 */
#define FLASH_KEY1             0x45670123U
#define FLASH_KEY2             0xCDEF89ABU

/* Flash CR寄存器位 */
#define FLASH_CR_PG            (1U << 0)
#define FLASH_CR_SER           (1U << 1)
#define FLASH_CR_MER           (1U << 2)
#define FLASH_CR_SNB_Pos       3
#define FLASH_CR_SNB_Msk       (0xFU << FLASH_CR_SNB_Pos)
#define FLASH_CR_PSIZE_Pos     8
#define FLASH_CR_PSIZE_Msk     (0x3U << FLASH_CR_PSIZE_Pos)
#define FLASH_CR_STRT          (1U << 16)
#define FLASH_CR_LOCK          (1U << 31)

/* Flash SR寄存器位 */
#define FLASH_SR_BSY           (1U << 16)
#define FLASH_SR_EOP           (1U << 0)

/**
 * @brief  Flash HAL初始化
 */
void flash_hal_init(void)
{
    /* 不需要特殊初始化 */
}

/**
 * @brief  解锁Flash
 */
flash_status_t flash_unlock(void)
{
    if (!(FLASH->CR & FLASH_CR_LOCK)) {
        return FLASH_OK;  /* 已解锁 */
    }

    /* 写入解锁序列 */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    return FLASH_OK;
}

/**
 * @brief  锁定Flash
 */
flash_status_t flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
    return FLASH_OK;
}

/**
 * @brief  等待Flash操作完成
 */
static flash_status_t flash_wait_ready(uint32_t timeout)
{
    while (timeout--) {
        if (!(FLASH->SR & FLASH_SR_BSY)) {
            return FLASH_OK;
        }
        /* 简单延时 */
        for (volatile int i = 0; i < 1000; i++);
    }
    return FLASH_TIMEOUT;
}

/**
 * @brief  获取扇区编号
 */
static uint32_t flash_get_sector_number(uint32_t addr)
{
    if (addr < 0x08004000) return 0;
    else if (addr < 0x08008000) return 1;
    else if (addr < 0x0800C000) return 2;
    else if (addr < 0x08010000) return 3;
    else if (addr < 0x08020000) return 4;
    else if (addr < 0x08040000) return 5;
    else if (addr < 0x08060000) return 6;
    else if (addr < 0x08080000) return 7;
    else if (addr < 0x080A0000) return 8;
    else if (addr < 0x080C0000) return 9;
    else if (addr < 0x080E0000) return 10;
    else return 11;
}

/**
 * @brief  获取扇区起始地址
 */
uint32_t flash_get_sector_start(uint32_t addr)
{
    if (addr < 0x08004000) return 0x08000000;
    else if (addr < 0x08008000) return 0x08004000;
    else if (addr < 0x0800C000) return 0x08008000;
    else if (addr < 0x08010000) return 0x0800C000;
    else if (addr < 0x08020000) return 0x08010000;
    else if (addr < 0x08040000) return 0x08020000;
    else if (addr < 0x08060000) return 0x08040000;
    else if (addr < 0x08080000) return 0x08060000;
    else if (addr < 0x080A0000) return 0x08080000;
    else if (addr < 0x080C0000) return 0x080A0000;
    else if (addr < 0x080E0000) return 0x080C0000;
    else return 0x080E0000;
}

/**
 * @brief  获取扇区大小
 */
uint32_t flash_get_sector_size(uint32_t addr)
{
    if (addr < 0x08010000) return 0x4000;      /* 16KB */
    else if (addr < 0x08020000) return 0x10000; /* 64KB */
    else return 0x20000;                         /* 128KB */
}

/**
 * @brief  擦除Flash扇区
 */
flash_status_t flash_erase_sector(uint32_t sector_addr)
{
    /* 检查地址 */
    if (!IS_VALID_FLASH_ADDR(sector_addr)) {
        return FLASH_ADDR_INVALID;
    }

    /* 保护Bootloader */
    if (IS_BOOTLOADER_ADDR(sector_addr)) {
        return FLASH_WRITE_PROTECTED;
    }

    uint32_t sector_num = flash_get_sector_number(sector_addr);

    flash_unlock();

    /* 等待就绪 */
    if (flash_wait_ready(FLASH_TIMEOUT_MS) != FLASH_OK) {
        flash_lock();
        return FLASH_TIMEOUT;
    }

    /* 配置并启动擦除 */
    FLASH->CR = (sector_num << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;

    /* 等待完成 */
    if (flash_wait_ready(FLASH_TIMEOUT_MS) != FLASH_OK) {
        flash_lock();
        return FLASH_TIMEOUT;
    }

    /* 清除标志 */
    FLASH->CR = 0;

    flash_lock();

    return FLASH_OK;
}

/**
 * @brief  擦除Flash区域
 */
flash_status_t flash_erase_region(uint32_t start_addr, uint32_t size)
{
    uint32_t addr = start_addr;
    uint32_t end_addr = start_addr + size;

    while (addr < end_addr) {
        uint32_t sector_start = flash_get_sector_start(addr);
        uint32_t sector_size = flash_get_sector_size(addr);

        flash_status_t status = flash_erase_sector(sector_start);
        if (status != FLASH_OK) {
            return status;
        }

        addr = sector_start + sector_size;
    }

    return FLASH_OK;
}

/**
 * @brief  写入Flash数据
 */
flash_status_t flash_write(uint32_t addr, const uint8_t *data, size_t len)
{
    /* 检查地址 */
    if (!IS_VALID_FLASH_ADDR(addr)) {
        return FLASH_ADDR_INVALID;
    }

    /* 保护Bootloader */
    if (IS_BOOTLOADER_ADDR(addr)) {
        return FLASH_WRITE_PROTECTED;
    }

    flash_unlock();

    /* 按字节编程 */
    for (size_t i = 0; i < len; i++) {
        /* 等待就绪 */
        if (flash_wait_ready(FLASH_TIMEOUT_MS) != FLASH_OK) {
            flash_lock();
            return FLASH_TIMEOUT;
        }

        /* 配置编程模式 */
        FLASH->CR = FLASH_CR_PG | (0 << FLASH_CR_PSIZE_Pos); /* 字节编程 */

        /* 写入数据 */
        *(volatile uint8_t*)(addr + i) = data[i];

        /* 等待完成 */
        if (flash_wait_ready(FLASH_TIMEOUT_MS) != FLASH_OK) {
            flash_lock();
            return FLASH_TIMEOUT;
        }
    }

    /* 清除标志 */
    FLASH->CR = 0;

    flash_lock();

    return FLASH_OK;
}

/**
 * @brief  从Flash读取数据
 */
flash_status_t flash_read(uint32_t addr, uint8_t *data, size_t len)
{
    if (!IS_VALID_FLASH_ADDR(addr)) {
        return FLASH_ADDR_INVALID;
    }

    memcpy(data, (const void*)addr, len);
    return FLASH_OK;
}

/**
 * @brief  校验Flash数据
 */
bool flash_verify(uint32_t addr, const uint8_t *data, size_t len)
{
    return (memcmp((const void*)addr, data, len) == 0);
}

/**
 * @brief  检查地址对齐
 */
bool flash_is_aligned(uint32_t addr, uint32_t alignment)
{
    return ((addr % alignment) == 0);
}