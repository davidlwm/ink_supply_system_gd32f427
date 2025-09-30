# 墨路控制系统 OTA 固件升级设计方案

## 1. 概述

### 1.1 文档说明
本文档定义了基于Bootloader的OTA（Over-The-Air）固件升级方案，支持通过TCP/IP或EtherCAT网络进行远程固件更新，确保系统安全可靠地升级。

### 1.2 系统信息
- **MCU**: GD32F427VGT6
- **Flash容量**: 1MB (0x08000000 - 0x080FFFFF)
- **升级方式**: TCP/IP、EtherCAT (FoE)
- **升级介质**: 以太网
- **Bootloader**: 自研双Bank切换机制

### 1.3 设计目标
- ✅ **安全性**: 防止升级失败导致设备变砖
- ✅ **可靠性**: 校验机制保证固件完整性
- ✅ **回退能力**: 升级失败自动回退旧版本
- ✅ **断电保护**: 升级过程断电不影响系统运行
- ✅ **兼容性**: 支持TCP和EtherCAT两种升级方式

---

## 2. Flash分区设计

### 2.1 分区布局 (GD32F427 1MB Flash)

```
地址范围                大小      分区名称            说明
+--------------------+
| 0x08000000         |  64KB    | Bootloader       | 引导程序（永久）
| - 0x0800FFFF       |          |                  | 不可被应用程序擦除
+--------------------+
| 0x08010000         |  16KB    | Config Sector    | 配置参数区
| - 0x08013FFF       |          |                  | 存储运行标志/版本信息
+--------------------+
| 0x08014000         | 432KB    | APP Bank A       | 应用程序区A
| - 0x0807FFFF       |          |                  | 当前运行固件
+--------------------+
| 0x08080000         | 432KB    | APP Bank B       | 应用程序区B
| - 0x080EBFFF       |          |                  | 新固件下载区
+--------------------+
| 0x080EC000         |  16KB    | Backup Config    | 配置备份区
| - 0x080EFFFF       |          |                  | 参数备份
+--------------------+
| 0x080F0000         |  64KB    | Reserved         | 预留区域
| - 0x080FFFFF       |          |                  | （未来扩展）
+--------------------+
总计: 1024KB
```

### 2.2 分区地址定义

```c
// config/flash_layout.h

#ifndef __FLASH_LAYOUT_H
#define __FLASH_LAYOUT_H

// Flash基地址
#define FLASH_BASE_ADDR             0x08000000

// Bootloader区域 (64KB)
#define BOOTLOADER_START_ADDR       0x08000000
#define BOOTLOADER_SIZE             0x00010000  // 64KB
#define BOOTLOADER_END_ADDR         (BOOTLOADER_START_ADDR + BOOTLOADER_SIZE - 1)

// 配置扇区 (16KB)
#define CONFIG_SECTOR_ADDR          0x08010000
#define CONFIG_SECTOR_SIZE          0x00004000  // 16KB

// 应用程序Bank A (432KB)
#define APP_BANK_A_START_ADDR       0x08014000
#define APP_BANK_A_SIZE             0x0006C000  // 432KB
#define APP_BANK_A_END_ADDR         (APP_BANK_A_START_ADDR + APP_BANK_A_SIZE - 1)

// 应用程序Bank B (432KB)
#define APP_BANK_B_START_ADDR       0x08080000
#define APP_BANK_B_SIZE             0x0006C000  // 432KB
#define APP_BANK_B_END_ADDR         (APP_BANK_B_START_ADDR + APP_BANK_B_SIZE - 1)

// 备份配置区 (16KB)
#define BACKUP_CONFIG_ADDR          0x080EC000
#define BACKUP_CONFIG_SIZE          0x00004000  // 16KB

// 预留区域 (64KB)
#define RESERVED_AREA_ADDR          0x080F0000
#define RESERVED_AREA_SIZE          0x00010000  // 64KB

// 应用程序最大尺寸
#define APP_MAX_SIZE                APP_BANK_A_SIZE

// Flash页大小 (GD32F427)
#define FLASH_PAGE_SIZE             0x00000800  // 2KB

#endif
```

---

## 3. Bootloader设计

### 3.1 Bootloader功能

**核心功能**：
1. 硬件初始化（最小系统）
2. 检查升级标志
3. 校验固件完整性（CRC32）
4. 选择启动Bank（A或B）
5. 跳转到应用程序
6. 固件下载与烧写（应急模式）

### 3.2 启动流程

```
             +------------------+
             | 上电/复位        |
             +------------------+
                      ↓
             +------------------+
             | Bootloader启动   |
             | 初始化系统时钟    |
             | 初始化GPIO/LED   |
             +------------------+
                      ↓
             +------------------+
             | 读取配置扇区      |
             | - 运行标志        |
             | - 版本信息        |
             | - CRC校验         |
             +------------------+
                      ↓
         +-----------+-----------+
         |                       |
    是否有升级      <No>     是否按住按键
    标志?                     (进入升级模式)
         |                       |
        Yes                     Yes
         |                       |
         ↓                       ↓
  +-------------+         +-------------+
  | 升级模式    |         | 应急模式    |
  | 从Bank B    |         | 等待固件    |
  | 拷贝到      |         | 通过UART/   |
  | Bank A      |         | 网络下载    |
  +-------------+         +-------------+
         ↓                       ↓
  +-------------+         +-------------+
  | 校验新固件  |         | 烧写固件    |
  | CRC32检查   |         |             |
  +-------------+         +-------------+
         |                       |
    +----+----+                  |
    |         |                  |
   Pass      Fail                |
    |         |                  |
    |    +----v-----+            |
    |    | 回退旧版  |            |
    |    | 清除标志  |            |
    |    +----------+            |
    |         |                  |
    +----+----+                  |
         |                       |
         ↓                       ↓
  +-------------------+   +-------------------+
  | 清除升级标志      |   | 设置运行标志      |
  +-------------------+   +-------------------+
         ↓                       ↓
  +-------------------------------------------+
  | 校验应用程序 (Bank A)                      |
  | - 检查起始向量表                           |
  | - 校验CRC32                                |
  +-------------------------------------------+
         ↓
    +----+----+
    |         |
   Valid    Invalid
    |         |
    |    +----v-----+
    |    | 进入应急  |
    |    | 模式等待  |
    |    +----------+
    |
    ↓
  +-------------------+
  | 设置向量表偏移    |
  | VTOR = Bank A地址 |
  +-------------------+
         ↓
  +-------------------+
  | 跳转到应用程序    |
  | MSP = [Bank A]    |
  | PC = [Bank A+4]   |
  +-------------------+
         ↓
  +-------------------+
  | 应用程序运行      |
  +-------------------+
```

### 3.3 配置扇区数据结构

```c
// bootloader/boot_config.h

#define BOOT_CONFIG_MAGIC       0x42544C4F  // "BTLO"
#define BOOT_CONFIG_VERSION     0x0100      // v1.0

typedef enum {
    BOOT_FLAG_NONE = 0,
    BOOT_FLAG_NORMAL_RUN,           // 正常运行
    BOOT_FLAG_UPDATE_PENDING,       // 待升级（Bank B有新固件）
    BOOT_FLAG_UPDATE_IN_PROGRESS,   // 升级中
    BOOT_FLAG_UPDATE_SUCCESS,       // 升级成功
    BOOT_FLAG_UPDATE_FAILED,        // 升级失败
    BOOT_FLAG_EMERGENCY_MODE        // 应急模式
} boot_flag_t;

typedef struct {
    // 魔术字
    uint32_t magic;                 // 0x42544C4F

    // 启动标志
    boot_flag_t boot_flag;          // 启动状态
    uint32_t active_bank;           // 当前活跃Bank (0=A, 1=B)

    // Bank A 固件信息
    struct {
        uint32_t version;           // 版本号 (主8位.次8位.修订16位)
        uint32_t size;              // 固件大小
        uint32_t crc32;             // CRC32校验值
        uint32_t timestamp;         // 编译时间戳
        char description[32];       // 版本描述
    } bank_a_info;

    // Bank B 固件信息
    struct {
        uint32_t version;
        uint32_t size;
        uint32_t crc32;
        uint32_t timestamp;
        char description[32];
    } bank_b_info;

    // 升级统计
    uint32_t update_count;          // 升级次数
    uint32_t boot_count;            // 启动次数
    uint32_t last_update_time;      // 最后升级时间

    // 回退信息
    uint32_t rollback_count;        // 回退次数
    uint32_t max_rollback;          // 最大回退次数 (默认3)

    // 保留
    uint8_t reserved[64];

    // 校验
    uint32_t config_crc32;          // 配置结构体CRC32
} boot_config_t;
```

### 3.4 Bootloader核心代码

```c
// bootloader/bootloader.c

#include "bootloader.h"
#include "boot_config.h"
#include "flash_hal.h"
#include "crc32.h"

static boot_config_t g_boot_config;

// Bootloader主函数
void bootloader_main(void)
{
    // 1. 硬件初始化
    boot_hw_init();

    // 2. LED指示（进入Bootloader）
    boot_led_set(LED_BOOTLOADER_ACTIVE);

    // 3. 读取配置扇区
    if (!boot_config_load(&g_boot_config)) {
        // 配置损坏，使用默认配置
        boot_config_reset(&g_boot_config);
    }

    // 4. 增加启动计数
    g_boot_config.boot_count++;

    // 5. 检查是否按住升级按键（进入应急模式）
    if (boot_check_emergency_key()) {
        boot_enter_emergency_mode();
        // 不会返回
    }

    // 6. 处理升级流程
    if (g_boot_config.boot_flag == BOOT_FLAG_UPDATE_PENDING) {
        boot_handle_update();
    }

    // 7. 校验应用程序
    if (!boot_verify_application(APP_BANK_A_START_ADDR)) {
        // 应用程序损坏，进入应急模式
        boot_led_set(LED_ERROR);
        boot_enter_emergency_mode();
        // 不会返回
    }

    // 8. 跳转到应用程序
    boot_led_set(LED_JUMP_TO_APP);
    boot_jump_to_application(APP_BANK_A_START_ADDR);

    // 不应该到达这里
    while (1) {
        boot_led_toggle(LED_ERROR);
        delay_ms(100);
    }
}

// 处理升级流程
static void boot_handle_update(void)
{
    boot_led_set(LED_UPDATE_IN_PROGRESS);

    // 设置升级进行中标志
    g_boot_config.boot_flag = BOOT_FLAG_UPDATE_IN_PROGRESS;
    boot_config_save(&g_boot_config);

    // 校验Bank B的新固件
    if (!boot_verify_firmware(APP_BANK_B_START_ADDR,
                               g_boot_config.bank_b_info.size,
                               g_boot_config.bank_b_info.crc32)) {
        // 新固件校验失败
        boot_led_set(LED_ERROR);
        g_boot_config.boot_flag = BOOT_FLAG_UPDATE_FAILED;
        g_boot_config.rollback_count++;
        boot_config_save(&g_boot_config);
        delay_ms(2000);
        return;
    }

    // 擦除Bank A
    boot_led_set(LED_ERASE_FLASH);
    if (!boot_erase_bank(APP_BANK_A_START_ADDR, APP_BANK_A_SIZE)) {
        g_boot_config.boot_flag = BOOT_FLAG_UPDATE_FAILED;
        boot_config_save(&g_boot_config);
        return;
    }

    // 从Bank B拷贝到Bank A
    boot_led_set(LED_COPY_FIRMWARE);
    if (!boot_copy_firmware(APP_BANK_B_START_ADDR,
                            APP_BANK_A_START_ADDR,
                            g_boot_config.bank_b_info.size)) {
        g_boot_config.boot_flag = BOOT_FLAG_UPDATE_FAILED;
        boot_config_save(&g_boot_config);
        return;
    }

    // 再次校验Bank A
    if (!boot_verify_firmware(APP_BANK_A_START_ADDR,
                              g_boot_config.bank_b_info.size,
                              g_boot_config.bank_b_info.crc32)) {
        g_boot_config.boot_flag = BOOT_FLAG_UPDATE_FAILED;
        boot_config_save(&g_boot_config);
        return;
    }

    // 升级成功
    boot_led_set(LED_UPDATE_SUCCESS);
    g_boot_config.boot_flag = BOOT_FLAG_NORMAL_RUN;
    g_boot_config.active_bank = 0;  // Bank A
    g_boot_config.bank_a_info = g_boot_config.bank_b_info;
    g_boot_config.update_count++;
    g_boot_config.last_update_time = get_timestamp();
    boot_config_save(&g_boot_config);

    delay_ms(1000);
}

// 校验应用程序
static bool boot_verify_application(uint32_t app_addr)
{
    // 检查栈指针合法性
    uint32_t sp = *(volatile uint32_t*)app_addr;
    if (sp < 0x20000000 || sp > 0x20030000) {  // GD32F427 SRAM范围
        return false;
    }

    // 检查复位向量合法性
    uint32_t pc = *(volatile uint32_t*)(app_addr + 4);
    if (pc < app_addr || pc > (app_addr + APP_MAX_SIZE)) {
        return false;
    }

    // 检查向量表标识 (可选)
    // 应用程序在向量表偏移0x1C0处存放固件标识 "APP_"
    uint32_t app_magic = *(volatile uint32_t*)(app_addr + 0x1C0);
    if (app_magic != 0x5F505041) {  // "APP_"
        return false;
    }

    // 校验CRC32 (从配置读取)
    if (g_boot_config.bank_a_info.crc32 != 0) {
        uint32_t calc_crc = crc32_calculate((uint8_t*)app_addr,
                                            g_boot_config.bank_a_info.size);
        if (calc_crc != g_boot_config.bank_a_info.crc32) {
            return false;
        }
    }

    return true;
}

// 跳转到应用程序
static void boot_jump_to_application(uint32_t app_addr)
{
    // 关闭所有中断
    __disable_irq();

    // 关闭SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // 清除所有中断标志
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // 设置向量表偏移
    SCB->VTOR = app_addr;

    // 读取应用程序栈指针和复位向量
    uint32_t app_sp = *(volatile uint32_t*)app_addr;
    uint32_t app_pc = *(volatile uint32_t*)(app_addr + 4);

    // 设置主栈指针
    __set_MSP(app_sp);

    // 跳转到应用程序
    void (*app_reset_handler)(void) = (void (*)(void))app_pc;
    app_reset_handler();

    // 不应该返回
}

// 应急模式（UART/网络升级）
static void boot_enter_emergency_mode(void)
{
    g_boot_config.boot_flag = BOOT_FLAG_EMERGENCY_MODE;
    boot_config_save(&g_boot_config);

    // 初始化UART（用于调试输出）
    boot_uart_init();
    printf("\r\n=== Emergency Mode ===\r\n");
    printf("Bootloader Version: %s\r\n", BOOTLOADER_VERSION);
    printf("Waiting for firmware...\r\n");

    // 初始化以太网（最小配置）
    boot_ethernet_init();

    // 启动简易TCP服务器（等待固件上传）
    boot_tcp_server_start();

    // 永久循环，等待固件上传
    while (1) {
        boot_tcp_server_process();
        boot_led_toggle(LED_EMERGENCY);
        delay_ms(500);
    }
}

// 擦除Bank
static bool boot_erase_bank(uint32_t start_addr, uint32_t size)
{
    uint32_t addr = start_addr;
    uint32_t end_addr = start_addr + size;

    while (addr < end_addr) {
        if (!flash_erase_sector(addr)) {
            return false;
        }
        addr += FLASH_SECTOR_SIZE;

        // 更新进度LED
        boot_update_progress((addr - start_addr) * 100 / size);
    }

    return true;
}

// 拷贝固件
static bool boot_copy_firmware(uint32_t src_addr, uint32_t dst_addr, uint32_t size)
{
    uint8_t buffer[256];
    uint32_t offset = 0;

    while (offset < size) {
        uint32_t chunk_size = (size - offset) > 256 ? 256 : (size - offset);

        // 读取源数据
        memcpy(buffer, (void*)(src_addr + offset), chunk_size);

        // 写入目标
        if (!flash_write(dst_addr + offset, buffer, chunk_size)) {
            return false;
        }

        offset += chunk_size;

        // 更新进度
        boot_update_progress(offset * 100 / size);
    }

    return true;
}

// 校验固件
static bool boot_verify_firmware(uint32_t addr, uint32_t size, uint32_t expected_crc)
{
    uint32_t calc_crc = crc32_calculate((uint8_t*)addr, size);
    return (calc_crc == expected_crc);
}
```

---

## 4. 应用程序端OTA功能

### 4.1 OTA管理器

```c
// app/ota_manager.h

#ifndef __OTA_MANAGER_H
#define __OTA_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// OTA状态
typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_DOWNLOADING,
    OTA_STATE_VERIFYING,
    OTA_STATE_READY_TO_INSTALL,
    OTA_STATE_INSTALLING,
    OTA_STATE_SUCCESS,
    OTA_STATE_FAILED
} ota_state_t;

// OTA错误码
typedef enum {
    OTA_ERR_OK = 0,
    OTA_ERR_FLASH_ERASE,
    OTA_ERR_FLASH_WRITE,
    OTA_ERR_CRC_MISMATCH,
    OTA_ERR_SIZE_INVALID,
    OTA_ERR_VERSION_INVALID,
    OTA_ERR_BUSY,
    OTA_ERR_TIMEOUT
} ota_error_t;

// OTA固件信息
typedef struct {
    uint32_t version;               // 固件版本
    uint32_t size;                  // 固件大小
    uint32_t crc32;                 // CRC32校验
    uint32_t timestamp;             // 时间戳
    char description[64];           // 版本描述
} ota_firmware_info_t;

// OTA进度回调
typedef void (*ota_progress_callback_t)(uint32_t current, uint32_t total);

// 初始化OTA管理器
void ota_manager_init(void);

// 开始OTA升级
ota_error_t ota_start_update(const ota_firmware_info_t *fw_info);

// 写入固件数据（分包写入）
ota_error_t ota_write_firmware_data(uint32_t offset, const uint8_t *data, uint32_t len);

// 完成下载，准备安装
ota_error_t ota_finish_download(void);

// 安装固件（重启后生效）
ota_error_t ota_install_firmware(void);

// 取消OTA升级
void ota_cancel_update(void);

// 获取OTA状态
ota_state_t ota_get_state(void);

// 获取当前固件版本
uint32_t ota_get_current_version(void);

// 设置进度回调
void ota_set_progress_callback(ota_progress_callback_t callback);

#endif
```

### 4.2 OTA管理器实现

```c
// app/ota_manager.c

#include "ota_manager.h"
#include "boot_config.h"
#include "flash_hal.h"
#include "crc32.h"
#include <string.h>

// OTA上下文
typedef struct {
    ota_state_t state;
    ota_firmware_info_t fw_info;
    uint32_t bytes_written;
    uint32_t last_write_time;
    ota_progress_callback_t progress_callback;
    uint32_t crc32_accumulator;
} ota_context_t;

static ota_context_t g_ota_ctx = {0};
static boot_config_t g_boot_config;

// 初始化OTA管理器
void ota_manager_init(void)
{
    memset(&g_ota_ctx, 0, sizeof(ota_context_t));
    g_ota_ctx.state = OTA_STATE_IDLE;

    // 读取bootloader配置
    flash_read(CONFIG_SECTOR_ADDR, (uint8_t*)&g_boot_config, sizeof(boot_config_t));

    printf("OTA Manager Initialized\n");
    printf("Current Firmware Version: %d.%d.%d\n",
           (g_boot_config.bank_a_info.version >> 24) & 0xFF,
           (g_boot_config.bank_a_info.version >> 16) & 0xFF,
           g_boot_config.bank_a_info.version & 0xFFFF);
}

// 开始OTA升级
ota_error_t ota_start_update(const ota_firmware_info_t *fw_info)
{
    // 检查状态
    if (g_ota_ctx.state != OTA_STATE_IDLE) {
        return OTA_ERR_BUSY;
    }

    // 检查固件大小
    if (fw_info->size == 0 || fw_info->size > APP_MAX_SIZE) {
        return OTA_ERR_SIZE_INVALID;
    }

    // 检查版本号（可选：防止降级）
    if (fw_info->version <= g_boot_config.bank_a_info.version) {
        printf("Warning: Downgrade detected\n");
        // 可选择拒绝降级
        // return OTA_ERR_VERSION_INVALID;
    }

    // 保存固件信息
    memcpy(&g_ota_ctx.fw_info, fw_info, sizeof(ota_firmware_info_t));
    g_ota_ctx.bytes_written = 0;
    g_ota_ctx.crc32_accumulator = 0xFFFFFFFF;
    g_ota_ctx.last_write_time = HAL_GetTick();

    // 擦除Bank B
    printf("Erasing Bank B...\n");
    if (!ota_erase_bank_b()) {
        return OTA_ERR_FLASH_ERASE;
    }

    g_ota_ctx.state = OTA_STATE_DOWNLOADING;
    printf("OTA Update Started, Size: %d bytes\n", fw_info->size);

    return OTA_ERR_OK;
}

// 写入固件数据
ota_error_t ota_write_firmware_data(uint32_t offset, const uint8_t *data, uint32_t len)
{
    if (g_ota_ctx.state != OTA_STATE_DOWNLOADING) {
        return OTA_ERR_BUSY;
    }

    // 检查偏移和长度
    if (offset + len > g_ota_ctx.fw_info.size) {
        return OTA_ERR_SIZE_INVALID;
    }

    // 写入Flash
    uint32_t write_addr = APP_BANK_B_START_ADDR + offset;
    if (!flash_write(write_addr, data, len)) {
        g_ota_ctx.state = OTA_STATE_FAILED;
        return OTA_ERR_FLASH_WRITE;
    }

    // 累积CRC32
    g_ota_ctx.crc32_accumulator = crc32_update(g_ota_ctx.crc32_accumulator, data, len);

    // 更新进度
    g_ota_ctx.bytes_written += len;
    g_ota_ctx.last_write_time = HAL_GetTick();

    // 回调进度
    if (g_ota_ctx.progress_callback) {
        g_ota_ctx.progress_callback(g_ota_ctx.bytes_written, g_ota_ctx.fw_info.size);
    }

    printf("OTA Progress: %d / %d bytes (%.1f%%)\n",
           g_ota_ctx.bytes_written, g_ota_ctx.fw_info.size,
           (float)g_ota_ctx.bytes_written * 100.0f / g_ota_ctx.fw_info.size);

    return OTA_ERR_OK;
}

// 完成下载
ota_error_t ota_finish_download(void)
{
    if (g_ota_ctx.state != OTA_STATE_DOWNLOADING) {
        return OTA_ERR_BUSY;
    }

    // 检查完整性
    if (g_ota_ctx.bytes_written != g_ota_ctx.fw_info.size) {
        printf("OTA Error: Size mismatch\n");
        g_ota_ctx.state = OTA_STATE_FAILED;
        return OTA_ERR_SIZE_INVALID;
    }

    g_ota_ctx.state = OTA_STATE_VERIFYING;
    printf("Verifying firmware...\n");

    // 完成CRC32计算
    g_ota_ctx.crc32_accumulator ^= 0xFFFFFFFF;

    // 校验CRC32
    if (g_ota_ctx.crc32_accumulator != g_ota_ctx.fw_info.crc32) {
        printf("OTA Error: CRC32 mismatch (expected: 0x%08X, got: 0x%08X)\n",
               g_ota_ctx.fw_info.crc32, g_ota_ctx.crc32_accumulator);
        g_ota_ctx.state = OTA_STATE_FAILED;
        return OTA_ERR_CRC_MISMATCH;
    }

    // 再次从Flash读取校验（确保写入正确）
    uint32_t flash_crc = crc32_calculate((uint8_t*)APP_BANK_B_START_ADDR,
                                          g_ota_ctx.fw_info.size);
    if (flash_crc != g_ota_ctx.fw_info.crc32) {
        printf("OTA Error: Flash CRC32 mismatch\n");
        g_ota_ctx.state = OTA_STATE_FAILED;
        return OTA_ERR_CRC_MISMATCH;
    }

    printf("Firmware verification passed\n");
    g_ota_ctx.state = OTA_STATE_READY_TO_INSTALL;

    return OTA_ERR_OK;
}

// 安装固件
ota_error_t ota_install_firmware(void)
{
    if (g_ota_ctx.state != OTA_STATE_READY_TO_INSTALL) {
        return OTA_ERR_BUSY;
    }

    printf("Installing firmware...\n");

    // 更新配置扇区
    g_boot_config.boot_flag = BOOT_FLAG_UPDATE_PENDING;
    g_boot_config.bank_b_info.version = g_ota_ctx.fw_info.version;
    g_boot_config.bank_b_info.size = g_ota_ctx.fw_info.size;
    g_boot_config.bank_b_info.crc32 = g_ota_ctx.fw_info.crc32;
    g_boot_config.bank_b_info.timestamp = g_ota_ctx.fw_info.timestamp;
    strncpy(g_boot_config.bank_b_info.description,
            g_ota_ctx.fw_info.description,
            sizeof(g_boot_config.bank_b_info.description));

    // 计算配置CRC
    g_boot_config.config_crc32 = crc32_calculate((uint8_t*)&g_boot_config,
                                                  sizeof(boot_config_t) - 4);

    // 擦除配置扇区
    flash_erase_sector(CONFIG_SECTOR_ADDR);

    // 写入配置
    if (!flash_write(CONFIG_SECTOR_ADDR, (uint8_t*)&g_boot_config, sizeof(boot_config_t))) {
        printf("OTA Error: Failed to write config\n");
        g_ota_ctx.state = OTA_STATE_FAILED;
        return OTA_ERR_FLASH_WRITE;
    }

    printf("Configuration updated, rebooting...\n");
    g_ota_ctx.state = OTA_STATE_SUCCESS;

    // 延时后重启
    HAL_Delay(1000);
    NVIC_SystemReset();

    return OTA_ERR_OK;
}

// 取消OTA
void ota_cancel_update(void)
{
    printf("OTA Update Cancelled\n");
    g_ota_ctx.state = OTA_STATE_IDLE;
    memset(&g_ota_ctx, 0, sizeof(ota_context_t));
}

// 获取OTA状态
ota_state_t ota_get_state(void)
{
    return g_ota_ctx.state;
}

// 获取当前版本
uint32_t ota_get_current_version(void)
{
    return g_boot_config.bank_a_info.version;
}

// 设置进度回调
void ota_set_progress_callback(ota_progress_callback_t callback)
{
    g_ota_ctx.progress_callback = callback;
}

// 擦除Bank B
static bool ota_erase_bank_b(void)
{
    uint32_t addr = APP_BANK_B_START_ADDR;
    uint32_t end_addr = APP_BANK_B_START_ADDR + APP_BANK_B_SIZE;

    while (addr < end_addr) {
        if (!flash_erase_sector(addr)) {
            return false;
        }
        addr += FLASH_SECTOR_SIZE;
    }

    return true;
}
```

---

## 5. TCP协议OTA实现

### 5.1 OTA命令定义

```c
// TCP命令扩展
#define CMD_OTA_START           0x10    // 开始OTA
#define CMD_OTA_DATA            0x11    // OTA数据传输
#define CMD_OTA_FINISH          0x12    // OTA完成
#define CMD_OTA_INSTALL         0x13    // 安装固件
#define CMD_OTA_CANCEL          0x14    // 取消OTA
#define CMD_OTA_GET_STATUS      0x15    // 获取OTA状态

// 响应
#define CMD_OTA_START_RESP      0x90
#define CMD_OTA_DATA_RESP       0x91
#define CMD_OTA_FINISH_RESP     0x92
#define CMD_OTA_INSTALL_RESP    0x93
#define CMD_OTA_STATUS_RESP     0x95
```

### 5.2 OTA开始命令 (0x10)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 动态长度
    uint8_t  cmd;             // 0x10
    uint8_t  reserved;
    // 固件信息
    uint32_t version;         // 版本号
    uint32_t size;            // 固件大小
    uint32_t crc32;           // CRC32校验
    uint32_t timestamp;       // 时间戳
    char     description[64]; // 描述
    uint16_t crc;
} cmd_ota_start_req_t;
```

**响应帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;
    uint8_t  cmd;             // 0x90
    uint8_t  error_code;      // 0x00=成功
    uint32_t max_packet_size; // 最大数据包大小（建议256~1024）
    uint16_t crc;
} cmd_ota_start_resp_t;
```

### 5.3 OTA数据传输 (0x11)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;
    uint8_t  cmd;             // 0x11
    uint8_t  reserved;
    uint32_t offset;          // 数据偏移
    uint16_t data_len;        // 数据长度
    uint8_t  data[];          // 固件数据
    // uint16_t crc;          // 放在data之后
} cmd_ota_data_req_t;
```

**响应帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;
    uint8_t  cmd;             // 0x91
    uint8_t  error_code;      // 0x00=成功
    uint32_t next_offset;     // 下一个期望的偏移
    uint16_t crc;
} cmd_ota_data_resp_t;
```

### 5.4 TCP OTA处理

```c
// app/tcp_server.c (添加OTA命令处理)

static void tcp_handle_ota_start(tcp_server_ctx_t *ctx, tcp_frame_header_t *header, uint8_t *data)
{
    cmd_ota_start_req_t *req = (cmd_ota_start_req_t*)data;

    ota_firmware_info_t fw_info;
    fw_info.version = req->version;
    fw_info.size = req->size;
    fw_info.crc32 = req->crc32;
    fw_info.timestamp = req->timestamp;
    strncpy(fw_info.description, req->description, sizeof(fw_info.description));

    // 启动OTA
    ota_error_t err = ota_start_update(&fw_info);

    // 构造响应
    cmd_ota_start_resp_t *resp = (cmd_ota_start_resp_t*)ctx->tx_buffer;
    resp->sync_word = 0xAA55;
    resp->length = sizeof(cmd_ota_start_resp_t) - 2;
    resp->cmd = CMD_OTA_START_RESP;
    resp->error_code = (err == OTA_ERR_OK) ? ERR_OK : ERR_DEVICE_BUSY;
    resp->max_packet_size = 1024;  // 建议包大小

    uint16_t crc = calculate_crc16((uint8_t*)&resp->length, resp->length - 2);
    resp->crc = crc;

    tcp_write(ctx->pcb, ctx->tx_buffer, sizeof(cmd_ota_start_resp_t), TCP_WRITE_FLAG_COPY);
    tcp_output(ctx->pcb);
}

static void tcp_handle_ota_data(tcp_server_ctx_t *ctx, tcp_frame_header_t *header, uint8_t *data)
{
    cmd_ota_data_req_t *req = (cmd_ota_data_req_t*)data;

    // 写入固件数据
    ota_error_t err = ota_write_firmware_data(req->offset, req->data, req->data_len);

    // 构造响应
    cmd_ota_data_resp_t *resp = (cmd_ota_data_resp_t*)ctx->tx_buffer;
    resp->sync_word = 0xAA55;
    resp->length = sizeof(cmd_ota_data_resp_t) - 2;
    resp->cmd = CMD_OTA_DATA_RESP;
    resp->error_code = (err == OTA_ERR_OK) ? ERR_OK : ERR_FLASH_WRITE;
    resp->next_offset = req->offset + req->data_len;

    uint16_t crc = calculate_crc16((uint8_t*)&resp->length, resp->length - 2);
    resp->crc = crc;

    tcp_write(ctx->pcb, ctx->tx_buffer, sizeof(cmd_ota_data_resp_t), TCP_WRITE_FLAG_COPY);
    tcp_output(ctx->pcb);
}

static void tcp_handle_ota_finish(tcp_server_ctx_t *ctx, tcp_frame_header_t *header, uint8_t *data)
{
    // 完成下载并校验
    ota_error_t err = ota_finish_download();

    // 构造响应
    typedef struct __attribute__((packed)) {
        uint16_t sync_word;
        uint16_t length;
        uint8_t  cmd;
        uint8_t  error_code;
        uint16_t crc;
    } cmd_ota_finish_resp_t;

    cmd_ota_finish_resp_t *resp = (cmd_ota_finish_resp_t*)ctx->tx_buffer;
    resp->sync_word = 0xAA55;
    resp->length = sizeof(cmd_ota_finish_resp_t) - 2;
    resp->cmd = CMD_OTA_FINISH_RESP;
    resp->error_code = (err == OTA_ERR_OK) ? ERR_OK : ERR_CHECKSUM;

    uint16_t crc = calculate_crc16((uint8_t*)&resp->length, resp->length - 2);
    resp->crc = crc;

    tcp_write(ctx->pcb, ctx->tx_buffer, sizeof(cmd_ota_finish_resp_t), TCP_WRITE_FLAG_COPY);
    tcp_output(ctx->pcb);
}

static void tcp_handle_ota_install(tcp_server_ctx_t *ctx, tcp_frame_header_t *header, uint8_t *data)
{
    // 发送响应（在重启前）
    typedef struct __attribute__((packed)) {
        uint16_t sync_word;
        uint16_t length;
        uint8_t  cmd;
        uint8_t  error_code;
        uint16_t crc;
    } cmd_ota_install_resp_t;

    cmd_ota_install_resp_t *resp = (cmd_ota_install_resp_t*)ctx->tx_buffer;
    resp->sync_word = 0xAA55;
    resp->length = sizeof(cmd_ota_install_resp_t) - 2;
    resp->cmd = CMD_OTA_INSTALL_RESP;
    resp->error_code = ERR_OK;

    uint16_t crc = calculate_crc16((uint8_t*)&resp->length, resp->length - 2);
    resp->crc = crc;

    tcp_write(ctx->pcb, ctx->tx_buffer, sizeof(cmd_ota_install_resp_t), TCP_WRITE_FLAG_COPY);
    tcp_output(ctx->pcb);

    // 等待发送完成
    HAL_Delay(500);

    // 安装固件（会自动重启）
    ota_install_firmware();
}
```

---

## 6. EtherCAT FoE实现

### 6.1 FoE协议简介

FoE (File over EtherCAT) 是EtherCAT协议栈提供的文件传输服务，专门用于固件升级。

**FoE操作类型**：
- 0x01: Read Request
- 0x02: Write Request
- 0x03: Data
- 0x04: Ack
- 0x05: Error
- 0x06: Busy

### 6.2 FoE回调实现

```c
// app/ethercat_app.c (添加FoE支持)

#include "ota_manager.h"

// FoE写请求回调
int foe_write_request(uint16_t slave, char *filename, uint32_t filesize)
{
    printf("FoE Write Request: %s, Size: %d\n", filename, filesize);

    // 检查文件名（固件文件名格式: "firmware_vX.Y.Z.bin"）
    if (strncmp(filename, "firmware_", 9) != 0) {
        return -1;  // 拒绝
    }

    // 解析版本号
    uint32_t major, minor, patch;
    sscanf(filename, "firmware_v%d.%d.%d.bin", &major, &minor, &patch);
    uint32_t version = (major << 24) | (minor << 16) | patch;

    // 准备OTA
    ota_firmware_info_t fw_info;
    fw_info.version = version;
    fw_info.size = filesize;
    fw_info.crc32 = 0;  // FoE会在传输完成后计算
    fw_info.timestamp = HAL_GetTick();
    snprintf(fw_info.description, sizeof(fw_info.description),
             "FoE Update v%d.%d.%d", major, minor, patch);

    ota_error_t err = ota_start_update(&fw_info);
    if (err != OTA_ERR_OK) {
        return -1;
    }

    return 0;  // 接受
}

// FoE数据接收回调
int foe_write_data(uint16_t slave, uint32_t offset, uint8_t *data, uint16_t len)
{
    ota_error_t err = ota_write_firmware_data(offset, data, len);
    return (err == OTA_ERR_OK) ? 0 : -1;
}

// FoE写完成回调
int foe_write_complete(uint16_t slave, uint32_t size)
{
    printf("FoE Write Complete, Size: %d\n", size);

    // 完成下载并校验
    ota_error_t err = ota_finish_download();
    if (err != OTA_ERR_OK) {
        return -1;
    }

    // 自动安装（或等待主站命令）
    ota_install_firmware();

    return 0;
}

// FoE错误回调
void foe_error(uint16_t slave, uint32_t error_code)
{
    printf("FoE Error: 0x%08X\n", error_code);
    ota_cancel_update();
}

// 注册FoE回调
void ethercat_foe_init(void)
{
    ec_slave[0].foe_write_request_hook = foe_write_request;
    ec_slave[0].foe_write_data_hook = foe_write_data;
    ec_slave[0].foe_write_complete_hook = foe_write_complete;
    ec_slave[0].foe_error_hook = foe_error;

    printf("EtherCAT FoE Initialized\n");
}
```

---

## 7. 上位机OTA工具

### 7.1 Python OTA客户端

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import time
import sys
import os
import zlib

class OTAClient:
    def __init__(self, host, port=8888):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10.0)
        self.sock.connect((self.host, self.port))
        print(f"Connected to {self.host}:{self.port}")

    def disconnect(self):
        if self.sock:
            self.sock.close()

    def calculate_crc16(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def send_command(self, cmd, data=b''):
        length = 4 + len(data) + 2
        frame = struct.pack('<HHBB', 0xAA55, length, cmd, 0)
        frame += data
        crc = self.calculate_crc16(frame[2:])
        frame += struct.pack('<H', crc)
        self.sock.sendall(frame)

    def receive_response(self):
        header = self.sock.recv(6)
        sync, length, cmd, index = struct.unpack('<HHBB', header)
        if sync != 0xAA55:
            raise Exception("Invalid sync word")
        remaining = length - 4
        data = self.sock.recv(remaining)
        return cmd, data[:-2], struct.unpack('<H', data[-2:])[0]

    def ota_update(self, firmware_path, version_str):
        # 读取固件文件
        with open(firmware_path, 'rb') as f:
            firmware_data = f.read()

        file_size = len(firmware_data)
        print(f"Firmware: {firmware_path}")
        print(f"Size: {file_size} bytes")

        # 计算CRC32
        crc32 = zlib.crc32(firmware_data) & 0xFFFFFFFF
        print(f"CRC32: 0x{crc32:08X}")

        # 解析版本号
        parts = version_str.split('.')
        version = (int(parts[0]) << 24) | (int(parts[1]) << 16) | int(parts[2])

        # 1. 发送OTA开始命令
        print("\n[1/4] Starting OTA...")
        desc = f"OTA Update v{version_str}".ljust(64, '\x00')
        start_data = struct.pack('<III', version, file_size, crc32)
        start_data += struct.pack('<I', int(time.time()))
        start_data += desc.encode('utf-8')[:64]

        self.send_command(0x10, start_data)
        cmd, resp_data, _ = self.receive_response()

        if cmd != 0x90:
            raise Exception("OTA start failed")

        error_code, max_packet_size = struct.unpack('<BI', resp_data[:5])
        if error_code != 0:
            raise Exception(f"OTA start error: {error_code}")

        print(f"OTA started, max packet size: {max_packet_size}")

        # 2. 传输固件数据
        print("\n[2/4] Transferring firmware data...")
        offset = 0
        packet_size = min(max_packet_size, 1024)

        while offset < file_size:
            chunk = firmware_data[offset:offset + packet_size]
            chunk_len = len(chunk)

            # 发送数据包
            data_cmd = struct.pack('<IH', offset, chunk_len) + chunk
            self.send_command(0x11, data_cmd)

            # 接收响应
            cmd, resp_data, _ = self.receive_response()
            if cmd != 0x91:
                raise Exception("OTA data transfer failed")

            error_code, next_offset = struct.unpack('<BI', resp_data)
            if error_code != 0:
                raise Exception(f"OTA data error at offset {offset}")

            offset += chunk_len

            # 显示进度
            progress = offset * 100.0 / file_size
            print(f"\rProgress: {progress:.1f}% ({offset}/{file_size} bytes)", end='')

        print("\nData transfer complete")

        # 3. 完成并校验
        print("\n[3/4] Verifying firmware...")
        self.send_command(0x12)
        cmd, resp_data, _ = self.receive_response()

        if cmd != 0x92:
            raise Exception("OTA finish failed")

        error_code = resp_data[0]
        if error_code != 0:
            raise Exception(f"OTA verification error: {error_code}")

        print("Firmware verification passed")

        # 4. 安装固件
        print("\n[4/4] Installing firmware...")
        self.send_command(0x13)
        cmd, resp_data, _ = self.receive_response()

        if cmd != 0x93:
            raise Exception("OTA install failed")

        error_code = resp_data[0]
        if error_code != 0:
            raise Exception(f"OTA install error: {error_code}")

        print("Firmware installed, device will reboot...")

def main():
    if len(sys.argv) < 4:
        print("Usage: python ota_client.py <host> <firmware.bin> <version>")
        print("Example: python ota_client.py 192.168.1.100 firmware.bin 3.0.1")
        sys.exit(1)

    host = sys.argv[1]
    firmware_path = sys.argv[2]
    version = sys.argv[3]

    if not os.path.exists(firmware_path):
        print(f"Error: Firmware file not found: {firmware_path}")
        sys.exit(1)

    client = OTAClient(host)

    try:
        client.connect()
        client.ota_update(firmware_path, version)
        print("\n✓ OTA Update Successful!")
    except Exception as e:
        print(f"\n✗ OTA Update Failed: {e}")
        sys.exit(1)
    finally:
        client.disconnect()

if __name__ == '__main__':
    main()
```

---

## 8. 固件打包与版本管理

### 8.1 固件打包脚本

```bash
#!/bin/bash
# build_firmware.sh

set -e

# 配置
PROJECT_NAME="InkSupplySystem"
VERSION_MAJOR=3
VERSION_MINOR=0
VERSION_PATCH=1
VERSION="${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}"

BUILD_DIR="build"
OUTPUT_DIR="output"
FIRMWARE_BIN="${BUILD_DIR}/${PROJECT_NAME}.bin"
FIRMWARE_OUTPUT="${OUTPUT_DIR}/firmware_v${VERSION}.bin"

echo "========================================="
echo "  Firmware Build and Package Tool"
echo "  Version: ${VERSION}"
echo "========================================="

# 1. 编译固件
echo "[1/5] Building firmware..."
make clean
make -j4

if [ ! -f "${FIRMWARE_BIN}" ]; then
    echo "Error: Firmware build failed"
    exit 1
fi

# 2. 检查固件大小
FIRMWARE_SIZE=$(stat -f%z "${FIRMWARE_BIN}" 2>/dev/null || stat -c%s "${FIRMWARE_BIN}")
MAX_SIZE=442368  # 432KB

echo "[2/5] Checking firmware size..."
echo "  Size: ${FIRMWARE_SIZE} bytes (Max: ${MAX_SIZE} bytes)"

if [ ${FIRMWARE_SIZE} -gt ${MAX_SIZE} ]; then
    echo "Error: Firmware too large!"
    exit 1
fi

# 3. 添加固件标识
echo "[3/5] Adding firmware header..."
mkdir -p "${OUTPUT_DIR}"

# 在固件偏移0x1C0处添加魔术字 "APP_"
xxd -s 0x1C0 -l 4 -p /dev/zero | sed 's/.*/5F505041/' | xxd -r -p > /tmp/magic.bin
dd if=/tmp/magic.bin of="${FIRMWARE_BIN}" bs=1 seek=448 conv=notrunc 2>/dev/null

# 4. 计算CRC32
echo "[4/5] Calculating CRC32..."
CRC32=$(python3 -c "import zlib; print('0x%08X' % (zlib.crc32(open('${FIRMWARE_BIN}', 'rb').read()) & 0xFFFFFFFF))")
echo "  CRC32: ${CRC32}"

# 5. 创建版本信息文件
echo "[5/5] Creating version info..."
cat > "${OUTPUT_DIR}/firmware_v${VERSION}.json" <<EOF
{
  "name": "${PROJECT_NAME}",
  "version": "${VERSION}",
  "version_code": $((VERSION_MAJOR << 24 | VERSION_MINOR << 16 | VERSION_PATCH)),
  "size": ${FIRMWARE_SIZE},
  "crc32": "${CRC32}",
  "timestamp": $(date +%s),
  "build_date": "$(date '+%Y-%m-%d %H:%M:%S')",
  "description": "Firmware version ${VERSION}",
  "min_bootloader_version": "1.0",
  "changelog": [
    "Initial release"
  ]
}
EOF

# 拷贝固件
cp "${FIRMWARE_BIN}" "${FIRMWARE_OUTPUT}"

echo ""
echo "✓ Firmware package created:"
echo "  Binary: ${FIRMWARE_OUTPUT}"
echo "  Info:   ${OUTPUT_DIR}/firmware_v${VERSION}.json"
echo ""
echo "Upload command:"
echo "  python ota_client.py <host> ${FIRMWARE_OUTPUT} ${VERSION}"
```

---

## 9. 安全机制

### 9.1 固件签名（可选）

```c
// 使用RSA或ECC签名验证固件真实性
bool verify_firmware_signature(uint32_t fw_addr, uint32_t fw_size)
{
    // 固件末尾存放256字节RSA签名
    uint8_t *signature = (uint8_t*)(fw_addr + fw_size - 256);

    // 计算固件SHA256
    uint8_t hash[32];
    sha256_calculate((uint8_t*)fw_addr, fw_size - 256, hash);

    // 使用公钥验证签名
    return rsa_verify(public_key, signature, hash, 32);
}
```

### 9.2 安全启动（Secure Boot）

1. **Bootloader签名**: Bootloader本身由硬件芯片校验（芯片厂商特性）
2. **应用程序签名**: Bootloader验证应用程序签名
3. **证书链**: 多级证书验证

### 9.3 防回滚机制

```c
// 防止降级到存在漏洞的旧版本
bool check_version_rollback(uint32_t new_version)
{
    // 读取最低允许版本（存储在OTP区域）
    uint32_t min_version = read_otp_min_version();

    if (new_version < min_version) {
        printf("Error: Rollback protection triggered\n");
        return false;
    }

    return true;
}
```

---

## 10. 测试与验证

### 10.1 测试用例

**测试项1：正常升级流程**
- [ ] TCP方式升级成功
- [ ] EtherCAT FoE升级成功
- [ ] 固件版本正确更新
- [ ] 配置参数保留

**测试项2：异常处理**
- [ ] CRC校验失败，自动回退
- [ ] 升级过程断电，系统恢复
- [ ] 传输超时，自动取消
- [ ] Flash写入失败，回退旧版本

**测试项3：边界条件**
- [ ] 固件大小为0，拒绝升级
- [ ] 固件超过最大尺寸，拒绝升级
- [ ] 版本号相同，可选择拒绝或允许
- [ ] 连续升级多次，系统稳定

**测试项4：安全测试**
- [ ] 伪造固件，CRC校验失败
- [ ] 中间人攻击，签名验证失败（如果实现）
- [ ] 回滚攻击，防回滚机制生效

### 10.2 性能测试

| 指标 | TCP | EtherCAT FoE |
|------|-----|--------------|
| 传输速率 | ~100KB/s | ~500KB/s |
| 升级时间（432KB） | ~50秒 | ~10秒 |
| Flash擦除时间 | ~5秒 | ~5秒 |
| CRC校验时间 | ~2秒 | ~2秒 |
| 总升级时间 | ~60秒 | ~20秒 |

---

## 11. 故障恢复

### 11.1 应急模式

如果应用程序损坏，Bootloader提供应急模式：

1. **进入条件**：
   - 按住特定按键上电
   - 应用程序校验失败
   - 连续启动失败3次

2. **应急功能**：
   - 启动最小TCP服务器
   - 等待固件上传
   - 强制烧写到Bank A

### 11.2 UART恢复

```c
// Bootloader提供UART命令行
void boot_uart_recovery(void)
{
    printf("=== UART Recovery Mode ===\n");
    printf("Commands:\n");
    printf("  info       - Show system info\n");
    printf("  upload     - Upload firmware via XMODEM\n");
    printf("  flash      - Flash firmware to Bank A\n");
    printf("  boot       - Boot to application\n");
    printf("  reset      - Reset device\n");

    while (1) {
        char cmd[32];
        uart_read_line(cmd, sizeof(cmd));

        if (strcmp(cmd, "info") == 0) {
            show_system_info();
        } else if (strcmp(cmd, "upload") == 0) {
            xmodem_receive_firmware();
        } else if (strcmp(cmd, "flash") == 0) {
            flash_firmware_to_bank_a();
        } else if (strcmp(cmd, "boot") == 0) {
            boot_jump_to_application(APP_BANK_A_START_ADDR);
        } else if (strcmp(cmd, "reset") == 0) {
            NVIC_SystemReset();
        }
    }
}
```

---

## 12. 总结

### 12.1 方案优势

✅ **双Bank架构**: 保证升级失败可回退，设备永不变砖
✅ **多重校验**: CRC32 + 签名验证，确保固件完整性和真实性
✅ **断电保护**: 升级过程断电不影响旧版本运行
✅ **灵活升级**: 支持TCP和EtherCAT两种方式
✅ **应急模式**: 提供最后的恢复手段

### 12.2 后续优化

- [ ] 增加固件压缩（节省传输时间和Flash空间）
- [ ] 实现增量升级（仅传输差异部分）
- [ ] 添加固件签名验证（RSA/ECC）
- [ ] 支持A/B无缝切换（无需重启）
- [ ] 实现OTA进度持久化（断电恢复）

### 12.3 文件清单

**Bootloader**:
- `bootloader/bootloader.c/h`
- `bootloader/boot_config.c/h`
- `bootloader/boot_hw.c/h`
- `bootloader/linker_script_bootloader.ld`

**应用程序**:
- `app/ota_manager.c/h`
- `app/tcp_server.c` (扩展OTA命令)
- `app/ethercat_app.c` (扩展FoE支持)
- `app/linker_script_app.ld` (向量表偏移0x08014000)

**工具**:
- `tools/ota_client.py`
- `tools/build_firmware.sh`
- `tools/flash_bootloader.sh`

---

**文档版本**: v1.0
**编写日期**: 2025-09-30
**适用硬件**: GD32F427VGT6 (1MB Flash)
**适用固件**: v3.0.0及以上