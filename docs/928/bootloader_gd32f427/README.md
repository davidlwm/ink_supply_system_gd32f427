# GD32F427 Bootloader 工程说明

## 1. 项目概述

这是一个基于GD32F427VGT6的Bootloader工程，支持OTA固件升级功能。

### 特性
- ✅ 双Bank架构（Bank A + Bank B）
- ✅ 固件完整性校验（CRC32）
- ✅ 升级失败自动回退
- ✅ 断电保护
- ✅ 应急恢复模式
- ✅ 防止Bootloader区域被擦除

## 2. Flash分区布局

```
地址范围                大小      分区名称            说明
+--------------------+
| 0x08000000         |  64KB    | Bootloader       | 引导程序（本工程）
| - 0x0800FFFF       |          |                  |
+--------------------+
| 0x08010000         |  16KB    | Config Sector    | 配置参数区
| - 0x08013FFF       |          |                  |
+--------------------+
| 0x08014000         | 432KB    | APP Bank A       | 应用程序区A
| - 0x0807FFFF       |          |                  |
+--------------------+
| 0x08080000         | 432KB    | APP Bank B       | 应用程序区B
| - 0x080EBFFF       |          |                  |
+--------------------+
| 0x080EC000         |  16KB    | Backup Config    | 配置备份区
| - 0x080EFFFF       |          |                  |
+--------------------+
| 0x080F0000         |  64KB    | Reserved         | 预留区域
| - 0x080FFFFF       |          |                  |
+--------------------+
总计: 1024KB
```

## 3. 目录结构

```
bootloader_gd32f427/
├── USER/                   # 用户主程序
│   └── main.c             # 主函数入口
├── CORE/                   # Cortex-M4核心文件
│   ├── core_cm4.h
│   └── startup_gd32f4xx.s
├── HALLIB/                 # GD32F4xx HAL库
│   ├── inc/
│   └── src/
├── HARDWARE/               # 硬件驱动
│   ├── LED/
│   └── KEY/
├── SYSTEM/                 # 系统相关
│   ├── delay/
│   └── usart/
├── bootloader/             # Bootloader核心代码
│   ├── bootloader.c       # 主程序
│   ├── bootloader.h
│   ├── boot_config.c      # 配置管理
│   └── boot_config.h
├── config/                 # 配置文件
│   ├── flash_layout.h     # Flash分区定义
│   ├── boot_config.h      # 启动配置
│   └── gd32f427_bootloader.ld  # 链接脚本
└── utils/                  # 工具函数
    ├── crc32.c            # CRC32计算
    ├── crc32.h
    ├── flash_hal.c        # Flash HAL抽象层
    └── flash_hal.h
```

## 4. 编译说明

### 4.1 使用Keil MDK编译

1. **创建新工程**：
   - 打开Keil MDK
   - 选择 `Project -> New µVision Project`
   - 选择芯片：`GigaDevice -> GD32F4xx -> GD32F427VE`

2. **添加源文件**：
   ```
   bootloader/
   ├── bootloader.c
   └── boot_config.c

   utils/
   ├── crc32.c
   └── flash_hal.c

   USER/
   └── main.c

   HALLIB/src/
   ├── gd32f4xx_fmc.c         # Flash控制
   ├── gd32f4xx_rcu.c         # 时钟控制
   ├── gd32f4xx_gpio.c        # GPIO
   └── gd32f4xx_usart.c       # USART（可选）

   CORE/
   └── startup_gd32f4xx.s     # 启动文件

   SYSTEM/
   ├── delay/delay.c
   └── usart/usart.c          # 调试串口（可选）
   ```

3. **配置头文件路径**：
   - 右键工程 -> `Options for Target`
   - `C/C++ -> Include Paths`，添加：
     ```
     ./config
     ./bootloader
     ./utils
     ./USER
     ./HALLIB/inc
     ./CORE
     ./SYSTEM/delay
     ./SYSTEM/usart
     ```

4. **配置链接脚本**：
   - `Linker -> Use Memory Layout from Target Dialog` 取消勾选
   - `Scatter File`: 选择 `config/gd32f427_bootloader.ld`
   - 或者在 `Target -> IROM1` 设置：
     - Start: `0x08000000`
     - Size: `0x10000` (64KB)

5. **编译选项**：
   - `C/C++ -> Define`: 添加 `GD32F427, USE_STDPERIPH_DRIVER`
   - `C/C++ -> Optimization`: Level 2 (-O2)
   - 可选：添加 `BOOTLOADER_DEBUG` 启用调试输出

6. **编译**：
   - `Project -> Build Target` (F7)

### 4.2 生成固件文件

编译成功后会生成：
- `*.axf` - ELF格式可执行文件
- `*.hex` - Intel HEX格式
- `*.bin` - 二进制格式

使用HEX或BIN文件烧录到Flash起始地址 `0x08000000`。

## 5. 烧录说明

### 5.1 首次烧录Bootloader

使用J-Link/ST-Link等调试器将Bootloader烧录到 `0x08000000`：

**使用J-Link Commander**：
```bash
J-Link> connect
J-Link> loadbin bootloader.bin 0x08000000
J-Link> verifybin bootloader.bin 0x08000000
J-Link> reset
J-Link> go
```

**使用JFlash**：
1. 创建新工程，选择GD32F427VE芯片
2. 打开 `bootloader.hex` 或 `bootloader.bin`
3. 确认地址为 `0x08000000`
4. 点击 `Target -> Program & Verify`

### 5.2 应用程序配置

应用程序需要做以下修改：

1. **修改链接脚本**（应用程序）：
   ```c
   MEMORY
   {
       FLASH (rx) : ORIGIN = 0x08014000, LENGTH = 432K
       RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 192K
   }
   ```

2. **设置向量表偏移**（应用程序启动代码）：
   ```c
   // 在SystemInit()或main()开始处添加
   SCB->VTOR = 0x08014000;  // APP Bank A起始地址
   ```

3. **添加固件标识**（可选，用于Bootloader识别）：
   在应用程序链接脚本中添加：
   ```
   .firmware_magic 0x080141C0 :
   {
       LONG(0x5F505041);  /* "APP_" */
   } >FLASH
   ```

## 6. 使用说明

### 6.1 正常启动流程

1. 上电复位
2. Bootloader启动，初始化硬件
3. 读取配置扇区
4. 校验Bank A应用程序
5. 跳转到应用程序（地址 `0x08014000`）

### 6.2 OTA升级流程

在应用程序中调用OTA管理器：

1. 接收新固件，写入Bank B (`0x08080000`)
2. 校验固件完整性（CRC32）
3. 更新配置扇区，设置升级标志
4. 系统重启
5. Bootloader检测到升级标志
6. 擦除Bank A
7. 从Bank B拷贝固件到Bank A
8. 校验Bank A
9. 清除升级标志
10. 跳转到新固件

### 6.3 应急模式

**进入条件**：
- 按住应急按键上电
- 应用程序损坏
- 连续启动失败3次

**应急模式功能**：
- LED慢闪（0.5秒周期）
- 等待通过UART/TCP上传固件
- 强制烧写到Bank A

### 6.4 配置扇区结构

配置扇区 (`0x08010000`) 存储的信息：
- 启动标志（正常/升级/应急）
- Bank A固件信息（版本、大小、CRC32）
- Bank B固件信息
- 启动次数、升级次数
- 回退次数、失败次数

## 7. 调试说明

### 7.1 启用调试输出

在工程中定义宏 `BOOTLOADER_DEBUG`：
- Keil MDK: `Options -> C/C++ -> Define` 添加 `BOOTLOADER_DEBUG`
- 或在代码中添加：`#define BOOTLOADER_DEBUG`

调试信息通过UART0输出（波特率115200）：
```
========================================
 GD32F427 Bootloader v1.0.0
========================================
Boot Count: 1
Boot Flag:  1

=== Firmware Update Start ===
Verifying new firmware...
Erasing Bank A...
Copying firmware...
Verifying Bank A...
Firmware update SUCCESS!

Jumping to application @ 0x08014000...
```

### 7.2 使用SEGGER RTT

可选使用SEGGER RTT进行无串口调试：
1. 添加RTT源文件
2. 替换`printf`为`SEGGER_RTT_printf`
3. 使用J-Link RTT Viewer查看输出

## 8. 常见问题

### 8.1 应用程序无法启动

**现象**：Bootloader LED一直闪烁错误指示

**排查**：
1. 检查应用程序链接脚本是否正确（起始地址0x08014000）
2. 检查向量表偏移是否设置（SCB->VTOR = 0x08014000）
3. 使用调试器查看Bank A的栈指针和复位向量
4. 检查应用程序是否添加固件标识

### 8.2 OTA升级失败

**现象**：升级后设备无法启动或回退到旧版本

**排查**：
1. 确认新固件CRC32计算正确
2. 确认固件大小不超过432KB
3. 检查配置扇区是否正确更新
4. 查看Bootloader调试输出

### 8.3 配置扇区损坏

**现象**：Bootloader使用默认配置启动

**解决**：
- Bootloader会自动重建配置扇区
- 如果备份扇区有效，会自动恢复

## 9. 安全建议

1. **写保护Bootloader扇区**：防止误擦除
   ```c
   // 在Bootloader中添加Flash写保护
   fmc_ob_unlock();
   ob_write_protection_enable(FMC_WP_SECTOR0 | FMC_WP_SECTOR1 |
                               FMC_WP_SECTOR2 | FMC_WP_SECTOR3);
   fmc_ob_lock();
   ```

2. **固件签名验证**：防止非法固件
   - 使用RSA/ECC对固件签名
   - Bootloader验证签名后才升级

3. **回滚保护**：防止降级攻击
   - 记录最小允许版本
   - 拒绝低于最小版本的固件

## 10. 相关文档

- [GD32F427数据手册](https://www.gigadevice.com/)
- [OTA固件升级设计方案.md](../OTA固件升级设计方案.md)
- [Flash分区布局](./config/flash_layout.h)

## 11. 版本历史

- **v1.0.0** (2025-09-30)
  - 初始版本
  - 支持双Bank升级
  - CRC32校验
  - 应急模式

---

**开发者**: Claude Code
**日期**: 2025-09-30
**芯片**: GD32F427VGT6 (1MB Flash)