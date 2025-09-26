# 供墨系统控制板卡 GD32F427 项目 (8周v4版本)
# Ink Supply System Control Board based on GD32F427VGT6 - 8-Week v4

## 项目概述

这是一个基于GD32F427VGT6微控制器的工业级供墨系统控制板卡项目，采用FreeRTOS实时操作系统，支持EtherCAT和TCP/IP双协议通信。本项目严格按照8周v4开发标准构建。

### 主要特性

- **MCU**: GD32F427VGT6 (ARM Cortex-M4F, 200MHz)
- **操作系统**: FreeRTOS v10.4.6
- **通信协议**: EtherCAT + TCP/IP (lwIP)
- **开发标准**: 8周v4架构标准
- **传感器支持**:
  - FRD-8061 液位传感器 (4-20mA)
  - HP10MY 压力传感器 (4-20mA)
  - FTT518 PT100温度传感器 (三线制)
- **执行器支持**:
  - MRA-23D3 固态继电器加热器
  - MPB025BBB 调速泵
  - 电磁阀控制
  - LED指示灯
- **HMI**: CH12832B-12 LCD显示屏 + 5路LED指示

## 目录结构 (8周v4标准)

```
ink_supply_system_gd32f427/
├── src/                    # 源代码
│   ├── main.c             # 主程序入口
│   ├── app/               # 应用层
│   │   ├── sensor/        # 传感器模块
│   │   ├── actuator/      # 执行器模块
│   │   ├── control/       # 控制算法
│   │   ├── communication/ # 通信模块
│   │   ├── hmi/          # 人机界面
│   │   ├── safety/       # 安全保护
│   │   └── config/       # 配置管理
│   ├── middleware/        # 中间件层
│   │   ├── filter.c/h    # 数字滤波 (滑动平均+卡尔曼)
│   │   ├── pid.c/h       # PID算法 (简化版)
│   │   └── tasks.c/h     # FreeRTOS任务管理
│   ├── hal/              # 硬件抽象层 (基于GD32 HAL)
│   │   ├── adc_hal.c/h   # ADC处理 (传感器采集)
│   │   ├── pwm_hal.c/h   # PWM处理 (执行器控制)
│   │   ├── gpio_hal.c/h  # GPIO处理 (数字IO)
│   │   ├── uart_hal.c/h  # UART处理 (调试+通信)
│   │   ├── spi_hal.c/h   # SPI处理 (LCD显示)
│   │   ├── eth_hal.c/h   # 以太网处理 (网络通信) [待开发]
│   │   └── flash_hal.c/h # Flash处理 (配置存储) [待开发]
│   └── drivers/          # 设备驱动层 (现成库)
├── include/              # 头文件
│   ├── app/             # 应用层头文件
│   ├── middleware/      # 中间件头文件
│   ├── hal/            # HAL层头文件
│   └── drivers/        # 驱动层头文件
├── config/             # 配置文件
│   ├── FreeRTOSConfig.h
│   ├── lwipopts.h
│   ├── ethercat_config.h
│   └── error_config.h
├── lib/                # 第三方库
│   ├── GD32F4xx_standard_peripheral/
│   ├── FreeRTOS/
│   ├── lwip/
│   └── EtherCAT_Stack/
├── build/              # 构建输出
├── Makefile           # Make构建脚本
├── CMakeLists.txt     # CMake构建脚本
├── build.sh           # 自动构建脚本
└── .gitignore         # Git配置
```

## 软件架构

### 四层架构设计

1. **应用层**: 业务逻辑实现，包括传感器处理、执行器控制、通信协议等
2. **中间件层**: RTOS服务、控制算法、数据处理等
3. **HAL层**: 硬件抽象层，屏蔽底层硬件差异
4. **BSP层**: 板级支持包，芯片和外设驱动

### 任务架构

- **Safety Task** (优先级8): 安全监控和紧急保护
- **EtherCAT Task** (优先级8): EtherCAT实时通信 (1ms周期)
- **Control Task** (优先级6): PID控制算法 (10ms周期)
- **Sensor Task** (优先级5): 传感器数据采集 (10ms周期)
- **Actuator Task** (优先级4): 执行器控制 (50ms周期)
- **Comm Task** (优先级3): TCP/IP通信管理 (50ms周期)
- **HMI Task** (优先级2): 人机界面更新 (200ms周期)
- **Config Task** (优先级1): 配置管理 (1000ms周期)

## 编译和构建

### 依赖工具

- ARM GCC工具链 (arm-none-eabi-gcc)
- Make构建工具
- OpenOCD (用于程序烧录和调试)

### 编译命令

```bash
# 调试版本
make DEBUG=1

# 发布版本
make

# 清理
make clean

# 烧录程序
make flash

# 查看程序大小
make size

# 栈使用分析
make stack-usage

# GDB调试
make debug
```

### 构建配置

- **调试版本**: 包含调试信息，优化级别-O0
- **发布版本**: 优化级别-Os，去除调试信息

## 硬件配置

### 引脚映射

| 功能 | 引脚 | 说明 |
|------|------|------|
| 液位传感器1 | PA0 | ADC通道0 (4-20mA) |
| 液位传感器2 | PA1 | ADC通道1 (4-20mA) |
| 压力传感器1 | PA2 | ADC通道2 (4-20mA) |
| 压力传感器2 | PA3 | ADC通道3 (4-20mA) |
| 温度传感器1信号 | PA4 | ADC通道4 (PT100) |
| 温度传感器1参考 | PA5 | ADC通道5 (PT100) |
| 加热器1控制 | PB0 | PWM输出 (10kHz) |
| 加热器2控制 | PB1 | PWM输出 (10kHz) |
| 泵1调速 | PC6 | PWM输出 (1kHz) |
| 泵2调速 | PC7 | PWM输出 (1kHz) |
| LED1-LED5 | PD12-PD15,PE0 | GPIO输出 |
| LCD SPI | PA5-PA7 | SPI1接口 |
| 以太网 | PA1,PA2,PA7,PB11-PB13,PC1,PC4,PC5 | RMII接口 |

### 系统时钟

- 系统主频: 200MHz
- AHB时钟: 200MHz
- APB1时钟: 50MHz
- APB2时钟: 100MHz

## 通信协议

### EtherCAT协议

- **循环时间**: 1ms
- **过程数据**: 输入16字节，输出12字节
- **实时性**: 硬件时间戳，抖动<50μs

### TCP/IP协议

- **端口**: 502 (Modbus TCP)
- **最大连接**: 8个客户端
- **数据格式**: JSON格式命令和响应

## 安全特性

- **多级保护**: 硬件保护 + 软件保护 + 通信保护
- **故障检测**: 传感器开路/短路检测
- **紧急停机**: 1ms内响应紧急停机命令
- **看门狗**: 硬件和软件双重看门狗保护
- **参数校验**: 所有输入参数范围检查

## 开发和调试

### 调试接口

- **SWD接口**: PA13(SWDIO), PA14(SWCLK)
- **串口调试**: USART0 (PA9/PA10), 波特率115200

### 日志系统

- **级别**: ERROR, WARN, INFO, DEBUG
- **输出**: 串口 + 网络 + Flash存储
- **格式**: 时间戳 + 模块 + 级别 + 消息

### 性能监控

- **CPU使用率**: 实时统计各任务CPU占用
- **内存使用**: 堆栈使用情况监控
- **通信性能**: 网络带宽和延迟统计
- **实时性**: 任务执行时间和周期抖动

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 联系方式

- 项目维护者: 开发团队
- 技术支持: 请通过GitHub Issues报告问题

---

**注意**: 本项目为工业级控制系统，请确保在合适的环境中使用，并遵循相关安全标准。