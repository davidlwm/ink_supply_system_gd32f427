# USMART串口调试实测指南

## 1. 现状分析

检查了现有代码，发现：

### ✅ 已经配置好的部分：
- 串口初始化：`uart_init(115200)` ✅
- USMART初始化：`usmart_dev.init(84)` ✅
- 串口中断接收：配置正确 ✅
- 基础函数注册：已注册6个函数 ✅

### ❌ 需要修改的关键问题：
- **主循环缺少`usmart_dev.scan()`调用** - 这是USMART工作的核心！

## 2. 必须修改的代码

### 2.1 修改main.c - 添加USMART扫描

**文件位置：** `USER/main.c`

**在主循环中添加usmart扫描：**

```c
// 找到main函数中的while(1)循环，在循环末尾添加：

/* 主循环 */
while (1)
{
    key = KEY_Scan(0);

    /* 按键处理 */
    if (key == WKUP_PRES)
    {
        // ... 现有代码保持不变
    }

    if (key == KEY0_PRES)
    {
        // ... 现有代码保持不变
    }

    /* 系统计数和状态显示 */
    system_counter++;

    /* LED闪烁指示系统运行 */
    led_counter++;
    if (led_counter >= 50)
    {
        LED0 = !LED0;
        led_counter = 0;

        if (system_counter % 500 == 0)
        {
            printf("System running... Counter: %lu\r\n", system_counter);
        }
    }

    // ========== 添加这一行 ==========
    usmart_dev.scan();    // USMART命令扫描 - 这是关键！
    // ================================

    delay_ms(10);
}
```

**完整的修改位置：**
在`delay_ms(10);`这行**之前**添加：
```c
usmart_dev.scan();    // USMART命令扫描
```

### 2.2 增加更多测试函数（可选）

**文件位置：** `USMART/usmart_config.c`

**在现有函数注册表中添加更多可测试的函数：**

```c
// 找到usmart_nametab[]数组，添加更多函数
struct _m_usmart_nametab usmart_nametab[]=
{
#if USMART_USE_WRFUNS==1
    (void*)read_addr,"u32 read_addr(u32 addr)",
    (void*)write_addr,"void write_addr(u32 addr,u32 val)",
#endif
    (void*)delay_ms,"void delay_ms(u16 nms)",
    (void*)delay_us,"void delay_us(u32 nus)",
    (void*)AT24CXX_ReadOneByte,"u8 AT24CXX_ReadOneByte(u16 ReadAddr)",
    (void*)AT24CXX_WriteOneByte,"void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)",

    // ========== 添加这些函数 (需要包含对应头文件) ==========
    (void*)LED_Init,"void LED_Init(void)",
    (void*)KEY_Scan,"u8 KEY_Scan(u8 mode)",
    // ====================================================
};
```

**注意：** 如果添加LED_Init和KEY_Scan，需要在usmart_config.c文件开头添加：
```c
#include "led.h"
#include "key.h"
```

## 3. 完整测试步骤

### 3.1 硬件连接

```
USB转串口模块     GD32开发板
VCC (3.3V)   →   3.3V
GND          →   GND
TXD          →   PA10 (USART1_RX)
RXD          →   PA9  (USART1_TX)
```

### 3.2 软件配置

#### 步骤1：下载串口调试工具
推荐使用：
- **串口调试助手** (搜索"串口调试助手"下载)
- **PuTTY** (专业用户)

#### 步骤2：配置串口参数
```
波特率：115200
数据位：8
停止位：1
校验位：无
流控制：无
```

#### 步骤3：编译烧录程序
1. 修改main.c添加`usmart_dev.scan();`
2. 编译工程
3. 烧录到开发板

### 3.3 测试验证

#### 步骤1：连接验证
1. 打开串口调试工具
2. 选择正确的COM口
3. 设置波特率115200
4. 点击"打开串口"

**期望看到：**
```
Ink Supply System Template Ready!
System Clock: 168MHz
Hardware: GD32F427VET6
Template: Based on IIC Experiment
System running... Counter: 500
```

#### 步骤2：USMART功能测试

**测试1：查看帮助**
在发送区输入：
```
help
```
点击发送

**期望返回：**
```
USMART V3.1
函数列表:
u32 read_addr(u32 addr)
void write_addr(u32 addr,u32 val)
void delay_ms(u16 nms)
void delay_us(u32 nus)
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
void LED_Init(void)
u8 KEY_Scan(u8 mode)
```

**测试2：延时函数**
在发送区输入：
```
delay_ms(1000)
```
点击发送

**期望现象：**
- 开发板暂停1秒（LED停止闪烁1秒）
- 串口返回：`delay_ms(1000) executed`

**测试3：按键检测**
在发送区输入：
```
KEY_Scan(0)
```
点击发送

**期望返回：**
- 没按键：`KEY_Scan(0) = 0`
- 按下WKUP：`KEY_Scan(0) = 2`
- 按下KEY0：`KEY_Scan(0) = 1`

**测试4：EEPROM读写**
```
# 读取EEPROM地址0
AT24CXX_ReadOneByte(0)
返回：AT24CXX_ReadOneByte(0) = 255

# 写入数据0x55到地址0
AT24CXX_WriteOneByte(0,85)
返回：AT24CXX_WriteOneByte(0,85) executed

# 再次读取验证
AT24CXX_ReadOneByte(0)
返回：AT24CXX_ReadOneByte(0) = 85
```

**测试5：内存操作**
```
# 读取内存地址
read_addr(0x20000000)
返回：read_addr(0x20000000) = 0x12345678

# 写入内存
write_addr(0x20000000,305419896)
返回：write_addr(0x20000000,305419896) executed

# 再次读取验证
read_addr(0x20000000)
返回：read_addr(0x20000000) = 305419896
```

## 4. 常见问题排查

### 4.1 问题：串口无响应

**可能原因：**
- 串口连接错误
- 波特率不匹配
- 忘记添加`usmart_dev.scan();`

**解决方法：**
1. 检查硬件连接（TX↔RX交叉连接）
2. 确认波特率115200
3. **确认已在main.c主循环中添加`usmart_dev.scan();`**

### 4.2 问题：返回"函数错误"

**可能原因：**
- 函数名拼写错误
- 函数未注册

**解决方法：**
1. 发送`help`查看可用函数
2. 检查拼写和大小写
3. 确认函数已在usmart_config.c中注册

### 4.3 问题：返回"参数错误"

**可能原因：**
- 参数数量不对
- 参数类型错误
- 语法错误

**解决方法：**
1. 检查参数数量
2. 确认括号和逗号
3. 参考help中的函数原型

## 5. 实测命令清单

复制粘贴以下命令进行快速测试：

```bash
# 基础测试
help
delay_ms(500)
KEY_Scan(0)

# EEPROM测试
AT24CXX_ReadOneByte(0)
AT24CXX_WriteOneByte(0,170)
AT24CXX_ReadOneByte(0)

# 内存测试
read_addr(0x20000100)
write_addr(0x20000100,12345678)
read_addr(0x20000100)

# LED测试（如果添加了LED_Init）
LED_Init()

# 性能测试
runtime 1
delay_ms(100)
runtime 0
```

## 6. 扩展测试

### 6.1 添加自定义测试函数

如果想测试更多功能，可以在代码中添加自定义函数：

```c
// 在main.c或其他文件中添加测试函数
void test_system_status(void)
{
    printf("=== System Status ===\r\n");
    printf("LED0 State: %d\r\n", LED0);
    printf("System Counter: %lu\r\n", system_counter);
    printf("==================\r\n");
}

// 在usmart_config.c中注册
(void*)test_system_status,"void test_system_status(void)",
```

### 6.2 测试LCD显示

```c
// 添加LCD测试函数
void test_lcd_display(void)
{
    LCD_ShowString(30, 150, 200, 16, 16, "USMART Test OK!");
}

// 注册到USMART
(void*)test_lcd_display,"void test_lcd_display(void)",
```

## 7. 总结

**关键修改：**
1. ✅ **必须在main.c主循环添加`usmart_dev.scan();`**
2. 🔧 可选添加更多测试函数

**测试流程：**
1. 硬件连接（串口线）
2. 打开串口工具（115200波特率）
3. 发送`help`查看函数列表
4. 逐个测试各种函数

通过这个测试，你可以验证USMART系统的完整功能，为后续的墨路控制系统开发打下基础！

---

*测试指南版本: v1.0*
*适用项目: ink_supply_template_simple*
*最后更新: 2025-09-29*