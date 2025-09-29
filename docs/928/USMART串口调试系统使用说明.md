# USMART串口调试系统使用说明

## 1. USMART系统概述

USMART（Universal Smart debugging system）是一个通过串口执行C函数的智能调试系统，可以在不重新编译代码的情况下，直接通过串口命令调用工程中的函数，极大提高了调试效率。

## 2. 系统特点

- **函数直接调用**: 通过串口直接执行C函数
- **参数支持**: 支持最多10个参数的函数调用
- **返回值显示**: 自动显示函数返回值
- **内存操作**: 支持内存地址读写
- **执行时间统计**: 可统计函数执行时间
- **智能解析**: 自动解析命令和参数

## 3. 硬件连接

### 3.1 串口连接
```
MCU端          USB转串口模块          电脑
TX  ---------> RX
RX  <--------- TX
GND ---------> GND
VCC ---------> 3.3V或5V (根据模块要求)
```

### 3.2 串口参数设置
- **波特率**: 115200 bps
- **数据位**: 8位
- **停止位**: 1位
- **校验位**: 无
- **流控制**: 无

## 4. 软件配置

### 4.1 包含头文件
在main.c中包含必要头文件：
```c
#include "usmart.h"
#include "usart.h"    // 串口驱动
```

### 4.2 系统初始化
在main函数中初始化USMART：
```c
int main(void)
{
    // 系统初始化
    HAL_Init();
    SystemClock_Config();

    // 串口初始化
    uart_init(115200);

    // USMART初始化 (参数为系统时钟MHz)
    usmart_init(168);  // 168MHz系统时钟

    while(1)
    {
        usmart_scan();  // 扫描串口命令
        // 其他应用代码...
    }
}
```

### 4.3 串口中断处理
在gd32f4xx_it.c中添加串口接收处理：
```c
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = USART_ReceiveData(USART1);
        // 将接收到的字符传递给USMART处理
        // 具体实现根据串口驱动而定
    }
}
```

## 5. 函数注册

### 5.1 注册函数到USMART
在`usmart_config.c`中注册要调试的函数：

```c
// 包含要调试的函数的头文件
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "24cxx.h"

// 函数注册表
struct _m_usmart_nametab usmart_nametab[] =
{
#if USMART_USE_WRFUNS==1 	// 内存读写函数
    (void*)read_addr,"u32 read_addr(u32 addr)",
    (void*)write_addr,"void write_addr(u32 addr,u32 val)",
#endif
    // 延时函数
    (void*)delay_ms,"void delay_ms(u16 nms)",
    (void*)delay_us,"void delay_us(u32 nus)",

    // LED控制函数
    (void*)LED_Init,"void LED_Init(void)",

    // 按键检测函数
    (void*)KEY_Scan,"u8 KEY_Scan(u8 mode)",

    // LCD显示函数
    (void*)LCD_ShowString,"void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,char *p)",

    // EEPROM读写函数
    (void*)AT24CXX_ReadOneByte,"u8 AT24CXX_ReadOneByte(u16 ReadAddr)",
    (void*)AT24CXX_WriteOneByte,"void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)",

    // 传感器函数 (根据项目需求添加)
    (void*)sensor_read_temp,"float sensor_read_temp(u8 channel)",
    (void*)actuator_set_valve,"void actuator_set_valve(u8 valve_id, u8 state)",
};
```

### 5.2 函数注册格式说明
```c
(void*)函数名, "返回类型 函数名(参数类型1 参数名1, 参数类型2 参数名2, ...)"
```

**注意事项：**
- 函数指针需要强制转换为(void*)
- 函数原型字符串必须与实际函数完全匹配
- 参数名可以简化，但类型必须正确

## 6. 串口调试命令

### 6.1 基本命令格式
```
函数名(参数1, 参数2, ...)
```

### 6.2 函数调用示例

#### 6.2.1 无参数函数调用
```bash
# 初始化LED
LED_Init()

# 扫描按键状态
KEY_Scan(0)
```

#### 6.2.2 带参数函数调用
```bash
# 延时1000毫秒
delay_ms(1000)

# 延时500微秒
delay_us(500)

# 读取EEPROM地址0的数据
AT24CXX_ReadOneByte(0)

# 向EEPROM地址10写入数据0x55
AT24CXX_WriteOneByte(10, 0x55)
```

#### 6.2.3 字符串参数函数调用
```bash
# 在LCD上显示字符串
LCD_ShowString(0, 0, 100, 16, 16, "Hello World")
```

### 6.3 内存操作命令

#### 6.3.1 读取内存
```bash
# 读取内存地址0x20000000的值
read_addr(0x20000000)

# 读取内存地址0x40021000的值 (GPIO端口地址)
read_addr(0x40021000)
```

#### 6.3.2 写入内存
```bash
# 向内存地址0x20000000写入值0x12345678
write_addr(0x20000000, 0x12345678)

# 向GPIO输出寄存器写入值
write_addr(0x40020014, 0x0002)
```

### 6.4 运行时统计命令

#### 6.4.1 开启运行时统计
```bash
# 开启函数执行时间统计
runtime 1
```

#### 6.4.2 关闭运行时统计
```bash
# 关闭函数执行时间统计
runtime 0
```

### 6.5 帮助命令
```bash
# 显示所有可用函数列表
help
```

## 7. 实际调试示例

### 7.1 墨路控制系统调试示例

假设已注册以下函数：
```c
// 传感器读取函数
(void*)sensor_read_temp,"float sensor_read_temp(u8 channel)",
(void*)sensor_read_pressure,"float sensor_read_pressure(u8 channel)",
(void*)sensor_read_level,"float sensor_read_level(u8 channel)",

// 执行器控制函数
(void*)actuator_set_valve,"void actuator_set_valve(u8 valve_id, u8 state)",
(void*)actuator_set_pump,"void actuator_set_pump(u8 pump_id, u8 speed)",
(void*)actuator_set_heater,"void actuator_set_heater(u8 heater_id, u8 state)",
```

#### 7.1.1 传感器测试
```bash
# 读取1号温度传感器
sensor_read_temp(1)
返回: 25.6

# 读取2号压力传感器
sensor_read_pressure(2)
返回: 101.3

# 读取3号液位传感器
sensor_read_level(3)
返回: 150.2
```

#### 7.1.2 执行器控制测试
```bash
# 打开1号阀门
actuator_set_valve(1, 1)

# 关闭1号阀门
actuator_set_valve(1, 0)

# 设置1号泵速度为80%
actuator_set_pump(1, 80)

# 打开2号加热器
actuator_set_heater(2, 1)
```

#### 7.1.3 组合测试序列
```bash
# 系统启动测试序列
LED_Init()                      # 初始化LED指示
delay_ms(100)                   # 延时100ms
sensor_read_temp(1)             # 读取温度
actuator_set_valve(1, 1)        # 开启主阀门
delay_ms(1000)                  # 延时1秒
actuator_set_pump(1, 50)        # 启动泵，50%速度
delay_ms(2000)                  # 运行2秒
sensor_read_pressure(1)         # 检查压力
actuator_set_pump(1, 0)         # 停止泵
actuator_set_valve(1, 0)        # 关闭阀门
```

## 8. 常用调试技巧

### 8.1 函数返回值检查
```bash
# 检查函数是否正确执行
result = KEY_Scan(0)
# 如果返回值不为0，说明有按键按下
```

### 8.2 循环测试
```bash
# 持续监控传感器数据（手动循环）
sensor_read_temp(1)
delay_ms(1000)
sensor_read_temp(1)
delay_ms(1000)
# 重复执行...
```

### 8.3 性能测试
```bash
# 开启运行时统计
runtime 1

# 测试函数执行时间
sensor_read_temp(1)
# 系统会显示: 函数执行时间: xxxx us

# 关闭运行时统计
runtime 0
```

## 9. 错误处理

### 9.1 常见错误信息

| 错误信息 | 说明 | 解决方法 |
|---------|------|----------|
| "函数错误" | 函数名不正确或未注册 | 检查函数名拼写，确认已在usmart_config.c中注册 |
| "参数错误" | 参数数量或类型不匹配 | 检查参数数量和类型是否与函数定义一致 |
| "参数溢出" | 参数超出限制 | 减少参数数量或长度 |
| "未找到匹配函数" | 函数未注册或格式错误 | 确认函数已正确注册到usmart_nametab |

### 9.2 调试无响应处理

1. **检查串口连接**: 确认硬件连接正确
2. **检查波特率**: 确认波特率设置为115200
3. **检查初始化**: 确认usmart_init()已调用
4. **检查扫描**: 确认主循环中调用了usmart_scan()

## 10. 配置参数说明

### 10.1 usmart.h中的配置参数

```c
#define MAX_FNAME_LEN 		30	// 函数名最大长度
#define MAX_PARM 			10	// 最大参数个数
#define PARM_LEN 			200	// 参数总长度限制

#define USMART_ENTIMX_SCAN 	1	// 使用定时器扫描
#define USMART_USE_HELP		1	// 使用帮助功能
#define USMART_USE_WRFUNS	1	// 使用内存读写功能
```

### 10.2 参数调整建议

- **MAX_FNAME_LEN**: 根据函数名长度适当调整
- **MAX_PARM**: 如需支持更多参数，可以增大此值
- **PARM_LEN**: 如参数较长（如字符串），需增大此值
- **USMART_USE_HELP**: 设为0可节省约700字节Flash

## 11. 高级应用

### 11.1 自定义调试宏
```c
// 在调试代码中添加便捷宏
#define DEBUG_TEMP(ch)    printf("Temp[%d]: %.2f°C\r\n", ch, sensor_read_temp(ch))
#define DEBUG_VALVE(id,s) printf("Valve[%d]: %s\r\n", id, s?"OPEN":"CLOSE")

// 注册到USMART
(void*)DEBUG_TEMP,"void DEBUG_TEMP(u8 ch)",
(void*)DEBUG_VALVE,"void DEBUG_VALVE(u8 id, u8 state)",
```

### 11.2 状态监控函数
```c
// 系统状态监控函数
void system_status_monitor(void)
{
    printf("=== System Status ===\r\n");
    printf("Temp1: %.2f°C\r\n", sensor_read_temp(1));
    printf("Temp2: %.2f°C\r\n", sensor_read_temp(2));
    printf("Pressure: %.2f kPa\r\n", sensor_read_pressure(1));
    printf("Level: %.2f mm\r\n", sensor_read_level(1));
    printf("==================\r\n");
}

// 注册到USMART
(void*)system_status_monitor,"void system_status_monitor(void)",
```

## 12. 注意事项

1. **函数安全性**: 注册的函数应该是线程安全的
2. **参数范围**: 注意参数的合法性检查
3. **内存管理**: 避免在USMART调用的函数中进行大量内存操作
4. **实时性**: USMART主要用于调试，不建议在实时性要求高的系统中频繁使用
5. **字符串参数**: 字符串参数需要用双引号包围
6. **浮点数**: 浮点数参数支持小数点格式

## 13. 总结

USMART是一个强大的串口调试工具，通过简单的串口命令即可：

- ✅ **直接调用C函数** - 无需重新编译
- ✅ **实时参数调整** - 动态测试不同参数
- ✅ **内存直接访问** - 读写任意内存地址
- ✅ **性能统计** - 函数执行时间测量
- ✅ **简单易用** - 命令格式直观明了

对于墨路控制系统的开发和调试，USMART可以大大提高开发效率，快速验证传感器读取、执行器控制等功能的正确性。

---

*文档版本: v1.0*
*创建时间: 2025-09-29*
*基于ink_supply_template_simple的USMART系统编写*