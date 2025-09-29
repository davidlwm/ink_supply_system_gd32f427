# USMART工作原理与完整使用流程详解

## 1. USMART工作原理

### 1.1 核心原理
USMART本质上是一个**命令行解析器**，它的工作流程是：

```
Windows串口工具 → 发送命令字符串 → 开发板串口接收 → USMART解析 → 调用函数 → 返回结果
```

### 1.2 详细工作流程
```
1. 开发板运行USMART程序
   ↓
2. Windows PC通过串口发送命令文本（如: "LED_Init()"）
   ↓
3. 开发板串口接收到字符串
   ↓
4. USMART解析字符串，识别函数名和参数
   ↓
5. USMART查找函数注册表，找到对应函数指针
   ↓
6. 调用真实的C函数
   ↓
7. 获取函数返回值，通过串口发送回PC
   ↓
8. Windows串口工具显示结果
```

### 1.3 关键技术
- **函数指针表**: USMART维护一个函数指针注册表
- **字符串解析**: 解析串口收到的命令字符串
- **动态调用**: 通过函数指针动态调用C函数
- **参数转换**: 将字符串参数转换为对应的C数据类型

## 2. 硬件连接

### 2.1 连接示意图
```
┌─────────────────┐    USB线    ┌─────────────────┐    杜邦线    ┌─────────────────┐
│   Windows PC    │ ────────── │  USB转串口模块   │ ────────── │   GD32开发板    │
│                 │            │                 │            │                 │
│  串口调试工具    │            │  CH340/CP2102   │            │     USART1      │
└─────────────────┘            └─────────────────┘            └─────────────────┘
```

### 2.2 具体连接
```
USB转串口模块        GD32开发板
VCC (3.3V/5V)  →   3.3V或5V
GND            →   GND
TXD            →   PA10 (USART1_RX)
RXD            →   PA9  (USART1_TX)
```

## 3. 软件环境搭建

### 3.1 Windows端软件
推荐的串口调试工具：
- **串口调试助手**（最常用）
- **PuTTY**（专业）
- **Tera Term**（轻量）
- **SecureCRT**（功能强大）

### 3.2 开发板端软件配置

#### 3.2.1 在main.c中初始化USMART
```c
#include "usmart.h"
#include "usart.h"

int main(void)
{
    // 系统初始化
    SystemInit();

    // 串口初始化 (115200波特率)
    uart_init(115200);

    // USMART初始化 (168MHz系统时钟)
    usmart_init(168);

    // 打印欢迎信息
    printf("USMART System Ready!\r\n");
    printf("Type 'help' for available functions\r\n");

    while(1)
    {
        // 这是关键：扫描串口数据并解析命令
        usmart_scan();

        // 其他应用代码...
        delay_ms(10);
    }
}
```

#### 3.2.2 串口接收处理（重要）
在usart.c中实现字符接收：
```c
// 串口接收缓冲区
#define USART_REC_LEN  200
u8 USART_RX_BUF[USART_REC_LEN];     // 接收缓冲
u16 USART_RX_STA = 0;               // 接收状态

// 串口中断服务函数
void USART1_IRQHandler(void)
{
    u8 Res;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        Res = USART_ReceiveData(USART1);    // 读取接收到的数据

        if((USART_RX_STA & 0x8000) == 0)   // 接收未完成
        {
            if(USART_RX_STA & 0x4000)       // 接收到了0x0d
            {
                if(Res != 0x0a)
                    USMART_RX_STA = 0;       // 接收错误,重新开始
                else
                    USART_RX_STA |= 0x8000;  // 接收完成了
            }
            else  // 还没收到0X0D
            {
                if(Res == 0x0d)
                    USART_RX_STA |= 0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
                    USART_RX_STA++;
                    if(USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0;    // 接收数据错误,重新开始接收
                }
            }
        }
    }
}
```

## 4. 函数注册过程详解

### 4.1 注册表结构理解
```c
// 函数注册表项结构
struct _m_usmart_nametab
{
    void* func;         // 函数指针（指向真实函数）
    const u8* name;     // 函数签名字符串（USMART用来解析）
};
```

### 4.2 注册示例详解
```c
// 在usmart_config.c中注册函数
struct _m_usmart_nametab usmart_nametab[] =
{
    // 注册LED初始化函数
    (void*)LED_Init,        // 函数指针
    "void LED_Init(void)",  // 函数签名（告诉USMART这个函数的调用格式）

    // 注册按键扫描函数
    (void*)KEY_Scan,        // 函数指针
    "u8 KEY_Scan(u8 mode)", // 函数签名（有参数和返回值）

    // 注册延时函数
    (void*)delay_ms,        // 函数指针
    "void delay_ms(u16 nms)", // 函数签名

    // 注册EEPROM读取函数
    (void*)AT24CXX_ReadOneByte,                    // 函数指针
    "u8 AT24CXX_ReadOneByte(u16 ReadAddr)",       // 函数签名

    // 注册自定义传感器函数
    (void*)read_temperature,                       // 函数指针
    "float read_temperature(u8 channel)",         // 函数签名
};
```

### 4.3 注册规则
1. **函数指针**：必须强制转换为`(void*)`
2. **函数签名**：必须与实际函数完全匹配
3. **参数类型**：使用USMART支持的类型（u8, u16, u32, float等）
4. **返回值**：支持基本数据类型和void

## 5. 完整使用流程演示

### 5.1 第一步：硬件连接
1. 将USB转串口模块连接到电脑
2. 用杜邦线连接模块和开发板
3. 给开发板上电

### 5.2 第二步：打开串口调试工具
以"串口调试助手"为例：

#### 5.2.1 配置串口参数
```
端口号：COM3 (根据实际情况选择)
波特率：115200
数据位：8
停止位：1
校验位：无
流控制：无
```

#### 5.2.2 打开串口连接
点击"打开串口"按钮

### 5.3 第三步：验证连接
在发送区输入：
```
help
```
按回车发送

**期望结果**：开发板返回可用函数列表
```
USMART V3.1
函数列表:
void LED_Init(void)
u8 KEY_Scan(u8 mode)
void delay_ms(u16 nms)
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
...
```

### 5.4 第四步：执行函数

#### 5.4.1 执行无参数函数
在串口工具发送区输入：
```
LED_Init()
```
按回车发送

**期望结果**：
```
LED_Init() executed
execution time: 120us
```

#### 5.4.2 执行有参数函数
在串口工具发送区输入：
```
delay_ms(1000)
```
按回车发送

**期望结果**：
```
delay_ms(1000) executed
execution time: 1000120us
```
（开发板会延时1秒，然后返回执行结果）

#### 5.4.3 执行有返回值的函数
在串口工具发送区输入：
```
KEY_Scan(0)
```
按回车发送

**期望结果**：
```
KEY_Scan(0) = 0
execution time: 45us
```
（返回值0表示没有按键按下）

#### 5.4.4 执行EEPROM读取
在串口工具发送区输入：
```
AT24CXX_ReadOneByte(0)
```
按回车发送

**期望结果**：
```
AT24CXX_ReadOneByte(0) = 255
execution time: 2300us
```
（返回EEPROM地址0处的数据值）

## 6. 实际调试场景演示

### 6.1 场景1：LED控制测试
```
步骤1: LED_Init()              // 初始化LED
返回: LED_Init() executed

步骤2: 看开发板LED状态          // 观察硬件变化
```

### 6.2 场景2：传感器数据读取
```
步骤1: read_temperature(1)     // 读取1号温度传感器
返回: read_temperature(1) = 25.6

步骤2: read_temperature(2)     // 读取2号温度传感器
返回: read_temperature(2) = 26.1
```

### 6.3 场景3：执行器控制
```
步骤1: set_valve_state(1, 1)   // 打开1号阀门
返回: set_valve_state(1, 1) executed

步骤2: delay_ms(2000)          // 延时2秒
返回: delay_ms(2000) executed

步骤3: set_valve_state(1, 0)   // 关闭1号阀门
返回: set_valve_state(1, 0) executed
```

### 6.4 场景4：内存直接操作
```
步骤1: read_addr(0x20000100)   // 读取RAM地址
返回: read_addr(0x20000100) = 0x00000000

步骤2: write_addr(0x20000100, 0x12345678)  // 写入数据
返回: write_addr(0x20000100, 0x12345678) executed

步骤3: read_addr(0x20000100)   // 再次读取验证
返回: read_addr(0x20000100) = 0x12345678
```

## 7. 调试工具界面示例

### 7.1 串口调试助手界面
```
┌─────────────────────────────────────────────────────────┐
│ 串口调试助手                                              │
├─────────────────────────────────────────────────────────┤
│ 串口设置：                                               │
│ 端口：COM3  波特率：115200  数据位：8                     │
│ 停止位：1   校验：无       流控：无      [打开串口]        │
├─────────────────────────────────────────────────────────┤
│ 接收区：                                                 │
│ USMART System Ready!                                    │
│ Type 'help' for available functions                     │
│ LED_Init() executed                                     │
│ execution time: 120us                                   │
│ KEY_Scan(0) = 0                                         │
│ execution time: 45us                                    │
├─────────────────────────────────────────────────────────┤
│ 发送区：                                                 │
│ LED_Init()                                    [发送]     │
└─────────────────────────────────────────────────────────┘
```

### 7.2 发送命令的具体操作
1. 在发送区输入命令：`LED_Init()`
2. 点击"发送"按钮或按回车键
3. 在接收区看到返回结果：`LED_Init() executed`

## 8. 常见问题排查

### 8.1 没有任何响应
**可能原因**：
- 串口连接问题
- 波特率不匹配
- 开发板程序未运行usmart_scan()

**排查步骤**：
1. 检查硬件连接
2. 确认串口参数（115200, 8N1）
3. 检查开发板是否正常运行
4. 在main函数while循环中确保调用了usmart_scan()

### 8.2 返回"函数错误"
**可能原因**：
- 函数名拼写错误
- 函数未在usmart_config.c中注册

**解决方法**：
1. 检查命令拼写
2. 发送"help"查看可用函数列表
3. 确认函数已正确注册

### 8.3 返回"参数错误"
**可能原因**：
- 参数数量不对
- 参数类型不匹配
- 缺少括号或逗号

**解决方法**：
1. 检查函数原型
2. 确认参数数量和类型
3. 检查语法格式

## 9. 进阶使用技巧

### 9.1 批量测试脚本
可以在串口工具中设置自动发送：
```
LED_Init()
delay_ms(500)
KEY_Scan(0)
delay_ms(500)
read_temperature(1)
delay_ms(500)
```

### 9.2 性能测试
```
runtime 1              // 开启时间统计
delay_ms(1000)         // 测试函数
runtime 0              // 关闭时间统计
```

### 9.3 系统状态监控
创建状态监控函数：
```c
void system_monitor(void)
{
    printf("Temp: %.2f°C\r\n", read_temperature(1));
    printf("Press: %.2f kPa\r\n", read_pressure(1));
    printf("Level: %.2f mm\r\n", read_level(1));
}
```

然后通过串口调用：
```
system_monitor()
```

## 10. 总结

USMART的核心就是：
1. **在开发板上运行一个命令解析程序**
2. **通过串口发送文本命令**
3. **开发板解析命令并调用对应的C函数**
4. **返回执行结果**

这样就实现了不需要重新编程，就能通过PC控制和测试开发板上的任何函数功能！

---

*文档版本: v2.0*
*更新时间: 2025-09-29*
*详细解释USMART工作原理和完整使用流程*