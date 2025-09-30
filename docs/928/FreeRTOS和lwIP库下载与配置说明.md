# FreeRTOS和lwIP库下载与配置说明

## 1. 下载信息

### 1.1 FreeRTOS Kernel

- **版本**: V10.6.2
- **发布日期**: 2023年11月
- **下载源**: https://github.com/FreeRTOS/FreeRTOS-Kernel
- **许可证**: MIT License
- **存放位置**: `lib/FreeRTOS/`

**版本选择原因**：
- ✅ 最新稳定版本
- ✅ 完整支持ARM Cortex-M4F
- ✅ 优化的内存占用和性能
- ✅ 长期支持版本

### 1.2 lwIP TCP/IP协议栈

- **版本**: 2.2.0 (STABLE-2_2_0_RELEASE)
- **发布日期**: 2023年9月
- **下载源**: https://github.com/lwip-tcpip/lwip
- **官方网站**: https://savannah.nongnu.org/projects/lwip
- **许可证**: BSD License
- **存放位置**: `lib/lwip/`

**版本选择原因**：
- ✅ 稳定的2.x分支最新版
- ✅ 完整TCP/IP协议栈支持
- ✅ 优化的内存使用（适合嵌入式）
- ✅ 与FreeRTOS集成良好

---

## 2. 目录结构

```
lib/
├── CMSIS/                          # ARM CMSIS库（已存在）
├── STM32F4xx_HAL_Driver/           # STM32 HAL库（已存在）
├── FreeRTOS/                       # ✅ FreeRTOS内核（已下载）
│   ├── Source/
│   │   ├── tasks.c                 # 任务管理
│   │   ├── queue.c                 # 队列
│   │   ├── timers.c                # 软件定时器
│   │   ├── event_groups.c          # 事件组
│   │   ├── stream_buffer.c         # 流缓冲区
│   │   ├── croutine.c              # 协程
│   │   ├── list.c                  # 链表
│   │   └── portable/               # 移植层
│   │       ├── GCC/                # GCC编译器
│   │       │   └── ARM_CM4F/       # Cortex-M4F支持
│   │       │       ├── port.c
│   │       │       └── portmacro.h
│   │       ├── MemMang/            # 内存管理
│   │       │   ├── heap_1.c        # 最简单（只分配）
│   │       │   ├── heap_2.c        # 可释放（不合并）
│   │       │   ├── heap_3.c        # 使用malloc/free
│   │       │   ├── heap_4.c        # 推荐（合并空闲块）
│   │       │   └── heap_5.c        # 多内存区域
│   │       └── Keil/               # Keil编译器支持
│   ├── include/                    # 头文件
│   │   ├── FreeRTOS.h
│   │   ├── task.h
│   │   ├── queue.h
│   │   ├── semphr.h
│   │   ├── timers.h
│   │   └── ...
│   └── README.md
│
└── lwip/                           # ✅ lwIP协议栈（已下载）
    ├── src/
    │   ├── core/                   # 核心协议栈
    │   │   ├── ipv4/               # IPv4
    │   │   │   ├── ip4.c
    │   │   │   ├── icmp.c
    │   │   │   ├── dhcp.c
    │   │   │   └── ...
    │   │   ├── ipv6/               # IPv6
    │   │   ├── def.c               # 通用定义
    │   │   ├── dns.c               # DNS
    │   │   ├── inet_chksum.c       # 校验和
    │   │   ├── init.c              # 初始化
    │   │   ├── mem.c               # 内存管理
    │   │   ├── memp.c              # 内存池
    │   │   ├── netif.c             # 网络接口
    │   │   ├── pbuf.c              # 数据包缓冲
    │   │   ├── raw.c               # RAW API
    │   │   ├── stats.c             # 统计
    │   │   ├── sys.c               # 系统抽象层
    │   │   ├── tcp.c               # TCP协议
    │   │   ├── tcp_in.c
    │   │   ├── tcp_out.c
    │   │   ├── udp.c               # UDP协议
    │   │   └── timeouts.c          # 超时管理
    │   ├── netif/                  # 网络接口驱动
    │   │   ├── ethernet.c          # 以太网
    │   │   └── etharp.c            # ARP协议
    │   ├── api/                    # 高级API
    │   │   ├── api_lib.c           # netconn API
    │   │   ├── api_msg.c
    │   │   ├── err.c
    │   │   ├── netbuf.c
    │   │   ├── netdb.c
    │   │   ├── netifapi.c
    │   │   ├── sockets.c           # BSD Socket API
    │   │   └── tcpip.c             # TCPIP线程
    │   └── apps/                   # 应用层协议
    │       ├── http/               # HTTP服务器
    │       ├── mqtt/               # MQTT客户端
    │       ├── sntp/               # SNTP客户端
    │       └── ...
    ├── port/                       # 移植层（需要实现）
    │   └── GD32F427/               # ✅ 已创建目录
    │       ├── sys_arch.c          # 系统抽象层实现
    │       ├── sys_arch.h
    │       └── ethernetif.c        # 以太网驱动接口
    └── README.md

config/
├── FreeRTOSConfig.h                # ✅ FreeRTOS配置（已创建）
└── lwipopts.h                      # ✅ lwIP配置（已创建）
```

---

## 3. 配置文件说明

### 3.1 FreeRTOSConfig.h

**位置**: `config/FreeRTOSConfig.h`

**关键配置**：
```c
#define configCPU_CLOCK_HZ            (SystemCoreClock)     // 200MHz
#define configTICK_RATE_HZ            1000                  // 1ms节拍
#define configMAX_PRIORITIES          20                    // 20个优先级
#define configTOTAL_HEAP_SIZE         (32 * 1024)           // 32KB堆
#define configUSE_PREEMPTION          1                     // 抢占式调度
#define configUSE_TIMERS              1                     // 软件定时器
#define configUSE_MUTEXES             1                     // 互斥量
#define configGENERATE_RUN_TIME_STATS 1                     // 运行统计
```

**内存配置**：
- 推荐使用 `heap_4.c`（内存合并，适合频繁分配释放）
- 堆大小：32KB（可根据实际需求调整）

### 3.2 lwipopts.h

**位置**: `config/lwipopts.h`

**关键配置**：
```c
#define NO_SYS                        0              // 使用RTOS
#define MEM_SIZE                      (16 * 1024)    // 16KB堆
#define LWIP_DHCP                     1              // 支持DHCP
#define LWIP_TCP                      1              // 支持TCP
#define LWIP_UDP                      1              // 支持UDP
#define TCP_MSS                       1460           // TCP最大段
#define TCP_WND                       (4 * TCP_MSS)  // TCP窗口
#define PBUF_POOL_SIZE                16             // 缓冲区数量
#define MEMP_NUM_TCP_PCB              10             // TCP连接数
```

**线程配置**：
```c
#define TCPIP_THREAD_PRIO             (configMAX_PRIORITIES - 2)  // TCPIP线程优先级
#define TCPIP_THREAD_STACKSIZE        1024                        // 1KB栈
#define TCPIP_MBOX_SIZE               16                          // 邮箱大小
```

---

## 4. 集成步骤

### 4.1 添加源文件到工程

#### Keil MDK工程配置

1. **添加FreeRTOS源文件**：
   ```
   lib/FreeRTOS/Source/tasks.c
   lib/FreeRTOS/Source/queue.c
   lib/FreeRTOS/Source/list.c
   lib/FreeRTOS/Source/timers.c
   lib/FreeRTOS/Source/event_groups.c
   lib/FreeRTOS/Source/stream_buffer.c
   lib/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c   (或Keil版本)
   lib/FreeRTOS/Source/portable/MemMang/heap_4.c
   ```

2. **添加lwIP源文件**：
   ```
   lib/lwip/src/core/*.c
   lib/lwip/src/core/ipv4/*.c
   lib/lwip/src/netif/ethernet.c
   lib/lwip/src/netif/etharp.c
   lib/lwip/src/api/*.c
   lib/lwip/port/GD32F427/sys_arch.c
   lib/lwip/port/GD32F427/ethernetif.c
   ```

3. **添加头文件路径**：
   ```
   lib/FreeRTOS/Source/include
   lib/FreeRTOS/Source/portable/GCC/ARM_CM4F
   lib/lwip/src/include
   lib/lwip/port/GD32F427
   config
   ```

#### CMake配置

```cmake
# FreeRTOS
add_subdirectory(lib/FreeRTOS)
target_include_directories(${PROJECT_NAME} PRIVATE
    lib/FreeRTOS/Source/include
    lib/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

# lwIP
add_subdirectory(lib/lwip)
target_include_directories(${PROJECT_NAME} PRIVATE
    lib/lwip/src/include
    lib/lwip/port/GD32F427
)
```

### 4.2 实现移植层

#### FreeRTOS移植（已完成）

FreeRTOS的ARM Cortex-M4F移植已包含在源码中，只需配置`FreeRTOSConfig.h`。

#### lwIP移植（需要实现）

需要实现以下文件：

**1. sys_arch.c** - 系统抽象层
```c
// lib/lwip/port/GD32F427/sys_arch.c
// 实现以下函数：
sys_sem_t sys_sem_new(u8_t count);
void sys_sem_free(sys_sem_t *sem);
void sys_sem_signal(sys_sem_t *sem);
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout);

sys_mutex_t sys_mutex_new(void);
void sys_mutex_free(sys_mutex_t *mutex);
void sys_mutex_lock(sys_mutex_t *mutex);
void sys_mutex_unlock(sys_mutex_t *mutex);

sys_mbox_t sys_mbox_new(int size);
void sys_mbox_free(sys_mbox_t *mbox);
void sys_mbox_post(sys_mbox_t *mbox, void *msg);
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg);
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout);
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg);

sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread,
                             void *arg, int stacksize, int prio);
```

**2. ethernetif.c** - 以太网驱动接口
```c
// lib/lwip/port/GD32F427/ethernetif.c
// 实现以下函数：
err_t ethernetif_init(struct netif *netif);
void ethernetif_input(struct netif *netif);
static err_t low_level_output(struct netif *netif, struct pbuf *p);
static struct pbuf *low_level_input(struct netif *netif);
```

### 4.3 启动代码示例

```c
// app/main.c

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "ethernetif.h"

// 网络接口
struct netif gnetif;

// TCPIP初始化完成回调
static void tcpip_init_done(void *arg)
{
    ip4_addr_t ipaddr, netmask, gw;

    // 配置静态IP（或使用DHCP）
    IP4_ADDR(&ipaddr, 192, 168, 1, 100);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 1, 1);

    // 添加网络接口
    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

    // 设置默认网络接口
    netif_set_default(&gnetif);

    // 启动网络接口
    if (netif_is_link_up(&gnetif)) {
        netif_set_up(&gnetif);
    }

    printf("TCPIP initialized\n");
}

int main(void)
{
    // 硬件初始化
    SystemClock_Config();
    HAL_Init();

    // 初始化TCPIP协议栈
    tcpip_init(tcpip_init_done, NULL);

    // 创建应用任务
    xTaskCreate(app_main_task, "app_main", 512, NULL, tskIDLE_PRIORITY + 2, NULL);

    // 启动调度器
    vTaskStartScheduler();

    // 不应该到达这里
    while (1);
}
```

---

## 5. 内存占用估算

### 5.1 FreeRTOS内存占用

| 项目 | 大小 | 说明 |
|------|------|------|
| 内核代码 | ~12KB | tasks.c, queue.c等 |
| 每个任务TCB | ~100 bytes | 任务控制块 |
| 每个任务栈 | 可配置 | 一般512~2048字节 |
| 堆内存 | 32KB | configTOTAL_HEAP_SIZE |
| **总计** | **~50KB** | 10个任务估算 |

### 5.2 lwIP内存占用

| 项目 | 大小 | 说明 |
|------|------|------|
| 协议栈代码 | ~40KB | core + api |
| 堆内存 | 16KB | MEM_SIZE |
| PBUF池 | ~25KB | 16个1536字节 |
| TCP/UDP PCB | ~2KB | 连接控制块 |
| TCPIP线程栈 | 1KB | 协议栈线程 |
| **总计** | **~84KB** | - |

### 5.3 GD32F427资源

| 资源 | 总量 | FreeRTOS+lwIP | 剩余 |
|------|------|---------------|------|
| Flash | 1MB | ~50KB | ~974KB |
| SRAM | 192KB | ~134KB | ~58KB |
| CCM | 64KB | 未使用 | 64KB |

**结论**：GD32F427资源充足，可以运行FreeRTOS + lwIP + 应用程序。

---

## 6. 测试验证

### 6.1 FreeRTOS测试

```c
// 创建测试任务
void vTestTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(1000);

    for (;;) {
        printf("Task running\n");
        vTaskDelay(xDelay);
    }
}

// main.c
xTaskCreate(vTestTask, "test", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
```

### 6.2 lwIP测试

```c
// 创建TCP服务器
void tcp_server_task(void *pvParameters)
{
    struct netconn *conn, *newconn;
    err_t err;

    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, IP_ADDR_ANY, 8888);
    netconn_listen(conn);

    while (1) {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            // 处理连接
            netconn_close(newconn);
            netconn_delete(newconn);
        }
    }
}
```

---

## 7. 常见问题

### Q1: 编译错误找不到头文件？
**A**: 检查头文件路径是否添加到工程Include Paths。

### Q2: 运行时卡死或HardFault？
**A**: 检查：
1. 栈大小是否足够（每个任务至少512字节）
2. 堆大小是否足够
3. 中断优先级配置是否正确

### Q3: 网络不通？
**A**: 检查：
1. 以太网PHY是否初始化
2. MAC地址是否配置
3. IP地址、网关、子网掩码是否正确
4. 网线是否连接

### Q4: 内存不足？
**A**:
1. 减少任务数量或栈大小
2. 减少lwIP缓冲区（PBUF_POOL_SIZE、TCP_WND等）
3. 使用CCM内存存放部分数据

---

## 8. 参考资源

### 8.1 官方文档

- **FreeRTOS**: https://www.freertos.org/
  - API参考: https://www.freertos.org/a00106.html
  - 书籍: "Mastering the FreeRTOS Real Time Kernel"

- **lwIP**: https://www.nongnu.org/lwip/
  - Wiki: https://lwip.fandom.com/wiki/LwIP_Wiki
  - API: https://www.nongnu.org/lwip/2_1_x/group__lwip__api.html

### 8.2 示例代码

- STM32 FreeRTOS示例: https://github.com/STMicroelectronics/STM32CubeF4
- lwIP移植示例: https://github.com/lwip-tcpip/lwip-contrib

---

## 9. 下载脚本

### 9.1 自动下载脚本 (Windows)

```bash
# download_libs.bat
@echo off
cd lib

echo Downloading FreeRTOS...
curl -L "https://github.com/FreeRTOS/FreeRTOS-Kernel/archive/refs/tags/V10.6.2.zip" -o freertos.zip
tar -xf freertos.zip
rename FreeRTOS-Kernel-10.6.2 FreeRTOS
del freertos.zip

echo Downloading lwIP...
curl -L "https://github.com/lwip-tcpip/lwip/archive/refs/tags/STABLE-2_2_0_RELEASE.zip" -o lwip.zip
tar -xf lwip.zip
rename lwip-STABLE-2_2_0_RELEASE lwip
del lwip.zip

echo Done!
pause
```

### 9.2 Linux/macOS脚本

```bash
#!/bin/bash
cd lib

echo "Downloading FreeRTOS..."
curl -L "https://github.com/FreeRTOS/FreeRTOS-Kernel/archive/refs/tags/V10.6.2.zip" -o freertos.zip
unzip -q freertos.zip
mv FreeRTOS-Kernel-10.6.2 FreeRTOS
rm freertos.zip

echo "Downloading lwIP..."
curl -L "https://github.com/lwip-tcpip/lwip/archive/refs/tags/STABLE-2_2_0_RELEASE.zip" -o lwip.zip
unzip -q lwip.zip
mv lwip-STABLE-2_2_0_RELEASE lwip
rm lwip.zip

echo "Done!"
```

---

**文档版本**: v1.0
**更新日期**: 2025-09-30
**下载完成**: ✅ FreeRTOS V10.6.2 + lwIP 2.2.0