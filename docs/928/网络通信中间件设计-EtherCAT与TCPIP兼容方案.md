# 网络通信中间件设计 - EtherCAT与TCP/IP兼容方案

## 1. 网络功能在分层架构中的位置

### 1.1 传统网络分层
```
应用层 (app/)           ← TCP服务器、EtherCAT应用
    ↓
中间件层 (middleware/)   ← 网络抽象、协议管理 [重点讨论]
    ↓
HAL层 (hal/)           ← 以太网HAL抽象
    ↓
驱动层 (drivers/)      ← 以太网MAC驱动
    ↓
lib层                 ← lwIP协议栈、EtherCAT库
```

### 1.2 为什么网络功能适合放在中间层？

**核心原因：协议抽象和服务统一**

#### 1.2.1 没有中间层抽象的问题
```c
// app/sensor_task.c - 应用层直接使用协议
void sensor_data_send_task(void)
{
    sensor_data_t data;
    collect_sensor_data(&data);

    // 问题：应用层需要知道用哪个协议
    if (system_mode == MODE_ETHERCAT) {
        ethercat_send_pdo(&data);        // EtherCAT协议
    } else if (system_mode == MODE_TCP) {
        tcp_send_sensor_data(&data);     // TCP协议
    }
    // 如果增加新协议，需要修改所有应用代码！
}
```

#### 1.2.2 使用中间层抽象的优势
```c
// app/sensor_task.c - 应用层使用统一接口
void sensor_data_send_task(void)
{
    sensor_data_t data;
    collect_sensor_data(&data);

    // 统一接口，不关心底层协议
    comm_send_sensor_data(&data);  // 中间件提供统一接口
}

// middleware/comm_manager.c - 中间件处理协议选择
hal_status_t comm_send_sensor_data(sensor_data_t* data)
{
    switch(g_comm_config.active_protocol) {
        case PROTOCOL_ETHERCAT:
            return ethercat_adapter_send_data(data);
        case PROTOCOL_TCP:
            return tcp_adapter_send_data(data);
        default:
            return HAL_ERROR;
    }
}
```

## 2. EtherCAT与TCP/IP兼容挑战

### 2.1 技术挑战分析

#### 2.1.1 硬件资源共享
```
┌─────────────────────────────────────────────┐
│              GD32F427                       │
│  ┌─────────────────────────────────────┐    │
│  │         以太网MAC控制器              │    │
│  └─────────────────────────────────────┘    │
│                    ↓                        │
│  ┌─────────────────────────────────────┐    │
│  │         PHY芯片 (如LAN8720)         │    │
│  └─────────────────────────────────────┘    │
└─────────────────────────────────────────────┘
               ↓
    ┌─────────────────┐    ┌─────────────────┐
    │   EtherCAT网络   │    │    TCP/IP网络   │
    │     (工业)      │    │      (管理)     │
    └─────────────────┘    └─────────────────┘
```

**问题：一个以太网接口，两个协议需求**

#### 2.1.2 协议特性差异
| 特性 | EtherCAT | TCP/IP |
|------|----------|--------|
| 实时性 | 硬实时 (1ms周期) | 软实时 |
| 数据类型 | PDO (过程数据) | 任意数据包 |
| 通信模式 | 主从模式 | 客户端/服务器 |
| 网络拓扑 | 环形拓扑 | 星形/树形 |
| 协议栈 | 专用协议栈 | 标准TCP/IP栈 |

#### 2.1.3 应用场景需求
```
EtherCAT用途：
- 实时过程数据交换 (温度、压力、液位、控制输出)
- 高频数据更新 (1ms)
- 与PLC/运动控制器通信

TCP/IP用途：
- 远程监控和配置
- 历史数据传输
- Web界面访问
- 第三方系统集成
```

### 2.2 兼容方案设计

#### 2.2.1 方案1：时分复用（推荐）
```c
// middleware/comm_manager.c
typedef enum {
    COMM_MODE_ETHERCAT_ONLY,    // 纯EtherCAT模式
    COMM_MODE_TCP_ONLY,         // 纯TCP模式
    COMM_MODE_TIME_DIVISION,    // 时分复用模式
    COMM_MODE_DUAL_STACK        // 双栈模式（如果硬件支持）
} comm_mode_t;

// 时分复用调度
void comm_time_division_scheduler(void)
{
    static uint32_t time_slot = 0;

    if (g_comm_config.mode == COMM_MODE_TIME_DIVISION) {
        time_slot = (time_slot + 1) % 100;  // 100ms周期

        if (time_slot < 80) {
            // 80% 时间给EtherCAT (实时性优先)
            comm_set_active_protocol(PROTOCOL_ETHERCAT);
        } else {
            // 20% 时间给TCP/IP (管理通信)
            comm_set_active_protocol(PROTOCOL_TCP);
        }
    }
}
```

#### 2.2.2 方案2：优先级抢占
```c
typedef struct {
    protocol_type_t type;
    uint8_t priority;        // 优先级 (数字越大优先级越高)
    bool is_active;
    uint32_t last_activity;
    uint32_t timeout_ms;
} protocol_info_t;

protocol_info_t protocols[] = {
    {PROTOCOL_ETHERCAT, 10, false, 0, 100},  // 高优先级，100ms超时
    {PROTOCOL_TCP,      5,  false, 0, 1000}, // 低优先级，1s超时
};

void comm_priority_scheduler(void)
{
    // EtherCAT有数据时立即抢占
    if (ethercat_has_pending_data()) {
        comm_switch_to_protocol(PROTOCOL_ETHERCAT);
    }
    // EtherCAT空闲时，TCP可以使用
    else if (tcp_has_pending_data()) {
        comm_switch_to_protocol(PROTOCOL_TCP);
    }
}
```

## 3. 中间件层网络架构设计

### 3.1 整体架构
```
middleware/network/
├── comm_manager.c/h        # 通信管理器 (核心)
├── protocol_adapter.c/h    # 协议适配器
├── message_queue.c/h       # 消息队列管理
├── network_config.c/h      # 网络配置管理
└── comm_monitor.c/h        # 通信监控
```

### 3.2 通信管理器设计

#### 3.2.1 核心数据结构
```c
// middleware/network/comm_manager.h
typedef enum {
    PROTOCOL_NONE = 0,
    PROTOCOL_ETHERCAT,
    PROTOCOL_TCP,
    PROTOCOL_COUNT
} protocol_type_t;

typedef struct {
    protocol_type_t active_protocol;     // 当前活动协议
    comm_mode_t mode;                   // 工作模式
    bool ethercat_enabled;              // EtherCAT使能
    bool tcp_enabled;                   // TCP使能
    uint32_t switch_interval_ms;        // 切换间隔
    uint8_t ethercat_time_percent;      // EtherCAT时间占比
} comm_config_t;

typedef struct {
    // 统计信息
    uint32_t ethercat_tx_count;
    uint32_t ethercat_rx_count;
    uint32_t tcp_tx_count;
    uint32_t tcp_rx_count;
    uint32_t protocol_switches;
    uint32_t communication_errors;

    // 状态信息
    bool ethercat_connected;
    bool tcp_connected;
    uint32_t last_ethercat_activity;
    uint32_t last_tcp_activity;
} comm_status_t;

extern comm_config_t g_comm_config;
extern comm_status_t g_comm_status;
```

#### 3.2.2 统一通信接口
```c
// middleware/network/comm_manager.h - 统一对外接口

// 数据发送接口 (应用层调用)
hal_status_t comm_send_sensor_data(sensor_data_t* data);
hal_status_t comm_send_control_data(control_data_t* data);
hal_status_t comm_send_alarm_data(alarm_data_t* data);
hal_status_t comm_send_config_data(config_data_t* data);

// 数据接收接口 (应用层注册回调)
typedef void (*comm_data_callback_t)(void* data, uint16_t len, protocol_type_t from_protocol);

hal_status_t comm_register_sensor_callback(comm_data_callback_t callback);
hal_status_t comm_register_control_callback(comm_data_callback_t callback);
hal_status_t comm_register_config_callback(comm_data_callback_t callback);

// 协议管理接口
hal_status_t comm_manager_init(void);
hal_status_t comm_set_mode(comm_mode_t mode);
hal_status_t comm_enable_protocol(protocol_type_t protocol, bool enable);
comm_status_t comm_get_status(void);
```

#### 3.2.3 协议适配器
```c
// middleware/network/protocol_adapter.c
typedef struct {
    protocol_type_t type;
    hal_status_t (*init)(void);
    hal_status_t (*send)(void* data, uint16_t len);
    hal_status_t (*receive)(void* buffer, uint16_t* len);
    hal_status_t (*start)(void);
    hal_status_t (*stop)(void);
    bool (*is_connected)(void);
} protocol_adapter_t;

// EtherCAT适配器
static protocol_adapter_t ethercat_adapter = {
    .type = PROTOCOL_ETHERCAT,
    .init = ethercat_adapter_init,
    .send = ethercat_adapter_send,
    .receive = ethercat_adapter_receive,
    .start = ethercat_adapter_start,
    .stop = ethercat_adapter_stop,
    .is_connected = ethercat_adapter_is_connected,
};

// TCP适配器
static protocol_adapter_t tcp_adapter = {
    .type = PROTOCOL_TCP,
    .init = tcp_adapter_init,
    .send = tcp_adapter_send,
    .receive = tcp_adapter_receive,
    .start = tcp_adapter_start,
    .stop = tcp_adapter_stop,
    .is_connected = tcp_adapter_is_connected,
};

static protocol_adapter_t* adapters[] = {
    &ethercat_adapter,
    &tcp_adapter,
};
```

### 3.3 消息队列管理

#### 3.3.1 消息队列设计
```c
// middleware/network/message_queue.h
typedef enum {
    MSG_TYPE_SENSOR_DATA,
    MSG_TYPE_CONTROL_DATA,
    MSG_TYPE_ALARM_DATA,
    MSG_TYPE_CONFIG_DATA,
    MSG_TYPE_COUNT
} message_type_t;

typedef struct {
    message_type_t type;
    protocol_type_t target_protocol;  // 目标协议
    uint16_t priority;               // 优先级
    uint32_t timestamp;
    uint16_t data_len;
    uint8_t data[256];
} comm_message_t;

typedef struct {
    QueueHandle_t ethercat_tx_queue;
    QueueHandle_t ethercat_rx_queue;
    QueueHandle_t tcp_tx_queue;
    QueueHandle_t tcp_rx_queue;
    SemaphoreHandle_t queue_mutex;
} message_queue_manager_t;
```

#### 3.3.2 消息路由
```c
hal_status_t comm_route_message(comm_message_t* msg)
{
    // 根据协议状态和优先级路由消息
    if (msg->target_protocol == PROTOCOL_ETHERCAT) {
        if (g_comm_status.ethercat_connected) {
            return queue_send_message(&g_msg_manager.ethercat_tx_queue, msg);
        } else {
            // EtherCAT未连接，尝试TCP备用
            msg->target_protocol = PROTOCOL_TCP;
            return queue_send_message(&g_msg_manager.tcp_tx_queue, msg);
        }
    } else if (msg->target_protocol == PROTOCOL_TCP) {
        return queue_send_message(&g_msg_manager.tcp_tx_queue, msg);
    }

    return HAL_ERROR;
}
```

## 4. 具体实现示例

### 4.1 应用层使用示例

#### 4.1.1 传感器数据发送
```c
// app/sensor_task.c
void sensor_task(void *pvParameters)
{
    sensor_data_t sensor_data;

    while(1) {
        // 采集传感器数据
        collect_all_sensors(&sensor_data);

        // 通过统一接口发送，中间件自动选择协议
        comm_send_sensor_data(&sensor_data);

        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms周期
    }
}
```

#### 4.1.2 控制命令接收
```c
// app/control_task.c
void control_command_callback(void* data, uint16_t len, protocol_type_t from_protocol)
{
    control_command_t* cmd = (control_command_t*)data;

    // 处理控制命令，不关心来自哪个协议
    switch(cmd->type) {
        case CMD_SET_TEMPERATURE:
            set_temperature_setpoint(cmd->channel, cmd->value);
            break;
        case CMD_SET_VALVE_STATE:
            set_valve_state(cmd->channel, cmd->value);
            break;
    }

    // 发送确认响应 (自动选择最佳协议)
    control_response_t response;
    response.cmd_id = cmd->id;
    response.result = RESULT_OK;
    comm_send_control_data(&response);
}

void control_task_init(void)
{
    // 注册控制命令回调
    comm_register_control_callback(control_command_callback);
}
```

### 4.2 协议切换示例

#### 4.2.1 自动协议选择
```c
// middleware/network/comm_manager.c
hal_status_t comm_send_sensor_data(sensor_data_t* data)
{
    comm_message_t msg;
    msg.type = MSG_TYPE_SENSOR_DATA;
    msg.priority = 5;  // 传感器数据中等优先级

    // 智能协议选择
    if (data->is_realtime_critical) {
        // 实时关键数据优先用EtherCAT
        msg.target_protocol = PROTOCOL_ETHERCAT;
    } else {
        // 非关键数据可以用TCP
        msg.target_protocol = PROTOCOL_TCP;
    }

    // 复制数据
    memcpy(msg.data, data, sizeof(sensor_data_t));
    msg.data_len = sizeof(sensor_data_t);

    return comm_route_message(&msg);
}
```

#### 4.2.2 协议故障切换
```c
void comm_monitor_task(void *pvParameters)
{
    while(1) {
        // 检查EtherCAT连接状态
        if (!ethercat_adapter.is_connected()) {
            if (g_comm_status.ethercat_connected) {
                g_comm_status.ethercat_connected = false;
                printf("EtherCAT连接丢失，切换到TCP模式\n");

                // 自动切换到TCP模式
                comm_set_mode(COMM_MODE_TCP_ONLY);
            }
        } else {
            if (!g_comm_status.ethercat_connected) {
                g_comm_status.ethercat_connected = true;
                printf("EtherCAT连接恢复，恢复双协议模式\n");

                // 恢复双协议模式
                comm_set_mode(COMM_MODE_TIME_DIVISION);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1s检查一次
    }
}
```

## 5. 配置和管理

### 5.1 网络配置管理
```c
// config/network_config.h
typedef struct {
    // EtherCAT配置
    struct {
        uint16_t vendor_id;
        uint16_t product_code;
        uint16_t station_address;
        uint32_t cycle_time_us;     // 循环时间 (微秒)
        bool auto_startup;
    } ethercat;

    // TCP/IP配置
    struct {
        uint32_t ip_address;        // IP地址
        uint32_t netmask;          // 子网掩码
        uint32_t gateway;          // 网关
        uint16_t tcp_port;         // TCP服务端口
        uint16_t max_connections;   // 最大连接数
    } tcp;

    // 协议管理配置
    struct {
        comm_mode_t default_mode;
        uint8_t ethercat_priority_percent;  // EtherCAT时间占比
        uint32_t protocol_switch_delay_ms;  // 协议切换延时
        bool auto_failover;                // 自动故障切换
    } management;
} network_config_t;
```

### 5.2 通过USMART调试
```c
// 在usmart_config.c中注册网络调试函数
(void*)comm_get_status,"comm_status_t comm_get_status(void)",
(void*)comm_set_mode,"hal_status_t comm_set_mode(u8 mode)",
(void*)comm_print_statistics,"void comm_print_statistics(void)",
```

**串口调试示例：**
```bash
# 查看通信状态
comm_get_status()

# 切换到EtherCAT模式
comm_set_mode(1)

# 切换到TCP模式
comm_set_mode(2)

# 切换到时分复用模式
comm_set_mode(3)

# 查看通信统计
comm_print_statistics()
```

## 6. 实施建议

### 6.1 实施优先级

**第一阶段：基础框架**
1. 实现通信管理器基础框架
2. 实现协议适配器接口
3. 支持单一协议模式

**第二阶段：协议兼容**
4. 实现时分复用调度
5. 实现消息队列管理
6. 支持协议切换

**第三阶段：智能管理**
7. 实现智能协议选择
8. 实现故障自动切换
9. 完善监控和调试功能

### 6.2 目录结构建议

```
middleware/
├── network/                    # 网络中间件 (新增)
│   ├── comm_manager.c/h        # 通信管理器
│   ├── protocol_adapter.c/h    # 协议适配器
│   ├── message_queue.c/h       # 消息队列
│   ├── network_config.c/h      # 网络配置
│   └── comm_monitor.c/h        # 通信监控
├── delay/                      # 保持现有
├── sys/                        # 保持现有
├── usart/                      # 保持现有
├── pid.c/h                     # PID算法
└── filter.c/h                  # 滤波算法
```

## 7. 总结

### 7.1 为什么网络功能适合放在中间层？

1. **协议抽象** - 为应用层提供统一接口
2. **兼容性处理** - 处理多协议共存问题
3. **智能调度** - 协议切换和资源分配
4. **故障处理** - 自动切换和恢复
5. **配置管理** - 统一的网络参数管理

### 7.2 设计优势

- ✅ **应用层简化** - 统一接口，不关心底层协议
- ✅ **协议解耦** - 增加新协议不影响应用代码
- ✅ **智能调度** - 自动选择最佳协议
- ✅ **故障容错** - 协议故障时自动切换
- ✅ **便于调试** - 统一的监控和配置接口

### 7.3 适用场景

特别适合需要**多协议兼容**的工业控制系统：
- EtherCAT用于实时控制
- TCP/IP用于远程监控
- 自动故障切换保证可靠性

这种设计将网络通信的复杂性封装在中间层，让应用层专注于业务逻辑，是一个很好的架构选择！

---

*文档版本: v1.0*
*创建时间: 2025-09-29*
*网络通信中间件架构设计方案*