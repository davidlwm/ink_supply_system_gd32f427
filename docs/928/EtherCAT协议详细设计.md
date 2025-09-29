# 墨路控制系统EtherCAT协议详细设计

## 1. 协议选择机制

### 1.1 通信方式选择
系统支持两种通信方式，运行时二选一：
- **EtherCAT协议**：高实时性工业现场总线通信
- **TCP协议**：标准以太网TCP/IP通信

### 1.2 选择依据
```c
typedef enum {
    COMM_MODE_TCP = 0,        // TCP/IP模式
    COMM_MODE_ETHERCAT = 1    // EtherCAT模式
} communication_mode_t;

// 通信模式配置
typedef struct {
    communication_mode_t mode;        // 通信模式
    bool auto_switch_enable;          // 自动切换使能
    uint32_t switch_timeout;          // 切换超时时间(ms)
    uint8_t priority_protocol;        // 优先协议 (0:TCP 1:EtherCAT)
} comm_mode_config_t;
```

## 2. EtherCAT硬件架构

### 2.1 硬件连接
```
GD32F427 (主控) <--SPI--> GDSCN832R2U6 (EtherCAT控制器)
                                |
                                | EtherCAT网络
                                |
                            上位机(EtherCAT主站)
```

### 2.2 GDSCN832R2U6控制器特性
```c
typedef struct {
    // 基本特性
    char controller_model[16];    // "GDSCN832R2U6"
    uint8_t port_count;          // 2 (双端口EtherCAT)
    uint32_t max_baudrate;       // 100Mbps
    bool hardware_realtime;      // true (硬件实时处理)
    bool distributed_clock;      // true (分布式时钟支持)

    // 性能参数
    uint16_t min_cycle_time;     // 125μs (最小周期时间)
    uint16_t typical_cycle_time; // 1ms (典型周期时间)
    uint16_t max_process_data;   // 1486字节 (最大过程数据)
    uint8_t max_mailbox_data;    // 64字节 (邮箱数据)

    // 接口特性
    uint32_t spi_max_frequency;  // 50MHz (SPI最大频率)
    uint8_t spi_mode;            // SPI模式0 (CPOL=0, CPHA=0)
    bool interrupt_support;      // true (中断支持)
} ethercat_controller_spec_t;
```

## 3. EtherCAT从站配置

### 3.1 ESI (EtherCAT Slave Information) 配置
```xml
<!-- 设备标识 -->
<Vendor>
    <Id>0x1234</Id>  <!-- 厂商ID -->
    <Name>MoLu Systems</Name>
</Vendor>

<Descriptions>
    <Groups>
        <Group>
            <Type>Ink Control System</Type>
            <Name>墨路控制系统</Name>
        </Group>
    </Groups>
    <Devices>
        <Device>
            <Type ProductCode="0x5678" RevisionNo="0x0001">墨路控制器</Type>
            <Name>MoLu Controller V1.0</Name>
        </Device>
    </Devices>
</Descriptions>
```

### 3.2 从站设备信息
```c
typedef struct {
    // 设备标识
    uint16_t vendor_id;          // 0x1234 (厂商ID)
    uint32_t product_code;       // 0x5678 (产品代码)
    uint32_t revision_number;    // 0x00010001 (版本号)
    uint32_t serial_number;      // 唯一序列号

    // 网络配置
    uint16_t station_address;    // 从站地址 (1-65535)
    char device_name[32];        // "MoLu_Controller_001"
    char device_type[32];        // "Ink Control System"

    // 功能特性
    bool coe_support;            // CANopen over EtherCAT支持
    bool foe_support;            // File over EtherCAT支持
    bool eoe_support;            // Ethernet over EtherCAT支持
    bool soe_support;            // Servo over EtherCAT支持
} ethercat_device_info_t;
```

## 4. 过程数据映射 (PDO)

### 4.1 输入PDO (主站→从站)
```c
// TxPDO: 从站发送给主站的数据
typedef struct {
    // 传感器数据区 (24字节)
    struct {
        int16_t temperature[3];      // 温度值 (°C*100)
        uint16_t pressure[4];        // 压力值 (kPa*10)
        uint16_t level[4];           // 液位值 (mm*10)
        uint16_t flow_rate;          // 流量值 (L/min*100)
        uint8_t sensor_status;       // 传感器状态位
    } sensors;

    // 系统状态区 (8字节)
    struct {
        uint16_t system_state;       // 系统状态字
        uint16_t alarm_word;         // 告警状态字
        uint16_t fault_code;         // 故障代码
        uint8_t cpu_usage;           // CPU使用率(%)
        uint8_t temperature_mcu;     // MCU温度(°C)
    } system_status;

    // 输出反馈区 (12字节)
    struct {
        uint16_t valve_feedback[2];  // 电磁阀反馈
        uint16_t heater_feedback[3]; // 加热器反馈
        uint16_t pump_speed_fb[2];   // 调速泵转速反馈
        uint16_t pump_dc_status[2];  // 直流泵状态反馈
    } actuator_feedback;

    // 时间戳 (4字节)
    uint32_t timestamp;              // 系统时间戳

} __attribute__((packed)) ethercat_input_pdo_t;  // 总计48字节
```

### 4.2 输出PDO (主站→从站)
```c
// RxPDO: 主站发送给从站的数据
typedef struct {
    // 控制命令区 (16字节)
    struct {
        uint16_t control_word;       // 控制字
        uint8_t system_command;      // 系统命令
        uint8_t operation_mode;      // 操作模式
        int16_t temp_setpoint[3];    // 温度设定值 (°C*100)
        uint16_t pressure_setpoint[4]; // 压力设定值 (kPa*10)
        uint16_t level_setpoint[3];  // 液位设定值 (mm*10)
        uint16_t flow_setpoint;      // 流量设定值 (L/min*100)
    } control_commands;

    // 执行器控制区 (12字节)
    struct {
        uint16_t valve_control[2];   // 电磁阀控制 (0:关闭 1:开启)
        uint16_t heater_control[3];  // 加热器控制 (0-100%)
        uint16_t pump_speed[2];      // 调速泵速度 (0-5000 RPM)
        uint16_t pump_dc_control[2]; // 直流泵控制 (0:停止 1:启动)
    } actuator_control;

    // PID参数区 (16字节) - 可选
    struct {
        float kp, ki, kd;            // PID参数
        uint8_t loop_id;             // 回路ID
        uint8_t pid_enable;          // PID使能
        uint8_t reserved[2];         // 保留
    } pid_params;

} __attribute__((packed)) ethercat_output_pdo_t;  // 总计44字节
```

### 4.3 PDO映射配置
```c
// PDO映射表定义
typedef struct {
    uint16_t index;              // 对象字典索引
    uint8_t subindex;            // 子索引
    uint8_t bit_length;          // 数据位长度
    char description[32];        // 描述
} pdo_mapping_entry_t;

// 输入PDO映射表 (1A00h)
static const pdo_mapping_entry_t input_pdo_mapping[] = {
    {0x6000, 0x01, 16, "Temperature 1"},
    {0x6000, 0x02, 16, "Temperature 2"},
    {0x6000, 0x03, 16, "Temperature 3"},
    {0x6001, 0x01, 16, "Pressure 1"},
    {0x6001, 0x02, 16, "Pressure 2"},
    {0x6001, 0x03, 16, "Pressure 3"},
    {0x6001, 0x04, 16, "Pressure 4"},
    // ... 其他映射项
};

// 输出PDO映射表 (1600h)
static const pdo_mapping_entry_t output_pdo_mapping[] = {
    {0x7000, 0x01, 16, "Control Word"},
    {0x7001, 0x01, 16, "Temp Setpoint 1"},
    {0x7001, 0x02, 16, "Temp Setpoint 2"},
    {0x7001, 0x03, 16, "Temp Setpoint 3"},
    // ... 其他映射项
};
```

## 5. 对象字典 (Object Dictionary)

### 5.1 标准对象
```c
// 标准CANopen对象字典条目
typedef struct {
    // 设备信息区 (1000h-1FFFh)
    uint32_t device_type;        // 1000h: 设备类型
    uint8_t error_register;      // 1001h: 错误寄存器
    char manufacturer_name[32];  // 1008h: 制造商名称
    char hardware_version[16];   // 1009h: 硬件版本
    char software_version[16];   // 100Ah: 软件版本
    uint32_t serial_number;      // 1018h: 序列号

    // PDO通信参数 (1400h-1BFFh)
    uint32_t rx_pdo_comm_param[4]; // 1400h-1403h: RxPDO通信参数
    uint32_t tx_pdo_comm_param[4]; // 1800h-1803h: TxPDO通信参数

    // PDO映射参数 (1600h-1AFFh)
    uint32_t rx_pdo_mapping[4][8]; // 1600h-1603h: RxPDO映射
    uint32_t tx_pdo_mapping[4][8]; // 1A00h-1A03h: TxPDO映射

} ethercat_object_dictionary_t;
```

### 5.2 制造商特定对象 (2000h-5FFFh)
```c
// 制造商特定对象字典
typedef struct {
    // 传感器参数区 (2000h-2FFFh)
    struct {
        float temp_offset[3];        // 2000h: 温度零点偏移
        float temp_scale[3];         // 2001h: 温度标定系数
        float pressure_offset[4];    // 2002h: 压力零点偏移
        float pressure_scale[4];     // 2003h: 压力标定系数
        float level_offset[4];       // 2004h: 液位零点偏移
        float level_scale[4];        // 2005h: 液位标定系数
        float flow_offset;           // 2006h: 流量零点偏移
        float flow_scale;            // 2007h: 流量标定系数
    } sensor_calibration;

    // 控制参数区 (3000h-3FFFh)
    struct {
        float pid_kp[8];             // 3000h: PID比例系数
        float pid_ki[8];             // 3001h: PID积分系数
        float pid_kd[8];             // 3002h: PID微分系数
        float output_limit_min[8];   // 3003h: 输出下限
        float output_limit_max[8];   // 3004h: 输出上限
        bool reverse_action[8];      // 3005h: 反作用标志
    } control_parameters;

    // 安全参数区 (4000h-4FFFh)
    struct {
        float alarm_high[12];        // 4000h: 高限报警
        float alarm_low[12];         // 4001h: 低限报警
        float shutdown_high[12];     // 4002h: 高限停机
        float shutdown_low[12];      // 4003h: 低限停机
        uint16_t delay_time[12];     // 4004h: 延时时间
        bool alarm_enable[12];       // 4005h: 报警使能
    } safety_parameters;

    // 系统配置区 (5000h-5FFFh)
    struct {
        uint32_t cycle_time;         // 5000h: 循环周期时间
        bool distributed_clock_enable; // 5001h: 分布式时钟使能
        uint16_t sync_mode;          // 5002h: 同步模式
        uint32_t watchdog_timeout;   // 5003h: 看门狗超时
        bool auto_startup;           // 5004h: 自动启动
        uint8_t led_brightness;      // 5005h: LED亮度
    } system_config;

} manufacturer_object_dictionary_t;
```

## 6. 状态机定义

### 6.1 EtherCAT状态机
```c
typedef enum {
    EC_STATE_INIT = 0x01,        // 初始化状态
    EC_STATE_PREOP = 0x02,       // 预操作状态
    EC_STATE_BOOTSTRAP = 0x03,   // 引导状态
    EC_STATE_SAFEOP = 0x04,      // 安全操作状态
    EC_STATE_OP = 0x08,          // 操作状态
} ethercat_state_t;

typedef struct {
    ethercat_state_t current_state;
    ethercat_state_t requested_state;
    uint16_t error_code;
    bool state_change_pending;
    uint32_t state_enter_time;
    uint32_t state_duration;
} ethercat_state_machine_t;
```

### 6.2 状态转换处理
```c
// 状态转换函数
typedef struct {
    ethercat_state_t from_state;
    ethercat_state_t to_state;
    bool (*transition_handler)(void);
    char description[32];
} state_transition_t;

static const state_transition_t state_transitions[] = {
    {EC_STATE_INIT, EC_STATE_PREOP, init_to_preop_handler, "Init to PreOp"},
    {EC_STATE_PREOP, EC_STATE_SAFEOP, preop_to_safeop_handler, "PreOp to SafeOp"},
    {EC_STATE_SAFEOP, EC_STATE_OP, safeop_to_op_handler, "SafeOp to Op"},
    {EC_STATE_OP, EC_STATE_SAFEOP, op_to_safeop_handler, "Op to SafeOp"},
    // ... 其他转换
};
```

## 7. 分布式时钟 (DC)

### 7.1 时钟同步配置
```c
typedef struct {
    bool dc_enable;              // 分布式时钟使能
    uint32_t sync0_cycle_time;   // SYNC0周期时间 (ns)
    uint32_t sync1_cycle_time;   // SYNC1周期时间 (ns)
    int32_t sync0_shift_time;    // SYNC0偏移时间 (ns)
    int32_t sync1_shift_time;    // SYNC1偏移时间 (ns)
    uint16_t dc_sync_mode;       // 同步模式
    uint32_t cycle_time_ns;      // 周期时间 (纳秒)
} distributed_clock_config_t;
```

### 7.2 时钟同步处理
```c
// SYNC0事件处理 (1ms周期)
void sync0_event_handler(void) {
    // 触发传感器数据采集
    sensor_task_trigger();

    // 更新控制算法
    control_task_trigger();

    // 更新PDO数据
    update_input_pdo_data();
    process_output_pdo_data();
}

// SYNC1事件处理 (可选)
void sync1_event_handler(void) {
    // 触发次要任务
    led_task_trigger();
    hmi_task_trigger();
}
```

## 8. 邮箱通信 (Mailbox)

### 8.1 邮箱配置
```c
typedef struct {
    uint16_t mailbox_size;       // 邮箱大小 (128字节)
    uint16_t mailbox_offset_out; // 输出邮箱偏移 (0x1000)
    uint16_t mailbox_offset_in;  // 输入邮箱偏移 (0x1080)
    bool coe_enable;             // CANopen邮箱使能
    bool foe_enable;             // 文件传输邮箱使能
    bool soe_enable;             // Servo邮箱使能
    bool eoe_enable;             // 以太网邮箱使能
} mailbox_config_t;
```

### 8.2 CANopen over EtherCAT (CoE)
```c
// CoE服务定义
typedef enum {
    COE_SDO_DOWNLOAD = 0x01,     // SDO下载
    COE_SDO_UPLOAD = 0x02,       // SDO上传
    COE_SDO_INFO = 0x03,         // SDO信息
    COE_EMERGENCY = 0x04,        // 紧急消息
} coe_service_t;

// CoE消息结构
typedef struct {
    uint8_t header;              // CoE头部
    uint16_t index;              // 对象字典索引
    uint8_t subindex;            // 子索引
    uint8_t data[64];            // 数据
    uint8_t data_len;            // 数据长度
} coe_message_t;
```

## 9. 错误处理

### 9.1 错误代码定义
```c
typedef enum {
    EC_ERR_NONE = 0x0000,                // 无错误
    EC_ERR_PDO_SIZE_MISMATCH = 0x8001,   // PDO大小不匹配
    EC_ERR_SYNC_TIMEOUT = 0x8002,        // 同步超时
    EC_ERR_WATCHDOG_TIMEOUT = 0x8003,    // 看门狗超时
    EC_ERR_INVALID_STATE = 0x8004,       // 无效状态
    EC_ERR_MAILBOX_OVERFLOW = 0x8005,    // 邮箱溢出
    EC_ERR_COMMUNICATION_FAULT = 0x8006, // 通信故障
    EC_ERR_PARAMETER_ERROR = 0x8007,     // 参数错误
    EC_ERR_HARDWARE_FAULT = 0x8008,      // 硬件故障
} ethercat_error_code_t;
```

### 9.2 错误恢复机制
```c
typedef struct {
    ethercat_error_code_t error_code;
    uint32_t error_count;
    uint32_t last_error_time;
    bool auto_recovery_enable;
    uint16_t recovery_timeout;
    void (*recovery_handler)(void);
} error_recovery_t;
```

## 10. 性能监控

### 10.1 性能指标
```c
typedef struct {
    // 周期性能
    uint32_t cycle_count;        // 周期计数
    uint32_t cycle_time_avg;     // 平均周期时间 (μs)
    uint32_t cycle_time_max;     // 最大周期时间 (μs)
    uint32_t cycle_time_min;     // 最小周期时间 (μs)

    // 通信性能
    uint32_t pdo_tx_count;       // PDO发送计数
    uint32_t pdo_rx_count;       // PDO接收计数
    uint32_t mailbox_tx_count;   // 邮箱发送计数
    uint32_t mailbox_rx_count;   // 邮箱接收计数
    uint32_t error_count;        // 错误计数

    // 时钟性能
    int32_t dc_drift_ns;         // 时钟漂移 (ns)
    uint32_t sync_error_count;   // 同步错误计数

} ethercat_performance_t;
```

## 11. 实现优势

### 11.1 EtherCAT vs TCP对比
| 特性 | EtherCAT | TCP |
|------|----------|-----|
| 实时性 | 硬实时 (< 1ms) | 软实时 (10-100ms) |
| 确定性 | 高 | 中等 |
| 带宽利用率 | 高 (90%+) | 中等 (30-50%) |
| 拓扑结构 | 任意拓扑 | 星型 |
| 线缆需求 | 标准以太网线 | 标准以太网线 |
| 开发复杂度 | 高 | 中等 |
| 标准化程度 | IEC 61158 | RFC 793 |

### 11.2 应用场景选择
- **选择EtherCAT**：
  - 要求高精度实时控制 (< 1ms)
  - 多设备协同控制
  - 分布式时钟同步要求
  - 工业标准化要求

- **选择TCP**：
  - 实时性要求不高 (> 10ms)
  - 简单监控应用
  - 现有TCP基础设施
  - 开发周期要求短

---

*版本: v1.0*
*最后更新: 2025-09-28*