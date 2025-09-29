# TCP上位机通信命令详细设计

## 1. 协议基础定义

### 1.1 帧结构（复用现有定义）
```c
typedef struct {
    uint16_t sync_word;       // 同步字 0xAA55
    uint16_t length;          // 帧长度(包含帧头)
    uint8_t  cmd;             // 命令字
    uint8_t  index;           // 索引/子命令
    uint16_t checksum;        // 校验和
} __attribute__((packed)) tcp_frame_header_t;

typedef struct {
    tcp_frame_header_t header;
    uint8_t data[];           // 可变长度数据
} __attribute__((packed)) tcp_protocol_frame_t;
```

### 1.2 命令分类
```c
// 主命令分类
#define CMD_CATEGORY_MASK    0xF0
#define CMD_DEVICE_INFO      0x10    // 设备信息类
#define CMD_SENSOR_DATA      0x20    // 传感器数据类
#define CMD_CONTROL_CMD      0x30    // 控制命令类
#define CMD_CONFIG_PARAM     0x40    // 配置参数类
#define CMD_SYSTEM_CTRL      0x50    // 系统控制类
#define CMD_ALARM_STATUS     0x60    // 告警状态类
#define CMD_FIRMWARE_UPG     0x70    // 固件升级类
#define CMD_DEBUG_TEST       0x80    // 调试测试类
```

## 2. 下行命令（上位机 → 控制器）

### 2.1 设备信息类命令 (0x10-0x1F)

#### 0x10 - 读取设备基本信息
```c
// 请求：无数据
// 响应：
typedef struct {
    char device_name[32];     // 设备名称
    char hardware_version[16]; // 硬件版本
    char firmware_version[16]; // 固件版本
    char manufacture_date[16]; // 生产日期
    uint32_t serial_number;   // 序列号
    uint16_t device_type;     // 设备类型
    uint16_t channel_count;   // 通道数量
} device_info_t;
```

#### 0x11 - 读取设备运行状态
```c
// 请求：无数据
// 响应：
typedef struct {
    uint32_t uptime_seconds;  // 运行时间(秒)
    uint8_t  system_state;    // 系统状态 (0:停止 1:运行 2:故障 3:维护)
    uint8_t  cpu_usage;       // CPU使用率(%)
    uint16_t memory_usage;    // 内存使用(KB)
    uint16_t temperature;     // MCU温度(°C*10)
    uint8_t  comm_status;     // 通信状态位
} device_status_t;
```

### 2.2 传感器数据类命令 (0x20-0x2F)

#### 0x20 - 读取所有传感器数据
```c
// 请求：无数据
// 响应：
typedef struct {
    // 温度数据 (°C*100)
    int16_t temp_values[3];   // 3路温度
    uint8_t temp_status[3];   // 温度状态 (0:正常 1:断线 2:超限 3:故障)

    // 压力数据 (kPa*100)
    int16_t pressure_values[4]; // 4路压力
    uint8_t pressure_status[4]; // 压力状态

    // 液位数据 (mm*10)
    uint16_t level_values[4];   // 4路液位(3路传感器+1路模拟量)
    uint8_t level_status[4];    // 液位状态

    // 流量数据 (L/min*100)
    uint16_t flow_value;        // 流量值
    uint8_t flow_status;        // 流量状态

    uint32_t timestamp;         // 时间戳
} sensor_data_all_t;
```

#### 0x21 - 读取指定传感器数据
```c
// 请求：
typedef struct {
    uint8_t sensor_type;      // 传感器类型 (1:温度 2:压力 3:液位 4:流量)
    uint8_t channel;          // 通道号 (0-3)
} sensor_read_req_t;

// 响应：
typedef struct {
    int32_t raw_value;        // 原始值
    int32_t calibrated_value; // 标定值
    uint8_t status;           // 状态
    uint32_t timestamp;       // 时间戳
} sensor_data_single_t;
```

#### 0x22 - 传感器标定
```c
// 请求：
typedef struct {
    uint8_t sensor_type;      // 传感器类型
    uint8_t channel;          // 通道号
    uint8_t calib_point;      // 标定点 (0:零点 1:满量程)
    int32_t reference_value;  // 参考值
} sensor_calib_req_t;

// 响应：错误码
```

### 2.3 控制命令类 (0x30-0x3F)

#### 0x30 - 设置控制模式
```c
// 请求：
typedef struct {
    uint8_t control_mode;     // 控制模式 (0:手动 1:自动 2:维护)
    uint8_t loop_id;          // 控制回路ID (0xFF表示全部)
} control_mode_req_t;

// 响应：错误码
```

#### 0x31 - 设置设定值
```c
// 请求：
typedef struct {
    uint8_t loop_id;          // 控制回路ID
    int32_t setpoint;         // 设定值 (根据类型放大相应倍数)
} setpoint_req_t;

// 响应：错误码
```

#### 0x32 - 手动控制输出
```c
// 请求：
typedef struct {
    uint8_t output_channel;   // 输出通道 (0-8)
    uint8_t output_type;      // 输出类型 (0:开关 1:PWM 2:模拟量)
    uint32_t output_value;    // 输出值
} manual_output_req_t;

// 响应：错误码
```

#### 0x33 - 读取控制状态
```c
// 请求：
typedef struct {
    uint8_t loop_id;          // 控制回路ID (0xFF表示全部)
} control_status_req_t;

// 响应：
typedef struct {
    uint8_t loop_count;       // 回路数量
    struct {
        uint8_t loop_id;      // 回路ID
        uint8_t mode;         // 控制模式
        int32_t setpoint;     // 设定值
        int32_t process_value; // 过程值
        int32_t output_value; // 输出值
        uint8_t status;       // 状态
    } loops[8];
} control_status_resp_t;
```

### 2.4 配置参数类命令 (0x40-0x4F)

#### 0x40 - PID参数配置
```c
// 请求：
typedef struct {
    uint8_t loop_id;          // 控制回路ID
    float kp;                 // 比例系数
    float ki;                 // 积分系数
    float kd;                 // 微分系数
    float output_min;         // 输出下限
    float output_max;         // 输出上限
    uint8_t reverse_action;   // 反作用 (0:正作用 1:反作用)
} pid_config_req_t;

// 响应：错误码
```

#### 0x41 - 读取PID参数
```c
// 请求：
typedef struct {
    uint8_t loop_id;          // 控制回路ID
} pid_read_req_t;

// 响应：pid_config_req_t结构
```

#### 0x42 - 安全参数配置
```c
// 请求：
typedef struct {
    uint8_t sensor_type;      // 传感器类型
    uint8_t channel;          // 通道号
    int32_t alarm_high;       // 高限报警
    int32_t alarm_low;        // 低限报警
    int32_t shutdown_high;    // 高限停机
    int32_t shutdown_low;     // 低限停机
    uint16_t delay_time;      // 延时时间(ms)
} safety_config_req_t;

// 响应：错误码
```

#### 0x43 - 通信参数配置
```c
// 请求：
typedef struct {
    uint32_t ip_address;      // IP地址
    uint16_t tcp_port;        // TCP端口
    uint16_t ethercat_addr;   // EtherCAT地址
    uint32_t baudrate;        // 波特率
    uint8_t comm_mode;        // 通信模式 (0:TCP 1:EtherCAT)
} comm_config_req_t;

// 响应：错误码
```

#### 0x44 - LED指示配置
```c
// 请求：
typedef struct {
    uint8_t led_id;           // LED编号
    uint8_t function;         // 功能定义
    uint16_t blink_period;    // 闪烁周期(ms)
    uint8_t brightness;       // 亮度 (0-100)
} led_config_req_t;

// 响应：错误码
```

### 2.5 系统控制类命令 (0x50-0x5F)

#### 0x50 - 系统复位
```c
// 请求：
typedef struct {
    uint8_t reset_type;       // 复位类型 (0:软复位 1:硬复位 2:恢复出厂设置)
    uint32_t confirm_code;    // 确认码 (0x12345678)
} system_reset_req_t;

// 响应：确认信息
```

#### 0x51 - 参数保存/加载
```c
// 请求：
typedef struct {
    uint8_t operation;        // 操作 (0:保存 1:加载默认 2:加载备份)
    uint8_t param_type;       // 参数类型 (0:全部 1:PID 2:安全 3:通信)
} param_operation_req_t;

// 响应：错误码
```

#### 0x52 - 时间同步
```c
// 请求：
typedef struct {
    uint32_t unix_timestamp;  // Unix时间戳
    uint16_t milliseconds;    // 毫秒
} time_sync_req_t;

// 响应：确认信息
```

#### 0x53 - 系统自检
```c
// 请求：
typedef struct {
    uint8_t test_items;       // 测试项目位掩码
} self_test_req_t;

// 响应：
typedef struct {
    uint8_t test_result;      // 测试结果位掩码
    char test_report[128];    // 测试报告
} self_test_resp_t;
```

### 2.6 告警管理类命令 (0x60-0x6F)

#### 0x60 - 读取告警列表
```c
// 请求：
typedef struct {
    uint8_t alarm_type;       // 告警类型 (0:全部 1:活动 2:历史)
    uint16_t start_index;     // 起始索引
    uint16_t count;           // 数量
} alarm_list_req_t;

// 响应：
typedef struct {
    uint16_t total_count;     // 总数量
    uint16_t current_count;   // 当前返回数量
    struct {
        uint16_t alarm_id;    // 告警ID
        uint32_t timestamp;   // 时间戳
        uint8_t alarm_type;   // 告警类型
        uint8_t severity;     // 严重程度
        uint8_t status;       // 状态 (0:活动 1:确认 2:清除)
        char description[64]; // 描述
    } alarms[16];
} alarm_list_resp_t;
```

#### 0x61 - 告警确认
```c
// 请求：
typedef struct {
    uint16_t alarm_id;        // 告警ID (0xFFFF表示全部确认)
    uint32_t confirm_time;    // 确认时间
    char operator_id[16];     // 操作员ID
} alarm_ack_req_t;

// 响应：错误码
```

### 2.7 固件升级类命令 (0x70-0x7F)

#### 0x70 - 开始固件升级
```c
// 请求：
typedef struct {
    uint32_t firmware_size;   // 固件大小
    uint32_t firmware_crc;    // 固件CRC
    char version[16];         // 版本信息
} firmware_start_req_t;

// 响应：
typedef struct {
    uint8_t result;           // 结果 (0:成功 1:失败)
    uint16_t packet_size;     // 数据包大小
    uint32_t session_id;      // 会话ID
} firmware_start_resp_t;
```

#### 0x71 - 发送固件数据
```c
// 请求：
typedef struct {
    uint32_t session_id;      // 会话ID
    uint16_t packet_seq;      // 包序号
    uint16_t data_len;        // 数据长度
    uint8_t data[1024];       // 固件数据
} firmware_data_req_t;

// 响应：
typedef struct {
    uint8_t result;           // 结果
    uint16_t next_seq;        // 下一个序号
} firmware_data_resp_t;
```

#### 0x72 - 完成固件升级
```c
// 请求：
typedef struct {
    uint32_t session_id;      // 会话ID
    uint32_t total_crc;       // 总CRC
} firmware_finish_req_t;

// 响应：
typedef struct {
    uint8_t result;           // 结果
    uint8_t need_reset;       // 是否需要复位
} firmware_finish_resp_t;
```

## 3. 上行命令（控制器 → 上位机）

### 3.1 主动上报类命令 (0x90-0x9F)

#### 0x90 - 实时数据上报
```c
// 主动发送（周期性）：
typedef struct {
    uint32_t timestamp;       // 时间戳

    // 关键传感器数据
    int16_t temp_values[3];   // 温度值
    int16_t pressure_values[4]; // 压力值
    uint16_t level_values[4]; // 液位值
    uint16_t flow_value;      // 流量值

    // 控制输出状态
    uint8_t output_status[9]; // 9路输出状态

    // 系统状态
    uint8_t system_state;     // 系统状态
    uint16_t alarm_word;      // 告警字
} realtime_data_report_t;
```

#### 0x91 - 告警事件上报
```c
// 事件触发发送：
typedef struct {
    uint16_t alarm_id;        // 告警ID
    uint32_t timestamp;       // 发生时间
    uint8_t alarm_type;       // 告警类型
    uint8_t severity;         // 严重程度
    uint8_t source_channel;   // 源通道
    int32_t current_value;    // 当前值
    int32_t threshold_value;  // 阈值
    char description[64];     // 告警描述
} alarm_event_report_t;
```

#### 0x92 - 系统状态变化上报
```c
// 状态变化触发：
typedef struct {
    uint8_t old_state;        // 原状态
    uint8_t new_state;        // 新状态
    uint32_t timestamp;       // 变化时间
    uint16_t reason_code;     // 原因码
    char reason_text[32];     // 原因描述
} state_change_report_t;
```

#### 0x93 - 控制回路状态上报
```c
// 定期上报：
typedef struct {
    uint8_t loop_id;          // 回路ID
    uint8_t mode;             // 控制模式
    int32_t setpoint;         // 设定值
    int32_t process_value;    // 过程值
    int32_t output_value;     // 输出值
    float pid_p_term;         // P项
    float pid_i_term;         // I项
    float pid_d_term;         // D项
    uint8_t status;           // 状态
    uint32_t timestamp;       // 时间戳
} loop_status_report_t;
```

## 4. 错误码定义

```c
typedef enum {
    ERR_OK = 0x00,                    // 成功
    ERR_INVALID_CMD = 0x01,           // 无效命令
    ERR_INVALID_PARAM = 0x02,         // 无效参数
    ERR_DEVICE_BUSY = 0x03,           // 设备忙
    ERR_TIMEOUT = 0x04,               // 超时
    ERR_CHECKSUM = 0x05,              // 校验错误
    ERR_PERMISSION = 0x06,            // 权限不足
    ERR_NOT_SUPPORTED = 0x07,         // 不支持
    ERR_HARDWARE_FAULT = 0x08,        // 硬件故障
    ERR_SENSOR_FAULT = 0x09,          // 传感器故障
    ERR_ACTUATOR_FAULT = 0x0A,        // 执行器故障
    ERR_COMMUNICATION_FAULT = 0x0B,   // 通信故障
    ERR_SAFETY_VIOLATION = 0x0C,      // 安全违规
    ERR_CONFIG_INVALID = 0x0D,        // 配置无效
    ERR_CALIBRATION_FAIL = 0x0E,      // 标定失败
    ERR_FIRMWARE_ERROR = 0x0F,        // 固件错误
    ERR_MEMORY_FULL = 0x10,           // 内存满
    ERR_FILE_NOT_FOUND = 0x11,        // 文件未找到
    ERR_UPGRADE_FAIL = 0x12,          // 升级失败
    ERR_SYSTEM_OVERLOAD = 0x13,       // 系统过载
} tcp_error_code_t;
```

## 5. 通信时序要求

### 5.1 实时性要求
- 实时数据上报：100ms周期
- 关键告警响应：≤50ms
- 控制命令响应：≤20ms
- 配置命令响应：≤200ms

### 5.2 可靠性机制
- 命令确认机制
- 超时重传
- 心跳检测
- 断线重连

### 5.3 数据完整性
- CRC校验
- 序列号验证
- 时间戳同步
- 数据缓冲

---

*版本: v1.0*
*最后更新: 2025-09-28*