# 墨路控制系统 TCP 通信协议设计文档

## 1. 概述

### 1.1 文档说明
本文档定义了墨路控制系统的TCP通信协议，用于上位机与GD32F427固件之间的数据交互。TCP协议与EtherCAT协议为二选一的通信方案，提供更灵活的网络配置和跨平台兼容性。

### 1.2 系统信息
- **设备名称**: 墨路控制系统
- **制造商**: 待定
- **设备类型**: 过程控制设备
- **通信协议**: TCP/IP
- **主控芯片**: GD32F427
- **网络协议栈**: lwIP

### 1.3 协议特点
- **通信方式**: TCP/IP (面向连接，可靠传输)
- **默认端口**: 8888
- **字节序**: 小端序 (Little-Endian)
- **数据编码**: 二进制 + 定点数
- **校验方式**: CRC16-CCITT
- **最大帧长**: 1024 bytes

---

## 2. 协议帧结构

### 2.1 基础帧格式
```
+--------+--------+--------+--------+--------+--------+--------+--------+
| Sync   | Sync   | Length | Length | Cmd    | Index  | Data   | ...    |
| [0]    | [1]    | [0]    | [1]    |        |        | [0]    |        |
+--------+--------+--------+--------+--------+--------+--------+--------+
| ...    | CRC    | CRC    |
| Data   | [0]    | [1]    |
+--------+--------+--------+
```

### 2.2 帧头定义
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 同步字: 0xAA55
    uint16_t length;          // 帧长度(包含帧头+数据+CRC，不含同步字)
    uint8_t  cmd;             // 命令字
    uint8_t  index;           // 索引/子命令
    // uint8_t data[];        // 数据段 (长度由length决定)
    // uint16_t crc;          // CRC16校验 (放在数据段末尾)
} tcp_frame_header_t;
```

### 2.3 字段说明
| 字段 | 偏移 | 字节数 | 说明 |
|------|------|--------|------|
| sync_word | 0 | 2 | 固定为0xAA55，用于帧同步识别 |
| length | 2 | 2 | 帧长度 = sizeof(header) - 2 + data_len + 2(CRC) |
| cmd | 4 | 1 | 命令码 (见命令定义表) |
| index | 5 | 1 | 索引/子索引，用于多对象访问 |
| data | 6 | N | 数据段，长度由命令决定 |
| crc | 6+N | 2 | CRC16校验，覆盖从length到data末尾 |

**示例帧**:
```
AA 55 08 00 01 00 01 00 XX XX
```
解析:
- `AA 55`: 同步字
- `08 00`: 长度 = 8 (4字节header + 2字节数据 + 2字节CRC)
- `01`: 命令 = 读取传感器
- `00`: 索引 = 温度传感器1
- `01 00`: 数据 = 传感器ID
- `XX XX`: CRC16

---

## 3. 命令定义

### 3.1 命令码表
| 命令码 | 命令名称 | 方向 | 说明 |
|-------|---------|------|------|
| 0x01 | 读取传感器数据 | 上位机→固件 | 读取单个或多个传感器当前值 |
| 0x02 | 写入设定值 | 上位机→固件 | 设置控制回路目标值 |
| 0x03 | 读取状态 | 上位机→固件 | 读取系统运行状态 |
| 0x04 | 写入控制命令 | 上位机→固件 | 控制执行器动作 |
| 0x05 | 读取配置参数 | 上位机→固件 | 读取PID/传感器配置 |
| 0x06 | 写入配置参数 | 上位机→固件 | 修改系统配置参数 |
| 0x07 | 告警确认 | 上位机→固件 | 确认并清除告警 |
| 0x08 | 系统复位 | 上位机→固件 | 软件复位系统 |
| 0x09 | 读取历史数据 | 上位机→固件 | 读取存储的历史记录 |
| 0x0A | 参数保存 | 上位机→固件 | 保存参数到Flash |
| 0x0B | 参数加载 | 上位机→固件 | 从Flash加载参数 |
| 0x0C | 自检命令 | 上位机→固件 | 启动系统自检 |
| 0x81 | 传感器数据响应 | 固件→上位机 | 响应0x01命令 |
| 0x82 | 设定值响应 | 固件→上位机 | 响应0x02命令 |
| 0x83 | 状态响应 | 固件→上位机 | 响应0x03命令 |
| 0x84 | 控制响应 | 固件→上位机 | 响应0x04命令 |
| 0x85 | 配置读取响应 | 固件→上位机 | 响应0x05命令 |
| 0x86 | 配置写入响应 | 固件→上位机 | 响应0x06命令 |
| 0x87 | 告警响应 | 固件→上位机 | 响应0x07命令 |
| 0x88 | 复位响应 | 固件→上位机 | 响应0x08命令 |
| 0x89 | 历史数据响应 | 固件→上位机 | 响应0x09命令 |
| 0x8A | 参数保存响应 | 固件→上位机 | 响应0x0A命令 |
| 0x8B | 参数加载响应 | 固件→上位机 | 响应0x0B命令 |
| 0x8C | 自检响应 | 固件→上位机 | 响应0x0C命令 |
| 0xF0 | 主动上报数据 | 固件→上位机 | 固件主动推送实时数据 |
| 0xF1 | 告警主动上报 | 固件→上位机 | 告警产生时主动推送 |
| 0xFF | 错误响应 | 固件→上位机 | 命令执行失败 |

### 3.2 错误码定义
| 错误码 | 名称 | 说明 |
|-------|------|------|
| 0x00 | ERR_OK | 成功 |
| 0x01 | ERR_INVALID_CMD | 无效命令 |
| 0x02 | ERR_INVALID_PARAM | 无效参数/索引 |
| 0x03 | ERR_DEVICE_BUSY | 设备忙 |
| 0x04 | ERR_TIMEOUT | 操作超时 |
| 0x05 | ERR_CHECKSUM | CRC校验错误 |
| 0x06 | ERR_PERMISSION | 权限不足/参数锁定 |
| 0x07 | ERR_OUT_OF_RANGE | 参数超出范围 |
| 0x08 | ERR_DEVICE_FAULT | 设备故障状态 |
| 0x09 | ERR_NOT_READY | 设备未就绪 |
| 0x0A | ERR_FLASH_ERROR | Flash读写错误 |

---

## 4. 数据对象定义

### 4.1 传感器数据索引
| Index | 名称 | 类型 | 单位 | 分辨率 | 说明 |
|-------|------|------|------|--------|------|
| 0x00 | 温度传感器1 | INT16 | °C | 0.1 | FTT518 Pt100 |
| 0x01 | 温度传感器2 | INT16 | °C | 0.1 | FTT518 Pt100 |
| 0x02 | 温度传感器3 | INT16 | °C | 0.1 | FTT518 Pt100 |
| 0x03 | 压力传感器1 | UINT16 | kPa | 0.1 | HP10MY |
| 0x04 | 压力传感器2 | UINT16 | kPa | 0.1 | HP10MY |
| 0x05 | 压力传感器3 | UINT16 | kPa | 0.1 | HP10MY |
| 0x06 | 压力传感器4 | UINT16 | kPa | 0.1 | HP10MY |
| 0x07 | 液位传感器1 | UINT16 | mm | 0.1 | FRD8061 |
| 0x08 | 液位传感器2 | UINT16 | mm | 0.1 | FRD8061 |
| 0x09 | 液位传感器3 | UINT16 | mm | 0.1 | FRD8061 |
| 0x0A | 模拟液位传感器 | UINT16 | mm | 0.1 | 模拟量 |
| 0x0B | 流量传感器 | UINT16 | L/min | 0.01 | I2C流量计 |
| 0x0C | 液位开关状态 | UINT16 | - | - | 14路液位开关位域 |
| 0xFF | 所有传感器 | - | - | - | 批量读取 |

### 4.2 设定值对象索引
| Index | 名称 | 类型 | 单位 | 分辨率 | 说明 |
|-------|------|------|------|--------|------|
| 0x00 | 温度设定值1 | INT16 | °C | 0.1 | 温度回路1目标 |
| 0x01 | 温度设定值2 | INT16 | °C | 0.1 | 温度回路2目标 |
| 0x02 | 温度设定值3 | INT16 | °C | 0.1 | 温度回路3目标 |
| 0x03 | 压力设定值1 | UINT16 | kPa | 0.1 | 压力回路1目标 |
| 0x04 | 压力设定值2 | UINT16 | kPa | 0.1 | 压力回路2目标 |
| 0x05 | 压力设定值3 | UINT16 | kPa | 0.1 | 压力回路3目标 |
| 0x06 | 压力设定值4 | UINT16 | kPa | 0.1 | 压力回路4目标 |
| 0x07 | 液位设定值1 | UINT16 | mm | 0.1 | 液位回路1目标 |
| 0x08 | 液位设定值2 | UINT16 | mm | 0.1 | 液位回路2目标 |
| 0x09 | 液位设定值3 | UINT16 | mm | 0.1 | 液位回路3目标 |
| 0x0A | 流量设定值 | UINT16 | L/min | 0.01 | 流量回路目标 |
| 0x0B | 调速泵1速度 | UINT16 | % | 0.1 | 泵1转速(0~100%) |
| 0x0C | 调速泵2速度 | UINT16 | % | 0.1 | 泵2转速(0~100%) |

### 4.3 执行器控制对象索引
| Index | 名称 | 类型 | 说明 |
|-------|------|------|------|
| 0x00 | 电磁阀1 | UINT8 | 0=关闭, 1=打开 |
| 0x01 | 电磁阀2 | UINT8 | 0=关闭, 1=打开 |
| 0x02 | 加热器1 | UINT8 | 0=关闭, 1=开启 |
| 0x03 | 加热器2 | UINT8 | 0=关闭, 1=开启 |
| 0x04 | 加热器3 | UINT8 | 0=关闭, 1=开启 |
| 0x05 | 直流泵1 | UINT8 | 0=关闭, 1=启动 |
| 0x06 | 直流泵2 | UINT8 | 0=关闭, 1=启动 |
| 0x07 | 调速泵1使能 | UINT8 | 0=禁止, 1=使能 |
| 0x08 | 调速泵2使能 | UINT8 | 0=禁止, 1=使能 |
| 0x10 | 控制模式 | UINT8 | 0=手动, 1=自动 |
| 0x11 | 系统使能 | UINT8 | 0=停止, 1=运行 |
| 0x12 | 紧急停止 | UINT8 | 1=触发紧急停止 |

### 4.4 配置参数对象索引
| Index | Sub | 名称 | 类型 | 单位 | 说明 |
|-------|-----|------|------|------|------|
| 0x00 | 0-2 | 温度传感器1配置 | - | - | 标定系数/偏移/滤波 |
| 0x01 | 0-2 | 温度传感器2配置 | - | - | - |
| 0x02 | 0-2 | 温度传感器3配置 | - | - | - |
| 0x10 | 0-5 | 温度PID1参数 | - | - | Kp/Ki/Kd/上限/下限/周期 |
| 0x11 | 0-5 | 温度PID2参数 | - | - | - |
| 0x12 | 0-5 | 温度PID3参数 | - | - | - |
| 0x20 | 0-5 | 压力PID1参数 | - | - | - |
| 0x21 | 0-5 | 压力PID2参数 | - | - | - |
| 0x30 | 0-3 | 安全限值配置 | - | - | 温度/压力/液位上下限 |
| 0x40 | 0-7 | 网络配置 | - | - | IP/掩码/网关/端口 |
| 0x50 | 0-3 | 系统参数 | - | - | 设备ID/名称/版本 |

---

## 5. 命令详细说明

### 5.1 读取传感器数据 (0x01)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 8
    uint8_t  cmd;             // 0x01
    uint8_t  index;           // 传感器索引 (0x00~0xFF)
    uint16_t crc;
} cmd_read_sensor_req_t;
```

**响应帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 12
    uint8_t  cmd;             // 0x81
    uint8_t  index;           // 传感器索引
    uint8_t  error_code;      // 错误码 (0x00=成功)
    uint8_t  status;          // 传感器状态 (bit0=有效, bit1=超限)
    int16_t  value;           // 传感器值 (定点数)
    uint32_t timestamp;       // 时间戳 (ms)
    uint16_t crc;
} cmd_read_sensor_resp_t;
```

**批量读取 (index=0xFF)**:
```c
// 响应帧包含所有传感器数据
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;          // 动态长度
    uint8_t  cmd;             // 0x81
    uint8_t  index;           // 0xFF
    uint8_t  error_code;
    uint8_t  sensor_count;    // 传感器数量
    struct {
        uint8_t status;
        int16_t value;
    } sensors[];              // 传感器数组
    uint32_t timestamp;
    uint16_t crc;
} cmd_read_all_sensors_resp_t;
```

### 5.2 写入设定值 (0x02)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 10
    uint8_t  cmd;             // 0x02
    uint8_t  index;           // 设定值对象索引
    int16_t  setpoint;        // 设定值 (定点数)
    uint16_t crc;
} cmd_write_setpoint_req_t;
```

**响应帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 10
    uint8_t  cmd;             // 0x82
    uint8_t  index;
    uint8_t  error_code;      // 0x00=成功
    uint8_t  reserved;
    int16_t  actual_value;    // 实际设置的值 (可能经过限幅)
    uint16_t crc;
} cmd_write_setpoint_resp_t;
```

### 5.3 读取状态 (0x03)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 8
    uint8_t  cmd;             // 0x03
    uint8_t  index;           // 0x00=系统状态, 0x01=执行器状态
    uint16_t crc;
} cmd_read_status_req_t;
```

**响应帧 (系统状态, index=0x00)**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;          // 28
    uint8_t  cmd;             // 0x83
    uint8_t  index;           // 0x00
    uint8_t  error_code;
    uint8_t  run_state;       // 0=初始化,1=就绪,2=运行,3=停止,4=告警,5=故障
    uint16_t status_word;     // 状态字位域
    uint16_t alarm_word1;     // 告警字1
    uint16_t alarm_word2;     // 告警字2
    uint16_t cpu_usage;       // CPU使用率 (0.01%)
    uint32_t free_heap;       // 剩余堆内存 (bytes)
    uint32_t uptime;          // 运行时间 (秒)
    uint32_t timestamp;       // 时间戳 (ms)
    uint16_t crc;
} cmd_read_system_status_resp_t;
```

**状态字位定义**:
```
Bit 0-2:  运行状态 (3 bits)
Bit 3:    安全模式 (1=激活)
Bit 4:    手动模式 (1=手动, 0=自动)
Bit 5:    配置有效 (1=已加载)
Bit 6:    自检通过 (1=OK)
Bit 7:    TCP通信正常 (1=正常)
Bit 8:    传感器故障 (1=存在故障)
Bit 9:    执行器故障 (1=存在故障)
Bit 10:   超温告警 (1=存在超温)
Bit 11:   超压告警 (1=存在超压)
Bit 12:   液位异常 (1=异常)
Bit 13:   通信超时 (1=超时)
Bit 14:   参数超限 (1=越界)
Bit 15:   系统错误 (1=系统级错误)
```

### 5.4 写入控制命令 (0x04)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 10
    uint8_t  cmd;             // 0x04
    uint8_t  index;           // 执行器对象索引
    uint8_t  control_value;   // 控制值 (0/1或百分比)
    uint8_t  control_mode;    // 0=手动, 1=自动
    uint16_t crc;
} cmd_write_control_req_t;
```

**响应帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;          // 10
    uint8_t  cmd;             // 0x84
    uint8_t  index;
    uint8_t  error_code;
    uint8_t  actual_state;    // 执行器实际状态
    uint16_t crc;
} cmd_write_control_resp_t;
```

### 5.5 读取配置参数 (0x05)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 9
    uint8_t  cmd;             // 0x05
    uint8_t  index;           // 配置对象索引
    uint8_t  sub_index;       // 子索引
    uint16_t crc;
} cmd_read_config_req_t;
```

**响应帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;          // 动态长度
    uint8_t  cmd;             // 0x85
    uint8_t  index;
    uint8_t  sub_index;
    uint8_t  error_code;
    uint8_t  data_type;       // 0=UINT8, 1=INT16, 2=UINT16, 3=INT32, 4=FLOAT32
    uint8_t  data_len;        // 数据长度
    uint8_t  data[];          // 配置数据
    uint16_t crc;
} cmd_read_config_resp_t;
```

### 5.6 写入配置参数 (0x06)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 动态长度
    uint8_t  cmd;             // 0x06
    uint8_t  index;           // 配置对象索引
    uint8_t  sub_index;       // 子索引
    uint8_t  data_type;       // 数据类型
    uint8_t  data_len;        // 数据长度
    uint8_t  data[];          // 配置数据
    uint16_t crc;
} cmd_write_config_req_t;
```

**响应帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;          // 9
    uint8_t  cmd;             // 0x86
    uint8_t  index;
    uint8_t  sub_index;
    uint8_t  error_code;      // 0x00=成功
    uint16_t crc;
} cmd_write_config_resp_t;
```

### 5.7 告警确认 (0x07)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 10
    uint8_t  cmd;             // 0x07
    uint8_t  index;           // 0x00=确认所有, 0x01=确认指定位
    uint16_t alarm_mask;      // 要确认的告警位掩码
    uint16_t crc;
} cmd_alarm_ack_req_t;
```

**响应帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;          // 12
    uint8_t  cmd;             // 0x87
    uint8_t  index;
    uint8_t  error_code;
    uint8_t  reserved;
    uint16_t remaining_alarms; // 剩余未确认的告警
    uint16_t crc;
} cmd_alarm_ack_resp_t;
```

### 5.8 系统复位 (0x08)

**请求帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 10
    uint8_t  cmd;             // 0x08
    uint8_t  index;           // 0x00=软复位, 0x01=恢复出厂
    uint16_t magic;           // 魔术字 0x5AA5 (防误触发)
    uint16_t crc;
} cmd_reset_req_t;
```

**响应帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;
    uint16_t length;          // 9
    uint8_t  cmd;             // 0x88
    uint8_t  index;
    uint8_t  error_code;      // 0x00=成功,即将复位
    uint16_t crc;
} cmd_reset_resp_t;
```

### 5.9 主动上报数据 (0xF0)

固件可以定期或事件触发主动推送实时数据到上位机。

**推送帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 动态长度
    uint8_t  cmd;             // 0xF0
    uint8_t  report_type;     // 0x00=实时数据, 0x01=状态变更
    // 实时数据 (report_type=0x00)
    struct {
        int16_t temp[3];      // 3路温度
        uint16_t pressure[4]; // 4路压力
        uint16_t level[3];    // 3路液位
        uint16_t flow;        // 流量
        uint16_t status_word;
        uint16_t alarm_word1;
        uint16_t alarm_word2;
    } realtime_data;
    uint32_t timestamp;
    uint16_t crc;
} cmd_report_data_t;
```

### 5.10 告警主动上报 (0xF1)

**推送帧**:
```c
typedef struct __attribute__((packed)) {
    uint16_t sync_word;       // 0xAA55
    uint16_t length;          // 20
    uint8_t  cmd;             // 0xF1
    uint8_t  alarm_level;     // 0=信息, 1=警告, 2=错误, 3=严重
    uint16_t alarm_code;      // 告警代码
    uint16_t alarm_word1;     // 当前告警字1
    uint16_t alarm_word2;     // 当前告警字2
    int16_t  related_value;   // 相关参数值
    uint32_t timestamp;       // 告警发生时间
    uint16_t crc;
} cmd_alarm_report_t;
```

---

## 6. 通信时序

### 6.1 连接建立流程
```
上位机                                固件
  |                                    |
  |--- TCP Connect (port 8888) ------>|
  |<--- TCP Accept --------------------|
  |                                    |
  |--- 0x03 读取状态 ----------------->|
  |<--- 0x83 状态响应 -----------------|
  |                                    |
  |--- 0x05 读取配置 ----------------->|
  |<--- 0x85 配置响应 -----------------|
  |                                    |
  [连接建立成功]
```

### 6.2 周期性数据交互
```
上位机                                固件
  |                                    |
  |--- 0x01 读取传感器(批量) --------->|
  |<--- 0x81 传感器数据 ---------------|
  |                                    |
  |--- 0x02 写入设定值 --------------->|
  |<--- 0x82 设定值响应 ---------------|
  |                                    |
  |<--- 0xF0 主动上报数据 -------------|
  |                                    |
  [每100ms~500ms循环]
```

### 6.3 告警处理流程
```
上位机                                固件
  |                                    |
  |                                    |--- 检测到告警 ---|
  |<--- 0xF1 告警上报 -----------------|
  |                                    |
  |--- 0x03 读取状态 ----------------->|
  |<--- 0x83 状态响应 (告警详情) ------|
  |                                    |
  [用户确认告警]
  |                                    |
  |--- 0x07 告警确认 ----------------->|
  |<--- 0x87 确认响应 -----------------|
```

### 6.4 超时与重连机制
- **请求超时**: 上位机发送请求后，若1秒内未收到响应，重发(最多3次)
- **心跳机制**: 上位机每5秒发送一次状态读取(0x03)，保持连接活跃
- **固件侧超时**: 若10秒内未收到任何有效命令，固件可主动断开连接
- **重连机制**: TCP连接断开后，上位机应每5秒尝试重连

---

## 7. 数据编码与解码

### 7.1 定点数编码 (与EtherCAT协议保持一致)

**温度数据** (INT16, 分辨率0.1°C):
```c
// 编码: 实际温度 → TCP值
int16_t temp_tcp = (int16_t)(actual_temp * 10.0f);

// 解码: TCP值 → 实际温度
float actual_temp = (float)temp_tcp / 10.0f;

// 示例: 25.3°C → 253 (0x00FD)
//       -10.5°C → -105 (0xFF97)
```

**压力数据** (UINT16, 分辨率0.1kPa):
```c
uint16_t pressure_tcp = (uint16_t)(actual_pressure * 10.0f);
float actual_pressure = (float)pressure_tcp / 10.0f;
```

**液位数据** (UINT16, 分辨率0.1mm):
```c
uint16_t level_tcp = (uint16_t)(actual_level * 10.0f);
float actual_level = (float)level_tcp / 10.0f;
```

**百分比数据** (UINT16, 分辨率0.1%):
```c
uint16_t percent_tcp = (uint16_t)(percent * 10.0f);
float percent = (float)percent_tcp / 10.0f;
```

### 7.2 浮点数编码 (配置参数)

配置参数(如PID系数)使用IEEE 754单精度浮点数(FLOAT32):
```c
// 直接以字节流传输
float kp_value = 2.5f;
memcpy(data_buffer, &kp_value, sizeof(float));
```

### 7.3 CRC16校验计算

采用CRC16-CCITT算法 (多项式: 0x1021, 初始值: 0xFFFF):
```c
uint16_t calculate_crc16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// 使用示例
// CRC覆盖从length字段到数据段末尾
uint16_t crc = calculate_crc16(&frame.length, frame.length);
frame.crc = crc;
```

### 7.4 字节序处理

协议采用**小端序** (Little-Endian)，与GD32F427本机字节序一致:
```c
// 示例: 将0x1234存储到缓冲区
uint16_t value = 0x1234;
buffer[0] = value & 0xFF;         // 0x34 (低字节)
buffer[1] = (value >> 8) & 0xFF;  // 0x12 (高字节)

// 读取
uint16_t parsed = buffer[0] | (buffer[1] << 8);
```

---

## 8. 固件实现示例

### 8.1 TCP服务器初始化 (app/tcp_server.c)

```c
#include "tcp_server.h"
#include "lwip/tcp.h"
#include "protocol.h"

#define TCP_SERVER_PORT 8888
#define TCP_RX_BUFFER_SIZE 1024
#define TCP_TX_BUFFER_SIZE 1024

typedef struct {
    struct tcp_pcb *pcb;
    uint8_t rx_buffer[TCP_RX_BUFFER_SIZE];
    uint16_t rx_index;
    uint8_t tx_buffer[TCP_TX_BUFFER_SIZE];
    bool connected;
    uint32_t last_activity;
} tcp_server_ctx_t;

static tcp_server_ctx_t g_tcp_ctx = {0};

// TCP连接回调
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    if (err != ERR_OK || newpcb == NULL) {
        return ERR_VAL;
    }

    // 保存连接
    g_tcp_ctx.pcb = newpcb;
    g_tcp_ctx.connected = true;
    g_tcp_ctx.rx_index = 0;
    g_tcp_ctx.last_activity = HAL_GetTick();

    // 设置回调
    tcp_arg(newpcb, &g_tcp_ctx);
    tcp_recv(newpcb, tcp_server_recv);
    tcp_err(newpcb, tcp_server_error);

    printf("TCP客户端已连接\n");
    return ERR_OK;
}

// 接收数据回调
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb,
                              struct pbuf *p, err_t err)
{
    tcp_server_ctx_t *ctx = (tcp_server_ctx_t *)arg;

    if (p == NULL) {
        // 连接关闭
        tcp_close(tpcb);
        ctx->connected = false;
        printf("TCP客户端断开连接\n");
        return ERR_OK;
    }

    // 拷贝数据到缓冲区
    if (ctx->rx_index + p->tot_len <= TCP_RX_BUFFER_SIZE) {
        pbuf_copy_partial(p, &ctx->rx_buffer[ctx->rx_index], p->tot_len, 0);
        ctx->rx_index += p->tot_len;
        ctx->last_activity = HAL_GetTick();
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    // 处理接收到的数据
    tcp_process_received_data(ctx);

    return ERR_OK;
}

// 错误回调
static void tcp_server_error(void *arg, err_t err)
{
    tcp_server_ctx_t *ctx = (tcp_server_ctx_t *)arg;
    ctx->connected = false;
    ctx->pcb = NULL;
    printf("TCP错误: %d\n", err);
}

// 初始化TCP服务器
void tcp_server_init(void)
{
    struct tcp_pcb *pcb;

    pcb = tcp_new();
    if (pcb == NULL) {
        printf("TCP服务器创建失败\n");
        return;
    }

    err_t err = tcp_bind(pcb, IP_ADDR_ANY, TCP_SERVER_PORT);
    if (err != ERR_OK) {
        tcp_close(pcb);
        printf("TCP端口绑定失败\n");
        return;
    }

    pcb = tcp_listen(pcb);
    tcp_accept(pcb, tcp_server_accept);

    printf("TCP服务器启动，端口: %d\n", TCP_SERVER_PORT);
}
```

### 8.2 协议解析与处理 (app/tcp_server.c)

```c
// 协议帧查找与验证
static bool find_valid_frame(tcp_server_ctx_t *ctx, uint16_t *frame_start, uint16_t *frame_len)
{
    // 查找同步字 0xAA55
    for (uint16_t i = 0; i < ctx->rx_index - 1; i++) {
        if (ctx->rx_buffer[i] == 0xAA && ctx->rx_buffer[i+1] == 0x55) {
            // 检查长度字段
            if (i + 4 <= ctx->rx_index) {
                uint16_t length = ctx->rx_buffer[i+2] | (ctx->rx_buffer[i+3] << 8);

                // 检查完整帧是否接收完毕
                if (i + 2 + length <= ctx->rx_index) {
                    // 验证CRC
                    uint16_t calc_crc = calculate_crc16(&ctx->rx_buffer[i+2], length - 2);
                    uint16_t recv_crc = ctx->rx_buffer[i+2+length-2] |
                                       (ctx->rx_buffer[i+2+length-1] << 8);

                    if (calc_crc == recv_crc) {
                        *frame_start = i;
                        *frame_len = length + 2;  // 包含同步字
                        return true;
                    } else {
                        printf("CRC校验失败\n");
                    }
                }
            }
        }
    }
    return false;
}

// 处理接收数据
static void tcp_process_received_data(tcp_server_ctx_t *ctx)
{
    uint16_t frame_start, frame_len;

    while (find_valid_frame(ctx, &frame_start, &frame_len)) {
        // 解析帧头
        tcp_frame_header_t *header = (tcp_frame_header_t *)&ctx->rx_buffer[frame_start];
        uint8_t *data = &ctx->rx_buffer[frame_start + sizeof(tcp_frame_header_t)];

        // 根据命令码分发处理
        switch (header->cmd) {
            case 0x01:  // 读取传感器
                tcp_handle_read_sensor(ctx, header, data);
                break;
            case 0x02:  // 写入设定值
                tcp_handle_write_setpoint(ctx, header, data);
                break;
            case 0x03:  // 读取状态
                tcp_handle_read_status(ctx, header, data);
                break;
            case 0x04:  // 写入控制命令
                tcp_handle_write_control(ctx, header, data);
                break;
            case 0x05:  // 读取配置
                tcp_handle_read_config(ctx, header, data);
                break;
            case 0x06:  // 写入配置
                tcp_handle_write_config(ctx, header, data);
                break;
            case 0x07:  // 告警确认
                tcp_handle_alarm_ack(ctx, header, data);
                break;
            case 0x08:  // 系统复位
                tcp_handle_reset(ctx, header, data);
                break;
            default:
                // 未知命令，返回错误
                tcp_send_error_response(ctx, header->cmd, ERR_INVALID_CMD);
                break;
        }

        // 移除已处理的帧
        memmove(ctx->rx_buffer, &ctx->rx_buffer[frame_start + frame_len],
                ctx->rx_index - frame_start - frame_len);
        ctx->rx_index -= (frame_start + frame_len);
    }
}

// 读取传感器处理
static void tcp_handle_read_sensor(tcp_server_ctx_t *ctx,
                                    tcp_frame_header_t *header, uint8_t *data)
{
    sensor_context_t *sensors = get_sensor_context();

    if (header->index == 0xFF) {
        // 批量读取所有传感器
        cmd_read_all_sensors_resp_t *resp = (cmd_read_all_sensors_resp_t *)ctx->tx_buffer;
        resp->sync_word = 0xAA55;
        resp->cmd = 0x81;
        resp->index = 0xFF;
        resp->error_code = ERR_OK;
        resp->sensor_count = 13;  // 3温度+4压力+3液位+1模拟液位+1流量+1开关

        uint16_t offset = 0;

        // 温度传感器
        for (int i = 0; i < 3; i++) {
            resp->sensors[offset].status = sensors->sensors[i].valid ? 0x01 : 0x00;
            resp->sensors[offset].value = (int16_t)(sensors->temp_values[i] * 10.0f);
            offset++;
        }

        // 压力传感器
        for (int i = 0; i < 4; i++) {
            resp->sensors[offset].status = sensors->sensors[3+i].valid ? 0x01 : 0x00;
            resp->sensors[offset].value = (uint16_t)(sensors->pressure_values[i] * 10.0f);
            offset++;
        }

        // 液位传感器
        for (int i = 0; i < 4; i++) {
            resp->sensors[offset].status = sensors->sensors[7+i].valid ? 0x01 : 0x00;
            resp->sensors[offset].value = (uint16_t)(sensors->level_values[i] * 10.0f);
            offset++;
        }

        // 流量传感器
        resp->sensors[offset].status = sensors->sensors[11].valid ? 0x01 : 0x00;
        resp->sensors[offset].value = (uint16_t)(sensors->flow_value * 100.0f);
        offset++;

        // 液位开关 (作为单独一项)
        resp->sensors[offset].status = 0x01;
        resp->sensors[offset].value = get_level_switch_status();

        resp->timestamp = HAL_GetTick();
        resp->length = sizeof(cmd_read_all_sensors_resp_t) - 2 +
                       resp->sensor_count * 3;  // 每个传感器3字节

        // 计算CRC
        uint16_t crc = calculate_crc16((uint8_t*)&resp->length, resp->length - 2);
        memcpy(&ctx->tx_buffer[resp->length], &crc, 2);

        // 发送响应
        tcp_write(ctx->pcb, ctx->tx_buffer, resp->length + 2, TCP_WRITE_FLAG_COPY);
        tcp_output(ctx->pcb);

    } else {
        // 读取单个传感器
        cmd_read_sensor_resp_t *resp = (cmd_read_sensor_resp_t *)ctx->tx_buffer;
        resp->sync_word = 0xAA55;
        resp->length = sizeof(cmd_read_sensor_resp_t) - 2;
        resp->cmd = 0x81;
        resp->index = header->index;
        resp->timestamp = HAL_GetTick();

        if (header->index < SENSOR_COUNT) {
            resp->error_code = ERR_OK;
            resp->status = sensors->sensors[header->index].valid ? 0x01 : 0x00;
            resp->value = (int16_t)(sensors->sensors[header->index].calibrated_value * 10.0f);
        } else {
            resp->error_code = ERR_INVALID_PARAM;
            resp->status = 0x00;
            resp->value = 0;
        }

        // 计算CRC
        uint16_t crc = calculate_crc16((uint8_t*)&resp->length, resp->length - 2);
        resp->crc = crc;

        // 发送响应
        tcp_write(ctx->pcb, ctx->tx_buffer, sizeof(cmd_read_sensor_resp_t), TCP_WRITE_FLAG_COPY);
        tcp_output(ctx->pcb);
    }
}

// 发送错误响应
static void tcp_send_error_response(tcp_server_ctx_t *ctx, uint8_t cmd, uint8_t error_code)
{
    typedef struct __attribute__((packed)) {
        uint16_t sync_word;
        uint16_t length;
        uint8_t  cmd;
        uint8_t  error_code;
        uint16_t crc;
    } error_response_t;

    error_response_t *resp = (error_response_t *)ctx->tx_buffer;
    resp->sync_word = 0xAA55;
    resp->length = sizeof(error_response_t) - 2;
    resp->cmd = 0xFF;  // 错误响应命令码
    resp->error_code = error_code;

    uint16_t crc = calculate_crc16((uint8_t*)&resp->length, resp->length - 2);
    resp->crc = crc;

    tcp_write(ctx->pcb, ctx->tx_buffer, sizeof(error_response_t), TCP_WRITE_FLAG_COPY);
    tcp_output(ctx->pcb);
}
```

### 8.3 主动上报实现

```c
// 定期上报实时数据 (在tcp_task中调用)
void tcp_report_realtime_data(void)
{
    if (!g_tcp_ctx.connected) {
        return;
    }

    sensor_context_t *sensors = get_sensor_context();

    cmd_report_data_t *report = (cmd_report_data_t *)g_tcp_ctx.tx_buffer;
    report->sync_word = 0xAA55;
    report->cmd = 0xF0;
    report->report_type = 0x00;  // 实时数据

    // 填充传感器数据
    for (int i = 0; i < 3; i++) {
        report->realtime_data.temp[i] = (int16_t)(sensors->temp_values[i] * 10.0f);
    }
    for (int i = 0; i < 4; i++) {
        report->realtime_data.pressure[i] = (uint16_t)(sensors->pressure_values[i] * 10.0f);
    }
    for (int i = 0; i < 3; i++) {
        report->realtime_data.level[i] = (uint16_t)(sensors->level_values[i] * 10.0f);
    }
    report->realtime_data.flow = (uint16_t)(sensors->flow_value * 100.0f);

    // 填充状态信息
    report->realtime_data.status_word = get_system_status_word();
    report->realtime_data.alarm_word1 = get_alarm_word1();
    report->realtime_data.alarm_word2 = get_alarm_word2();

    report->timestamp = HAL_GetTick();
    report->length = sizeof(cmd_report_data_t) - 2;

    // 计算CRC
    uint16_t crc = calculate_crc16((uint8_t*)&report->length, report->length - 2);
    memcpy(&g_tcp_ctx.tx_buffer[report->length], &crc, 2);

    // 发送
    tcp_write(g_tcp_ctx.pcb, g_tcp_ctx.tx_buffer, report->length + 2, TCP_WRITE_FLAG_COPY);
    tcp_output(g_tcp_ctx.pcb);
}

// 告警上报 (在safety_task中检测到告警时调用)
void tcp_report_alarm(uint16_t alarm_code, uint8_t level, int16_t related_value)
{
    if (!g_tcp_ctx.connected) {
        return;
    }

    cmd_alarm_report_t *report = (cmd_alarm_report_t *)g_tcp_ctx.tx_buffer;
    report->sync_word = 0xAA55;
    report->length = sizeof(cmd_alarm_report_t) - 2;
    report->cmd = 0xF1;
    report->alarm_level = level;
    report->alarm_code = alarm_code;
    report->alarm_word1 = get_alarm_word1();
    report->alarm_word2 = get_alarm_word2();
    report->related_value = related_value;
    report->timestamp = HAL_GetTick();

    uint16_t crc = calculate_crc16((uint8_t*)&report->length, report->length - 2);
    report->crc = crc;

    tcp_write(g_tcp_ctx.pcb, g_tcp_ctx.tx_buffer, sizeof(cmd_alarm_report_t), TCP_WRITE_FLAG_COPY);
    tcp_output(g_tcp_ctx.pcb);
}
```

---

## 9. 上位机实现示例 (Python)

### 9.1 Python客户端类

```python
import socket
import struct
import time
from enum import IntEnum

class TcpCommand(IntEnum):
    READ_SENSOR = 0x01
    WRITE_SETPOINT = 0x02
    READ_STATUS = 0x03
    WRITE_CONTROL = 0x04
    READ_CONFIG = 0x05
    WRITE_CONFIG = 0x06
    ALARM_ACK = 0x07
    RESET = 0x08

class ErrorCode(IntEnum):
    ERR_OK = 0x00
    ERR_INVALID_CMD = 0x01
    ERR_INVALID_PARAM = 0x02
    ERR_DEVICE_BUSY = 0x03
    ERR_TIMEOUT = 0x04
    ERR_CHECKSUM = 0x05

class InkSupplyTcpClient:
    def __init__(self, host='192.168.1.100', port=8888):
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.host, self.port))
            self.connected = True
            print(f"已连接到 {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.connected = False

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

    def send_command(self, cmd, index, data=b''):
        if not self.connected:
            return None

        # 构造帧
        length = 4 + len(data) + 2  # header(4) + data + crc(2)
        frame = struct.pack('<HHBB', 0xAA55, length, cmd, index)
        frame += data

        # 计算CRC (从length字段开始)
        crc = self.calculate_crc16(frame[2:])
        frame += struct.pack('<H', crc)

        # 发送
        try:
            self.sock.sendall(frame)
            return self.receive_response()
        except Exception as e:
            print(f"发送失败: {e}")
            return None

    def receive_response(self):
        try:
            # 接收帧头 (6字节)
            header = self.sock.recv(6)
            if len(header) < 6:
                return None

            sync, length, cmd, index = struct.unpack('<HHBB', header)
            if sync != 0xAA55:
                print("同步字错误")
                return None

            # 接收剩余数据
            remaining = length - 4
            data = self.sock.recv(remaining)
            if len(data) < remaining:
                return None

            # 验证CRC
            crc_recv = struct.unpack('<H', data[-2:])[0]
            crc_calc = self.calculate_crc16(header[2:] + data[:-2])
            if crc_recv != crc_calc:
                print("CRC校验失败")
                return None

            return {
                'cmd': cmd,
                'index': index,
                'data': data[:-2]
            }
        except socket.timeout:
            print("接收超时")
            return None
        except Exception as e:
            print(f"接收失败: {e}")
            return None

    def read_sensor(self, sensor_index):
        """读取单个传感器"""
        resp = self.send_command(TcpCommand.READ_SENSOR, sensor_index)
        if resp and resp['cmd'] == 0x81:
            error_code, status, value, timestamp = struct.unpack(
                '<BBhI', resp['data'])
            if error_code == ErrorCode.ERR_OK:
                return {
                    'status': status,
                    'value': value / 10.0,  # 定点数转浮点
                    'timestamp': timestamp
                }
        return None

    def read_all_sensors(self):
        """批量读取所有传感器"""
        resp = self.send_command(TcpCommand.READ_SENSOR, 0xFF)
        if resp and resp['cmd'] == 0x81:
            error_code, sensor_count = struct.unpack('<BB', resp['data'][:2])
            if error_code == ErrorCode.ERR_OK:
                sensors = []
                offset = 2
                for i in range(sensor_count):
                    status, value = struct.unpack('<Bh',
                        resp['data'][offset:offset+3])
                    sensors.append({
                        'index': i,
                        'status': status,
                        'value': value / 10.0
                    })
                    offset += 3
                timestamp = struct.unpack('<I', resp['data'][offset:offset+4])[0]
                return {'sensors': sensors, 'timestamp': timestamp}
        return None

    def write_setpoint(self, index, value):
        """写入设定值"""
        data = struct.pack('<h', int(value * 10))  # 浮点转定点
        resp = self.send_command(TcpCommand.WRITE_SETPOINT, index, data)
        if resp and resp['cmd'] == 0x82:
            error_code, _, actual_value = struct.unpack('<BBh', resp['data'])
            return error_code == ErrorCode.ERR_OK
        return False

    def read_status(self):
        """读取系统状态"""
        resp = self.send_command(TcpCommand.READ_STATUS, 0x00)
        if resp and resp['cmd'] == 0x83:
            (error_code, run_state, status_word, alarm_word1, alarm_word2,
             cpu_usage, free_heap, uptime, timestamp) = struct.unpack(
                '<BBHHHHIII', resp['data'])
            if error_code == ErrorCode.ERR_OK:
                return {
                    'run_state': run_state,
                    'status_word': status_word,
                    'alarm_word1': alarm_word1,
                    'alarm_word2': alarm_word2,
                    'cpu_usage': cpu_usage / 100.0,
                    'free_heap': free_heap,
                    'uptime': uptime,
                    'timestamp': timestamp
                }
        return None

    def write_control(self, actuator_index, value, mode=1):
        """控制执行器"""
        data = struct.pack('<BB', value, mode)
        resp = self.send_command(TcpCommand.WRITE_CONTROL, actuator_index, data)
        if resp and resp['cmd'] == 0x84:
            error_code, actual_state = struct.unpack('<BB', resp['data'][:2])
            return error_code == ErrorCode.ERR_OK
        return False

    def alarm_acknowledge(self, alarm_mask=0xFFFF):
        """确认告警"""
        data = struct.pack('<H', alarm_mask)
        resp = self.send_command(TcpCommand.ALARM_ACK, 0x00, data)
        if resp and resp['cmd'] == 0x87:
            error_code, _, remaining = struct.unpack('<BBH', resp['data'])
            return error_code == ErrorCode.ERR_OK
        return False

# 使用示例
def main():
    client = InkSupplyTcpClient('192.168.1.100', 8888)

    if not client.connect():
        return

    try:
        # 读取系统状态
        status = client.read_status()
        if status:
            print(f"系统状态: {status}")

        # 读取所有传感器
        sensors = client.read_all_sensors()
        if sensors:
            print(f"传感器数据:")
            for s in sensors['sensors']:
                print(f"  传感器{s['index']}: {s['value']:.1f}")

        # 设置温度目标值为60°C
        if client.write_setpoint(0x00, 60.0):
            print("温度设定值已更新")

        # 打开电磁阀1
        if client.write_control(0x00, 1, 1):
            print("电磁阀1已打开")

        # 周期性读取数据
        while True:
            temp1 = client.read_sensor(0x00)  # 读取温度传感器1
            if temp1:
                print(f"温度1: {temp1['value']:.1f}°C")
            time.sleep(1)

    except KeyboardInterrupt:
        print("停止")
    finally:
        client.disconnect()

if __name__ == '__main__':
    main()
```

---

## 10. 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 数据更新周期 | 100~500ms | 可配置的上报周期 |
| 命令响应时间 | < 50ms | 从接收到响应的平均延迟 |
| 最大并发连接 | 1 | 仅支持单客户端连接 |
| 数据吞吐量 | > 10KB/s | 实时数据流量 |
| 丢包率 | < 0.01% | TCP协议保证可靠传输 |
| 重连时间 | < 5s | 连接断开后的重连间隔 |

---

## 11. 安全机制

### 11.1 通信超时
- **命令超时**: 固件收到命令后必须在50ms内响应
- **心跳超时**: 10秒内无有效命令，固件可断开连接
- **上位机超时**: 请求无响应时，1秒后重发(最多3次)

### 11.2 参数保护
- **范围检查**: 所有设定值必须在安全范围内，否则返回ERR_OUT_OF_RANGE
- **锁定保护**: 配置锁定后(bit 12=1)，拒绝参数修改，返回ERR_PERMISSION
- **复位保护**: 系统复位命令需要魔术字(0x5AA5)确认

### 11.3 数据完整性
- **CRC校验**: 所有帧必须通过CRC16校验
- **帧同步**: 使用0xAA55同步字识别帧边界
- **长度校验**: 帧长度必须与length字段一致

---

## 12. 测试与验证

### 12.1 功能测试项
1. **连接建立测试**
   - TCP三次握手成功
   - 多次重连稳定性
   - 异常断开恢复

2. **命令响应测试**
   - 所有命令码正常响应
   - 错误处理正确
   - 响应时间符合要求

3. **数据完整性测试**
   - CRC校验正确
   - 定点数编解码准确
   - 批量数据传输无丢失

4. **实时性能测试**
   - 数据更新周期稳定
   - 命令响应延迟
   - 高频请求压力测试

5. **异常处理测试**
   - 无效命令处理
   - 参数越界处理
   - 网络中断恢复

### 12.2 兼容性测试
- **操作系统**: Windows/Linux/macOS上位机
- **编程语言**: Python/C#/C++/LabVIEW客户端
- **网络环境**: 100Mbps/1Gbps以太网
- **路由器**: 跨子网通信测试

---

## 13. 附录

### 13.1 告警代码定义
| 告警码 | 名称 | 严重级别 | 说明 |
|-------|------|---------|------|
| 0x1001 | TEMP_SENSOR_FAULT | 2 | 温度传感器故障 |
| 0x1002 | PRESSURE_SENSOR_FAULT | 2 | 压力传感器故障 |
| 0x1003 | LEVEL_SENSOR_FAULT | 2 | 液位传感器故障 |
| 0x2001 | VALVE_FAULT | 2 | 电磁阀故障 |
| 0x2002 | HEATER_FAULT | 3 | 加热器故障 |
| 0x2003 | PUMP_FAULT | 2 | 泵故障 |
| 0x3001 | TEMP_OVER_LIMIT | 3 | 温度超限 |
| 0x3002 | PRESSURE_OVER_LIMIT | 3 | 压力超限 |
| 0x3003 | LEVEL_ABNORMAL | 2 | 液位异常 |
| 0x4001 | COMM_TIMEOUT | 2 | TCP通信超时 |
| 0x5001 | CONFIG_INVALID | 2 | 配置无效 |
| 0x6001 | SYSTEM_OVERLOAD | 3 | 系统过载 |

**严重级别**:
- 0: 信息
- 1: 警告 (系统继续运行)
- 2: 错误 (部分功能受限)
- 3: 严重 (进入安全模式)

### 13.2 与EtherCAT协议对比

| 特性 | TCP协议 | EtherCAT协议 |
|------|---------|-------------|
| 通信周期 | 100~500ms | 1ms |
| 实时性 | 中等 | 高 |
| 延迟 | 10~50ms | < 1ms |
| 网络要求 | 标准以太网 | 专用EtherCAT网络 |
| 上位机兼容性 | 广泛 (TCP/IP通用) | 需要EtherCAT主站库 |
| 实现复杂度 | 低 | 高 |
| 成本 | 低 | 高 (需要专用硬件) |
| 适用场景 | 监控/配置/数据采集 | 精确运动控制/同步控制 |

**选择建议**:
- **选择TCP**: 监控系统、数据采集、远程配置、多平台兼容
- **选择EtherCAT**: 高精度同步控制、多轴运动、实时性要求高

### 13.3 参考文档
- `墨路控制系统详细设计文档v3.md` - 系统总体设计
- `墨路控制系统EtherCAT通信协议设计文档.md` - EtherCAT协议对比参考
- `app/tcp_server.c/h` - TCP服务器实现
- `middleware/protocol.c/h` - 协议解析实现

### 13.4 缩写术语表
- **TCP**: Transmission Control Protocol (传输控制协议)
- **IP**: Internet Protocol (互联网协议)
- **CRC**: Cyclic Redundancy Check (循环冗余校验)
- **lwIP**: Lightweight IP (轻量级TCP/IP协议栈)
- **PDO**: Process Data Object (过程数据对象, EtherCAT术语)
- **SDO**: Service Data Object (服务数据对象, EtherCAT术语)

---

**文档版本**: v1.0
**编写日期**: 2025-09-30
**编写依据**: 墨路通信协议表格 + 墨路控制系统详细设计文档v3 + EtherCAT协议设计文档
**适用固件版本**: v3.0.0及以上