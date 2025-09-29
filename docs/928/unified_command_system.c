/*
 * TCP/EtherCAT 统一命令处理实现
 * 实现上行/下行命令的协议无关性
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lwip/sockets.h"
#include "ethercat_slave.h"

//=============================================================================
// 1. 统一命令抽象层定义
//=============================================================================

/* 统一命令类型定义 */
typedef enum {
    /* 下行命令（上位机→控制器）*/
    UNIFIED_CMD_READ_SENSOR_DATA = 0x01,      // 读取传感器数据
    UNIFIED_CMD_READ_SYSTEM_STATUS = 0x02,    // 读取系统状态
    UNIFIED_CMD_SET_CONTROL_MODE = 0x03,      // 设置控制模式
    UNIFIED_CMD_SET_SETPOINT = 0x04,          // 设置设定值
    UNIFIED_CMD_MANUAL_CONTROL = 0x05,        // 手动控制输出
    UNIFIED_CMD_SET_PID_PARAMS = 0x06,        // 设置PID参数
    UNIFIED_CMD_SET_SAFETY_PARAMS = 0x07,     // 设置安全参数
    UNIFIED_CMD_SYSTEM_RESET = 0x08,          // 系统复位
    UNIFIED_CMD_SAVE_CONFIG = 0x09,           // 保存配置
    UNIFIED_CMD_FIRMWARE_UPGRADE = 0x0A,      // 固件升级

    /* 上行命令（控制器→上位机）*/
    UNIFIED_CMD_SENSOR_DATA_REPORT = 0x80,    // 传感器数据上报
    UNIFIED_CMD_STATUS_REPORT = 0x81,         // 状态上报
    UNIFIED_CMD_ALARM_REPORT = 0x82,          // 告警上报
    UNIFIED_CMD_CONTROL_FEEDBACK = 0x83,      // 控制反馈
    UNIFIED_CMD_HEARTBEAT = 0x84,             // 心跳包

    /* 响应命令 */
    UNIFIED_CMD_ACK_SUCCESS = 0xF0,           // 成功响应
    UNIFIED_CMD_ACK_ERROR = 0xF1,             // 错误响应

} unified_command_t;

/* 统一命令数据结构 */
typedef struct {
    unified_command_t cmd;        // 命令类型
    uint16_t data_length;         // 数据长度
    uint8_t* data;                // 数据指针
    uint32_t timestamp;           // 时间戳
    uint16_t sequence;            // 序列号
} unified_cmd_packet_t;

/* 命令参数联合体 */
typedef union {
    /* 传感器读取参数 */
    struct {
        uint8_t sensor_type;      // 传感器类型
        uint8_t channel;          // 通道号
    } sensor_read;

    /* 设定值参数 */
    struct {
        uint8_t loop_id;          // 控制回路ID
        float setpoint;           // 设定值
    } setpoint;

    /* 手动控制参数 */
    struct {
        uint8_t output_channel;   // 输出通道
        uint16_t output_value;    // 输出值
        uint8_t output_type;      // 输出类型
    } manual_control;

    /* PID参数 */
    struct {
        uint8_t loop_id;          // 回路ID
        float kp, ki, kd;         // PID参数
        float output_min, output_max; // 输出限制
    } pid_params;

    /* 安全参数 */
    struct {
        uint8_t sensor_type;      // 传感器类型
        uint8_t channel;          // 通道号
        float alarm_high;         // 高限报警
        float alarm_low;          // 低限报警
        float shutdown_high;      // 高限停机
        float shutdown_low;       // 低限停机
    } safety_params;

    /* 控制模式参数 */
    struct {
        uint8_t mode;             // 控制模式
        uint8_t loop_id;          // 控制回路ID
    } control_mode;

} cmd_params_t;

/* 统一响应数据结构 */
typedef union {
    /* 传感器数据响应 */
    struct {
        int16_t temperature[3];
        uint16_t pressure[4];
        uint16_t level[4];
        uint16_t flow_rate;
        uint8_t status_bits;
        uint32_t timestamp;
    } sensor_data;

    /* 系统状态响应 */
    struct {
        uint16_t system_state;
        uint16_t alarm_word;
        uint16_t fault_code;
        uint8_t cpu_usage;
        uint8_t mcu_temperature;
        uint32_t uptime;
    } system_status;

    /* 控制反馈响应 */
    struct {
        uint8_t loop_id;
        uint8_t mode;
        float setpoint;
        float process_value;
        float output_value;
        uint8_t status;
    } control_feedback;

    /* 告警响应 */
    struct {
        uint16_t alarm_id;
        uint8_t alarm_type;
        uint8_t severity;
        uint8_t source_channel;
        float current_value;
        float threshold_value;
        uint32_t timestamp;
        char description[32];
    } alarm_info;

    /* 错误响应 */
    struct {
        uint8_t error_code;
        char error_message[64];
    } error_info;

} cmd_response_t;

//=============================================================================
// 2. 统一命令处理接口
//=============================================================================

/* 命令处理函数指针类型 */
typedef int (*cmd_handler_func_t)(const cmd_params_t* params, cmd_response_t* response);

/* 命令处理接口 */
typedef struct {
    /* 下行命令处理接口 */
    int (*process_downlink_cmd)(unified_command_t cmd, const cmd_params_t* params, cmd_response_t* response);

    /* 上行命令发送接口 */
    int (*send_uplink_cmd)(unified_command_t cmd, const cmd_response_t* response);

    /* 命令队列接口 */
    int (*enqueue_downlink_cmd)(const unified_cmd_packet_t* packet);
    int (*dequeue_downlink_cmd)(unified_cmd_packet_t* packet);

} command_interface_t;

/* 全局命令接口 */
static command_interface_t* g_cmd_interface = NULL;

//=============================================================================
// 3. TCP协议命令映射和处理
//=============================================================================

/* TCP命令映射表 */
typedef struct {
    unified_command_t unified_cmd;    // 统一命令
    uint8_t tcp_cmd;                  // TCP协议命令字
    const char* description;          // 描述
} tcp_cmd_mapping_t;

static const tcp_cmd_mapping_t tcp_cmd_map[] = {
    /* 下行命令映射 */
    {UNIFIED_CMD_READ_SENSOR_DATA,    0x20, "Read Sensor Data"},
    {UNIFIED_CMD_READ_SYSTEM_STATUS,  0x15, "Read System Status"},
    {UNIFIED_CMD_SET_CONTROL_MODE,    0x30, "Set Control Mode"},
    {UNIFIED_CMD_SET_SETPOINT,        0x31, "Set Setpoint"},
    {UNIFIED_CMD_MANUAL_CONTROL,      0x32, "Manual Control"},
    {UNIFIED_CMD_SET_PID_PARAMS,      0x40, "Set PID Parameters"},
    {UNIFIED_CMD_SET_SAFETY_PARAMS,   0x42, "Set Safety Parameters"},
    {UNIFIED_CMD_SYSTEM_RESET,        0x50, "System Reset"},
    {UNIFIED_CMD_SAVE_CONFIG,         0x51, "Save Configuration"},

    /* 上行命令映射 */
    {UNIFIED_CMD_SENSOR_DATA_REPORT,  0x90, "Sensor Data Report"},
    {UNIFIED_CMD_STATUS_REPORT,       0x92, "Status Report"},
    {UNIFIED_CMD_ALARM_REPORT,        0x91, "Alarm Report"},
    {UNIFIED_CMD_CONTROL_FEEDBACK,    0x93, "Control Feedback"},
    {UNIFIED_CMD_HEARTBEAT,           0x84, "Heartbeat"},

    /* 响应命令映射 */
    {UNIFIED_CMD_ACK_SUCCESS,         0xA0, "Success ACK"},
    {UNIFIED_CMD_ACK_ERROR,           0xA1, "Error ACK"},
};

/* TCP协议上下文 */
typedef struct {
    int client_socket;
    QueueHandle_t cmd_queue;          // 命令队列
    QueueHandle_t response_queue;     // 响应队列
    bool connected;
    uint16_t sequence_counter;        // 序列号计数器
} tcp_cmd_context_t;

static tcp_cmd_context_t tcp_cmd_ctx = {0};

/* TCP命令到统一命令的转换 */
static unified_command_t tcp_to_unified_cmd(uint8_t tcp_cmd) {
    for (int i = 0; i < sizeof(tcp_cmd_map)/sizeof(tcp_cmd_map[0]); i++) {
        if (tcp_cmd_map[i].tcp_cmd == tcp_cmd) {
            return tcp_cmd_map[i].unified_cmd;
        }
    }
    return 0xFF; // 无效命令
}

/* 统一命令到TCP命令的转换 */
static uint8_t unified_to_tcp_cmd(unified_command_t unified_cmd) {
    for (int i = 0; i < sizeof(tcp_cmd_map)/sizeof(tcp_cmd_map[0]); i++) {
        if (tcp_cmd_map[i].unified_cmd == unified_cmd) {
            return tcp_cmd_map[i].tcp_cmd;
        }
    }
    return 0xFF; // 无效命令
}

/* TCP下行命令处理 */
static int tcp_process_downlink_cmd(unified_command_t cmd, const cmd_params_t* params, cmd_response_t* response) {
    int result = 0;

    switch (cmd) {
        case UNIFIED_CMD_READ_SENSOR_DATA:
            result = read_sensor_data_handler(params, response);
            break;

        case UNIFIED_CMD_READ_SYSTEM_STATUS:
            result = read_system_status_handler(params, response);
            break;

        case UNIFIED_CMD_SET_CONTROL_MODE:
            result = set_control_mode_handler(params, response);
            break;

        case UNIFIED_CMD_SET_SETPOINT:
            result = set_setpoint_handler(params, response);
            break;

        case UNIFIED_CMD_MANUAL_CONTROL:
            result = manual_control_handler(params, response);
            break;

        case UNIFIED_CMD_SET_PID_PARAMS:
            result = set_pid_params_handler(params, response);
            break;

        case UNIFIED_CMD_SET_SAFETY_PARAMS:
            result = set_safety_params_handler(params, response);
            break;

        case UNIFIED_CMD_SYSTEM_RESET:
            result = system_reset_handler(params, response);
            break;

        case UNIFIED_CMD_SAVE_CONFIG:
            result = save_config_handler(params, response);
            break;

        default:
            result = -1; // 不支持的命令
            break;
    }

    return result;
}

/* TCP上行命令发送 */
static int tcp_send_uplink_cmd(unified_command_t cmd, const cmd_response_t* response) {
    if (!tcp_cmd_ctx.connected) {
        return -1;
    }

    /* 转换为TCP命令字 */
    uint8_t tcp_cmd = unified_to_tcp_cmd(cmd);
    if (tcp_cmd == 0xFF) {
        return -2; // 无效命令
    }

    /* 构造TCP数据包 */
    tcp_protocol_frame_t* frame;
    uint16_t data_size = 0;

    switch (cmd) {
        case UNIFIED_CMD_SENSOR_DATA_REPORT:
            data_size = sizeof(response->sensor_data);
            break;
        case UNIFIED_CMD_STATUS_REPORT:
            data_size = sizeof(response->system_status);
            break;
        case UNIFIED_CMD_ALARM_REPORT:
            data_size = sizeof(response->alarm_info);
            break;
        case UNIFIED_CMD_CONTROL_FEEDBACK:
            data_size = sizeof(response->control_feedback);
            break;
        case UNIFIED_CMD_HEARTBEAT:
            data_size = 4; // 只发送时间戳
            break;
        default:
            return -3; // 不支持的上行命令
    }

    frame = pvPortMalloc(sizeof(tcp_frame_header_t) + data_size);
    if (!frame) {
        return -4; // 内存分配失败
    }

    /* 填充TCP帧头 */
    frame->header.sync_word = 0xAA55;
    frame->header.cmd = tcp_cmd;
    frame->header.index = tcp_cmd_ctx.sequence_counter++;
    frame->header.length = sizeof(tcp_frame_header_t) + data_size;

    /* 填充数据 */
    memcpy(frame->data, response, data_size);

    /* 计算校验和 */
    frame->header.checksum = calculate_checksum((uint8_t*)frame, frame->header.length - 2);

    /* 发送数据 */
    int bytes_sent = send(tcp_cmd_ctx.client_socket, frame, frame->header.length, 0);

    vPortFree(frame);

    return (bytes_sent > 0) ? 0 : -5;
}

/* TCP命令队列操作 */
static int tcp_enqueue_downlink_cmd(const unified_cmd_packet_t* packet) {
    return xQueueSend(tcp_cmd_ctx.cmd_queue, packet, pdMS_TO_TICKS(100)) == pdTRUE ? 0 : -1;
}

static int tcp_dequeue_downlink_cmd(unified_cmd_packet_t* packet) {
    return xQueueReceive(tcp_cmd_ctx.cmd_queue, packet, 0) == pdTRUE ? 0 : -1;
}

/* TCP命令接口实例 */
static command_interface_t tcp_cmd_interface = {
    .process_downlink_cmd = tcp_process_downlink_cmd,
    .send_uplink_cmd = tcp_send_uplink_cmd,
    .enqueue_downlink_cmd = tcp_enqueue_downlink_cmd,
    .dequeue_downlink_cmd = tcp_dequeue_downlink_cmd,
};

//=============================================================================
// 4. EtherCAT协议命令映射和处理
//=============================================================================

/* EtherCAT命令映射（通过对象字典索引） */
typedef struct {
    unified_command_t unified_cmd;    // 统一命令
    uint16_t sdo_index;               // SDO对象字典索引
    uint8_t sdo_subindex;             // SDO子索引
    const char* description;          // 描述
} ethercat_cmd_mapping_t;

static const ethercat_cmd_mapping_t ethercat_cmd_map[] = {
    /* 下行命令映射（通过SDO） */
    {UNIFIED_CMD_SET_CONTROL_MODE,    0x3000, 0x01, "Control Mode"},
    {UNIFIED_CMD_SET_SETPOINT,        0x3001, 0x01, "Setpoint Value"},
    {UNIFIED_CMD_SET_PID_PARAMS,      0x3002, 0x01, "PID Parameters"},
    {UNIFIED_CMD_SET_SAFETY_PARAMS,   0x4000, 0x01, "Safety Parameters"},
    {UNIFIED_CMD_SYSTEM_RESET,        0x5000, 0x01, "System Reset"},
    {UNIFIED_CMD_SAVE_CONFIG,         0x5001, 0x01, "Save Config"},

    /* 上行命令映射（通过PDO/SDO读取） */
    {UNIFIED_CMD_SENSOR_DATA_REPORT,  0x6000, 0x00, "Sensor Data"},
    {UNIFIED_CMD_STATUS_REPORT,       0x6001, 0x00, "System Status"},
    {UNIFIED_CMD_CONTROL_FEEDBACK,    0x6002, 0x00, "Control Feedback"},
};

/* EtherCAT协议上下文 */
typedef struct {
    bool slave_operational;           // 从站运行状态
    QueueHandle_t sdo_cmd_queue;      // SDO命令队列
    QueueHandle_t pdo_update_queue;   // PDO更新队列
    uint32_t pdo_sequence;            // PDO序列号
    SemaphoreHandle_t cmd_mutex;      // 命令互斥锁
} ethercat_cmd_context_t;

static ethercat_cmd_context_t ethercat_cmd_ctx = {0};

/* EtherCAT下行命令处理 */
static int ethercat_process_downlink_cmd(unified_command_t cmd, const cmd_params_t* params, cmd_response_t* response) {
    int result = 0;

    /* EtherCAT下行命令主要通过SDO写入或PDO控制字实现 */
    switch (cmd) {
        case UNIFIED_CMD_SET_CONTROL_MODE:
            /* 通过输出PDO的控制字设置模式 */
            if (xSemaphoreTake(ethercat_cmd_ctx.cmd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ethercat_output_pdo_t* output_pdo = ec_slave_get_output_pdo();
                output_pdo->operation_mode = params->control_mode.mode;
                output_pdo->control_word |= (1 << params->control_mode.loop_id);
                xSemaphoreGive(ethercat_cmd_ctx.cmd_mutex);
                result = 0;
            } else {
                result = -1;
            }
            break;

        case UNIFIED_CMD_SET_SETPOINT:
            /* 通过输出PDO设置设定值 */
            if (xSemaphoreTake(ethercat_cmd_ctx.cmd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ethercat_output_pdo_t* output_pdo = ec_slave_get_output_pdo();
                switch (params->setpoint.loop_id) {
                    case 0: case 1: case 2: // 温度回路
                        output_pdo->temp_setpoint[params->setpoint.loop_id] = (int16_t)(params->setpoint.setpoint * 100);
                        break;
                    case 3: case 4: case 5: case 6: // 压力回路
                        output_pdo->pressure_setpoint[params->setpoint.loop_id - 3] = (uint16_t)(params->setpoint.setpoint * 10);
                        break;
                    case 7: case 8: case 9: // 液位回路
                        output_pdo->level_setpoint[params->setpoint.loop_id - 7] = (uint16_t)(params->setpoint.setpoint * 10);
                        break;
                    case 10: // 流量回路
                        output_pdo->flow_setpoint = (uint16_t)(params->setpoint.setpoint * 100);
                        break;
                }
                xSemaphoreGive(ethercat_cmd_ctx.cmd_mutex);
                result = 0;
            } else {
                result = -1;
            }
            break;

        case UNIFIED_CMD_MANUAL_CONTROL:
            /* 通过输出PDO的执行器控制区 */
            if (xSemaphoreTake(ethercat_cmd_ctx.cmd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ethercat_output_pdo_t* output_pdo = ec_slave_get_output_pdo();
                uint8_t channel = params->manual_control.output_channel;
                uint16_t value = params->manual_control.output_value;

                if (channel < 2) {
                    output_pdo->valve_control[channel] = value;
                } else if (channel < 5) {
                    output_pdo->heater_control[channel - 2] = value;
                } else if (channel < 7) {
                    output_pdo->pump_speed[channel - 5] = value;
                } else if (channel < 9) {
                    output_pdo->pump_dc_control[channel - 7] = value;
                }
                xSemaphoreGive(ethercat_cmd_ctx.cmd_mutex);
                result = 0;
            } else {
                result = -1;
            }
            break;

        case UNIFIED_CMD_SET_PID_PARAMS:
            /* 通过输出PDO的PID参数区或SDO写入 */
            if (xSemaphoreTake(ethercat_cmd_ctx.cmd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ethercat_output_pdo_t* output_pdo = ec_slave_get_output_pdo();
                output_pdo->kp = params->pid_params.kp;
                output_pdo->ki = params->pid_params.ki;
                output_pdo->kd = params->pid_params.kd;
                output_pdo->loop_id = params->pid_params.loop_id;
                output_pdo->pid_enable = 1;
                xSemaphoreGive(ethercat_cmd_ctx.cmd_mutex);
                result = 0;
            } else {
                result = -1;
            }
            break;

        case UNIFIED_CMD_SET_SAFETY_PARAMS:
        case UNIFIED_CMD_SYSTEM_RESET:
        case UNIFIED_CMD_SAVE_CONFIG:
            /* 这些命令通过SDO邮箱通信实现 */
            result = ethercat_sdo_write_command(cmd, params);
            break;

        default:
            result = -1; // 不支持的命令
            break;
    }

    return result;
}

/* EtherCAT上行命令发送 */
static int ethercat_send_uplink_cmd(unified_command_t cmd, const cmd_response_t* response) {
    if (!ethercat_cmd_ctx.slave_operational) {
        return -1;
    }

    int result = 0;

    switch (cmd) {
        case UNIFIED_CMD_SENSOR_DATA_REPORT:
            /* 通过输入PDO发送传感器数据 */
            if (xSemaphoreTake(ethercat_cmd_ctx.cmd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ethercat_input_pdo_t* input_pdo = ec_slave_get_input_pdo();
                memcpy(input_pdo->temperature, response->sensor_data.temperature, sizeof(input_pdo->temperature));
                memcpy(input_pdo->pressure, response->sensor_data.pressure, sizeof(input_pdo->pressure));
                memcpy(input_pdo->level, response->sensor_data.level, sizeof(input_pdo->level));
                input_pdo->flow_rate = response->sensor_data.flow_rate;
                input_pdo->sensor_status = response->sensor_data.status_bits;
                input_pdo->timestamp = response->sensor_data.timestamp;
                xSemaphoreGive(ethercat_cmd_ctx.cmd_mutex);
                result = 0;
            } else {
                result = -1;
            }
            break;

        case UNIFIED_CMD_STATUS_REPORT:
            /* 通过输入PDO的系统状态区发送 */
            if (xSemaphoreTake(ethercat_cmd_ctx.cmd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ethercat_input_pdo_t* input_pdo = ec_slave_get_input_pdo();
                input_pdo->system_state = response->system_status.system_state;
                input_pdo->alarm_word = response->system_status.alarm_word;
                input_pdo->fault_code = response->system_status.fault_code;
                input_pdo->cpu_usage = response->system_status.cpu_usage;
                input_pdo->temperature_mcu = response->system_status.mcu_temperature;
                xSemaphoreGive(ethercat_cmd_ctx.cmd_mutex);
                result = 0;
            } else {
                result = -1;
            }
            break;

        case UNIFIED_CMD_ALARM_REPORT:
            /* 告警通过邮箱通信（Emergency）或特殊PDO位发送 */
            result = ethercat_send_emergency_message(&response->alarm_info);
            break;

        case UNIFIED_CMD_CONTROL_FEEDBACK:
            /* 控制反馈通过输入PDO的执行器反馈区发送 */
            if (xSemaphoreTake(ethercat_cmd_ctx.cmd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ethercat_input_pdo_t* input_pdo = ec_slave_get_input_pdo();
                // 根据回路ID更新对应的反馈数据
                // 这里简化处理，实际需要根据回路ID映射到具体的反馈区域
                xSemaphoreGive(ethercat_cmd_ctx.cmd_mutex);
                result = 0;
            } else {
                result = -1;
            }
            break;

        case UNIFIED_CMD_HEARTBEAT:
            /* EtherCAT的心跳通过PDO的时间戳字段体现 */
            if (xSemaphoreTake(ethercat_cmd_ctx.cmd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ethercat_input_pdo_t* input_pdo = ec_slave_get_input_pdo();
                input_pdo->timestamp = xTaskGetTickCount();
                xSemaphoreGive(ethercat_cmd_ctx.cmd_mutex);
                result = 0;
            } else {
                result = -1;
            }
            break;

        default:
            result = -1; // 不支持的上行命令
            break;
    }

    return result;
}

/* EtherCAT命令队列操作 */
static int ethercat_enqueue_downlink_cmd(const unified_cmd_packet_t* packet) {
    return xQueueSend(ethercat_cmd_ctx.sdo_cmd_queue, packet, pdMS_TO_TICKS(100)) == pdTRUE ? 0 : -1;
}

static int ethercat_dequeue_downlink_cmd(unified_cmd_packet_t* packet) {
    return xQueueReceive(ethercat_cmd_ctx.sdo_cmd_queue, packet, 0) == pdTRUE ? 0 : -1;
}

/* EtherCAT命令接口实例 */
static command_interface_t ethercat_cmd_interface = {
    .process_downlink_cmd = ethercat_process_downlink_cmd,
    .send_uplink_cmd = ethercat_send_uplink_cmd,
    .enqueue_downlink_cmd = ethercat_enqueue_downlink_cmd,
    .dequeue_downlink_cmd = ethercat_dequeue_downlink_cmd,
};

//=============================================================================
// 5. 统一命令处理器实现
//=============================================================================

/* 命令处理器初始化 */
int command_processor_init(comm_protocol_t protocol) {
    switch (protocol) {
        case COMM_PROTOCOL_TCP:
            g_cmd_interface = &tcp_cmd_interface;
            /* 初始化TCP命令队列 */
            tcp_cmd_ctx.cmd_queue = xQueueCreate(16, sizeof(unified_cmd_packet_t));
            tcp_cmd_ctx.response_queue = xQueueCreate(16, sizeof(unified_cmd_packet_t));
            break;

        case COMM_PROTOCOL_ETHERCAT:
            g_cmd_interface = &ethercat_cmd_interface;
            /* 初始化EtherCAT命令队列 */
            ethercat_cmd_ctx.sdo_cmd_queue = xQueueCreate(8, sizeof(unified_cmd_packet_t));
            ethercat_cmd_ctx.pdo_update_queue = xQueueCreate(32, sizeof(unified_cmd_packet_t));
            ethercat_cmd_ctx.cmd_mutex = xSemaphoreCreateMutex();
            break;

        default:
            return -1;
    }

    return 0;
}

/* 应用层统一命令发送接口 */
int app_send_command(unified_command_t cmd, const cmd_response_t* response) {
    if (g_cmd_interface && g_cmd_interface->send_uplink_cmd) {
        return g_cmd_interface->send_uplink_cmd(cmd, response);
    }
    return -1;
}

/* 应用层统一命令处理接口 */
int app_process_command(unified_command_t cmd, const cmd_params_t* params, cmd_response_t* response) {
    if (g_cmd_interface && g_cmd_interface->process_downlink_cmd) {
        return g_cmd_interface->process_downlink_cmd(cmd, params, response);
    }
    return -1;
}

//=============================================================================
// 6. 具体命令处理函数实现
//=============================================================================

/* 读取传感器数据处理器 */
static int read_sensor_data_handler(const cmd_params_t* params, cmd_response_t* response) {
    /* 从传感器任务获取最新数据 */
    sensor_data_t current_sensor_data;
    if (get_current_sensor_data(&current_sensor_data) == 0) {
        /* 转换为统一响应格式 */
        memcpy(response->sensor_data.temperature, current_sensor_data.temperature,
               sizeof(response->sensor_data.temperature));
        memcpy(response->sensor_data.pressure, current_sensor_data.pressure,
               sizeof(response->sensor_data.pressure));
        memcpy(response->sensor_data.level, current_sensor_data.level,
               sizeof(response->sensor_data.level));
        response->sensor_data.flow_rate = current_sensor_data.flow_rate;
        response->sensor_data.status_bits = current_sensor_data.sensor_status;
        response->sensor_data.timestamp = current_sensor_data.timestamp;
        return 0;
    }
    return -1;
}

/* 读取系统状态处理器 */
static int read_system_status_handler(const cmd_params_t* params, cmd_response_t* response) {
    /* 获取系统状态 */
    system_status_t current_status;
    if (get_current_system_status(&current_status) == 0) {
        response->system_status.system_state = current_status.system_state;
        response->system_status.alarm_word = current_status.alarm_word;
        response->system_status.fault_code = current_status.fault_code;
        response->system_status.cpu_usage = current_status.cpu_usage;
        response->system_status.mcu_temperature = current_status.temperature_mcu;
        response->system_status.uptime = xTaskGetTickCount() / 1000; // 转换为秒
        return 0;
    }
    return -1;
}

/* 设置控制模式处理器 */
static int set_control_mode_handler(const cmd_params_t* params, cmd_response_t* response) {
    /* 设置控制模式 */
    if (set_controller_mode(params->control_mode.loop_id, params->control_mode.mode) == 0) {
        return 0;
    }
    return -1;
}

/* 设置设定值处理器 */
static int set_setpoint_handler(const cmd_params_t* params, cmd_response_t* response) {
    /* 设置控制回路设定值 */
    if (set_controller_setpoint(params->setpoint.loop_id, params->setpoint.setpoint) == 0) {
        return 0;
    }
    return -1;
}

/* 手动控制处理器 */
static int manual_control_handler(const cmd_params_t* params, cmd_response_t* response) {
    /* 执行手动控制 */
    if (execute_manual_control(params->manual_control.output_channel,
                               params->manual_control.output_value,
                               params->manual_control.output_type) == 0) {
        return 0;
    }
    return -1;
}

/* 设置PID参数处理器 */
static int set_pid_params_handler(const cmd_params_t* params, cmd_response_t* response) {
    /* 设置PID参数 */
    if (set_controller_pid_params(params->pid_params.loop_id,
                                  params->pid_params.kp,
                                  params->pid_params.ki,
                                  params->pid_params.kd,
                                  params->pid_params.output_min,
                                  params->pid_params.output_max) == 0) {
        return 0;
    }
    return -1;
}

/* 设置安全参数处理器 */
static int set_safety_params_handler(const cmd_params_t* params, cmd_response_t* response) {
    /* 设置安全参数 */
    if (set_safety_parameters(params->safety_params.sensor_type,
                              params->safety_params.channel,
                              params->safety_params.alarm_high,
                              params->safety_params.alarm_low,
                              params->safety_params.shutdown_high,
                              params->safety_params.shutdown_low) == 0) {
        return 0;
    }
    return -1;
}

/* 系统复位处理器 */
static int system_reset_handler(const cmd_params_t* params, cmd_response_t* response) {
    /* 执行系统复位 */
    schedule_system_reset();
    return 0;
}

/* 保存配置处理器 */
static int save_config_handler(const cmd_params_t* params, cmd_response_t* response) {
    /* 保存配置到存储 */
    if (save_system_configuration() == 0) {
        return 0;
    }
    return -1;
}

//=============================================================================
// 7. 应用层使用示例
//=============================================================================

/* 定期发送传感器数据的任务 */
void sensor_report_task(void* parameter) {
    cmd_response_t response;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        /* 获取传感器数据 */
        if (read_sensor_data_handler(NULL, &response) == 0) {
            /* 发送传感器数据报告（协议无关） */
            app_send_command(UNIFIED_CMD_SENSOR_DATA_REPORT, &response);
        }

        /* 等待下一个周期 */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/* 命令处理任务 */
void command_processor_task(void* parameter) {
    unified_cmd_packet_t cmd_packet;
    cmd_params_t params;
    cmd_response_t response;

    while (1) {
        /* 从命令队列获取下行命令 */
        if (g_cmd_interface && g_cmd_interface->dequeue_downlink_cmd(&cmd_packet) == 0) {
            /* 解析命令参数 */
            memcpy(&params, cmd_packet.data, sizeof(cmd_params_t));

            /* 处理命令 */
            int result = app_process_command(cmd_packet.cmd, &params, &response);

            /* 发送响应 */
            unified_command_t ack_cmd = (result == 0) ? UNIFIED_CMD_ACK_SUCCESS : UNIFIED_CMD_ACK_ERROR;
            if (result != 0) {
                response.error_info.error_code = result;
                strcpy(response.error_info.error_message, "Command execution failed");
            }
            app_send_command(ack_cmd, &response);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//=============================================================================
// 8. EtherCAT专用功能实现
//=============================================================================

/* EtherCAT SDO写入命令 */
static int ethercat_sdo_write_command(unified_command_t cmd, const cmd_params_t* params) {
    /* 通过邮箱发送SDO写入请求 */
    uint16_t index = 0;
    uint8_t subindex = 0;
    uint8_t* data = NULL;
    uint32_t data_size = 0;

    /* 根据命令类型确定SDO参数 */
    switch (cmd) {
        case UNIFIED_CMD_SET_SAFETY_PARAMS:
            index = 0x4000;
            subindex = params->safety_params.channel;
            data = (uint8_t*)&params->safety_params;
            data_size = sizeof(params->safety_params);
            break;

        case UNIFIED_CMD_SYSTEM_RESET:
            index = 0x5000;
            subindex = 0x01;
            uint32_t reset_code = 0x12345678;
            data = (uint8_t*)&reset_code;
            data_size = sizeof(reset_code);
            break;

        case UNIFIED_CMD_SAVE_CONFIG:
            index = 0x5001;
            subindex = 0x01;
            uint8_t save_flag = 1;
            data = &save_flag;
            data_size = sizeof(save_flag);
            break;

        default:
            return -1;
    }

    /* 执行SDO写入 */
    return ec_slave_sdo_write(index, subindex, data, data_size);
}

/* EtherCAT紧急消息发送 */
static int ethercat_send_emergency_message(const void* alarm_data) {
    /* 发送EtherCAT紧急消息 */
    return ec_slave_send_emergency(0x8001, 0, (uint8_t*)alarm_data, 8);
}

//=============================================================================
// 9. 系统初始化和测试
//=============================================================================

/* 统一命令系统初始化 */
void unified_command_system_init(comm_protocol_t protocol) {
    /* 初始化命令处理器 */
    command_processor_init(protocol);

    /* 创建命令处理任务 */
    xTaskCreate(command_processor_task, "CmdProc", 2048, NULL, 10, NULL);
    xTaskCreate(sensor_report_task, "SensorReport", 1024, NULL, 8, NULL);
}

/* 命令一致性测试函数 */
void test_command_consistency(void) {
    cmd_params_t params;
    cmd_response_t response;

    /* 测试设置温度设定值 */
    params.setpoint.loop_id = 0;  // 温度回路1
    params.setpoint.setpoint = 75.5; // 75.5°C

    /* 无论是TCP还是EtherCAT，调用接口相同 */
    if (app_process_command(UNIFIED_CMD_SET_SETPOINT, &params, &response) == 0) {
        printf("Setpoint set successfully\n");
    }

    /* 测试读取传感器数据 */
    if (app_process_command(UNIFIED_CMD_READ_SENSOR_DATA, NULL, &response) == 0) {
        printf("Temperature: %.2f°C\n", response.sensor_data.temperature[0] / 100.0);
        printf("Pressure: %.1fkPa\n", response.sensor_data.pressure[0] / 10.0);
    }

    /* 测试手动控制 */
    params.manual_control.output_channel = 0; // 电磁阀1
    params.manual_control.output_value = 1;   // 开启
    params.manual_control.output_type = 0;    // 开关量

    if (app_process_command(UNIFIED_CMD_MANUAL_CONTROL, &params, &response) == 0) {
        printf("Manual control executed successfully\n");
    }
}