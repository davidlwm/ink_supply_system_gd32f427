/*
 * 墨路控制系统 - TCP/EtherCAT 双协议兼容通信代码
 * 基于 design1.txt 设计文档实现
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lwip/sockets.h"
#include "ethercat_slave.h"
#include "gd32f4xx.h"

//=============================================================================
// 1. 通用数据结构定义 (兼容性的核心)
//=============================================================================

/* 传感器数据统一结构 */
typedef struct {
    int16_t temperature[3];      // 温度值 (°C*100)
    uint16_t pressure[4];        // 压力值 (kPa*10)
    uint16_t level[4];           // 液位值 (mm*10)
    uint16_t flow_rate;          // 流量值 (L/min*100)
    uint8_t sensor_status;       // 传感器状态位
    uint32_t timestamp;          // 时间戳
} sensor_data_t;

/* 执行器控制统一结构 */
typedef struct {
    uint16_t valve_control[2];   // 电磁阀控制
    uint16_t heater_control[3];  // 加热器控制
    uint16_t pump_speed[2];      // 调速泵控制
    uint16_t pump_dc_control[2]; // 直流泵控制
    uint16_t control_word;       // 控制字
    uint8_t operation_mode;      // 操作模式
} actuator_control_t;

/* 系统状态统一结构 */
typedef struct {
    uint16_t system_state;       // 系统状态字
    uint16_t alarm_word;         // 告警状态字
    uint16_t fault_code;         // 故障代码
    uint8_t cpu_usage;           // CPU使用率
    uint8_t temperature_mcu;     // MCU温度
} system_status_t;

//=============================================================================
// 2. 协议抽象接口层 (兼容性的关键)
//=============================================================================

/* 通信协议类型 */
typedef enum {
    COMM_PROTOCOL_TCP = 0,
    COMM_PROTOCOL_ETHERCAT = 1
} comm_protocol_t;

/* 协议抽象接口 */
typedef struct {
    comm_protocol_t protocol_type;

    /* 初始化接口 */
    int (*init)(void);

    /* 数据发送接口 */
    int (*send_sensor_data)(const sensor_data_t* data);
    int (*send_status)(const system_status_t* status);

    /* 数据接收接口 */
    int (*receive_control_cmd)(actuator_control_t* control);
    int (*receive_config)(void* config_data, uint16_t config_type);

    /* 协议处理接口 */
    void (*process_task)(void);
    void (*error_handler)(uint16_t error_code);

    /* 状态查询接口 */
    bool (*is_connected)(void);
    uint32_t (*get_stats)(void);

} comm_interface_t;

/* 全局通信接口指针 */
static comm_interface_t* g_comm_interface = NULL;

//=============================================================================
// 3. TCP协议实现部分
//=============================================================================

/* TCP协议帧结构 (来自design1.txt) */
typedef struct {
    uint16_t sync_word;       // 同步字 0xAA55
    uint16_t length;          // 帧长度(包含帧头)
    uint8_t  cmd;             // 命令字
    uint8_t  index;           // 索引
    uint16_t checksum;        // 校验和
} __attribute__((packed)) tcp_frame_header_t;

typedef struct {
    tcp_frame_header_t header;
    uint8_t data[];
} __attribute__((packed)) tcp_protocol_frame_t;

/* TCP通信上下文 */
typedef struct {
    int server_socket;
    int client_socket;
    struct sockaddr_in server_addr;
    bool connected;
    uint32_t last_heartbeat;
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
} tcp_context_t;

static tcp_context_t tcp_ctx = {0};

/* TCP协议具体实现函数 */
static int tcp_init(void) {
    /* 创建socket */
    tcp_ctx.server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_ctx.server_socket < 0) {
        return -1;
    }

    /* 绑定地址 */
    tcp_ctx.server_addr.sin_family = AF_INET;
    tcp_ctx.server_addr.sin_addr.s_addr = INADDR_ANY;
    tcp_ctx.server_addr.sin_port = htons(8080);

    if (bind(tcp_ctx.server_socket, (struct sockaddr*)&tcp_ctx.server_addr,
             sizeof(tcp_ctx.server_addr)) < 0) {
        return -2;
    }

    /* 开始监听 */
    if (listen(tcp_ctx.server_socket, 1) < 0) {
        return -3;
    }

    /* 创建消息队列 */
    tcp_ctx.tx_queue = xQueueCreate(16, sizeof(tcp_protocol_frame_t*));
    tcp_ctx.rx_queue = xQueueCreate(16, sizeof(tcp_protocol_frame_t*));

    return 0;
}

static int tcp_send_sensor_data(const sensor_data_t* data) {
    if (!tcp_ctx.connected) return -1;

    /* 构造TCP数据包 */
    tcp_protocol_frame_t* frame = pvPortMalloc(sizeof(tcp_frame_header_t) + sizeof(sensor_data_t));

    frame->header.sync_word = 0xAA55;
    frame->header.cmd = 0x90;  // 实时数据上报命令
    frame->header.index = 0;
    frame->header.length = sizeof(tcp_frame_header_t) + sizeof(sensor_data_t);

    /* 拷贝传感器数据 */
    memcpy(frame->data, data, sizeof(sensor_data_t));

    /* 计算校验和 */
    frame->header.checksum = calculate_checksum((uint8_t*)frame, frame->header.length - 2);

    /* 发送数据 */
    int result = send(tcp_ctx.client_socket, frame, frame->header.length, 0);

    vPortFree(frame);
    return result > 0 ? 0 : -1;
}

static int tcp_send_status(const system_status_t* status) {
    if (!tcp_ctx.connected) return -1;

    /* 构造状态数据包 */
    tcp_protocol_frame_t* frame = pvPortMalloc(sizeof(tcp_frame_header_t) + sizeof(system_status_t));

    frame->header.sync_word = 0xAA55;
    frame->header.cmd = 0x92;  // 状态变化上报命令
    frame->header.index = 0;
    frame->header.length = sizeof(tcp_frame_header_t) + sizeof(system_status_t);

    memcpy(frame->data, status, sizeof(system_status_t));
    frame->header.checksum = calculate_checksum((uint8_t*)frame, frame->header.length - 2);

    int result = send(tcp_ctx.client_socket, frame, frame->header.length, 0);

    vPortFree(frame);
    return result > 0 ? 0 : -1;
}

static int tcp_receive_control_cmd(actuator_control_t* control) {
    if (!tcp_ctx.connected) return -1;

    tcp_protocol_frame_t* frame;
    if (xQueueReceive(tcp_ctx.rx_queue, &frame, 0) == pdTRUE) {
        /* 解析控制命令 */
        if (frame->header.cmd == 0x31) {  // 设定值设置命令
            memcpy(control, frame->data, sizeof(actuator_control_t));
            vPortFree(frame);
            return 0;
        }
        vPortFree(frame);
    }
    return -1;
}

static void tcp_process_task(void) {
    /* TCP服务器处理任务 */
    fd_set readfds;
    struct timeval timeout = {0, 100000}; // 100ms timeout

    FD_ZERO(&readfds);
    FD_SET(tcp_ctx.server_socket, &readfds);
    if (tcp_ctx.connected) {
        FD_SET(tcp_ctx.client_socket, &readfds);
    }

    int activity = select(FD_SETSIZE, &readfds, NULL, NULL, &timeout);

    if (activity > 0) {
        /* 处理新连接 */
        if (FD_ISSET(tcp_ctx.server_socket, &readfds)) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);

            tcp_ctx.client_socket = accept(tcp_ctx.server_socket,
                                         (struct sockaddr*)&client_addr, &client_len);
            if (tcp_ctx.client_socket >= 0) {
                tcp_ctx.connected = true;
            }
        }

        /* 处理客户端数据 */
        if (tcp_ctx.connected && FD_ISSET(tcp_ctx.client_socket, &readfds)) {
            tcp_protocol_frame_t frame_header;
            int bytes = recv(tcp_ctx.client_socket, &frame_header,
                           sizeof(tcp_frame_header_t), MSG_PEEK);

            if (bytes > 0) {
                /* 接收完整帧 */
                tcp_protocol_frame_t* frame = pvPortMalloc(frame_header.header.length);
                bytes = recv(tcp_ctx.client_socket, frame, frame_header.header.length, 0);

                if (bytes > 0) {
                    /* 验证校验和并放入接收队列 */
                    if (verify_checksum((uint8_t*)frame, frame->header.length)) {
                        xQueueSend(tcp_ctx.rx_queue, &frame, 0);
                    } else {
                        vPortFree(frame);
                    }
                }
            } else {
                /* 连接断开 */
                close(tcp_ctx.client_socket);
                tcp_ctx.connected = false;
            }
        }
    }
}

static bool tcp_is_connected(void) {
    return tcp_ctx.connected;
}

/* TCP协议接口实例 */
static comm_interface_t tcp_interface = {
    .protocol_type = COMM_PROTOCOL_TCP,
    .init = tcp_init,
    .send_sensor_data = tcp_send_sensor_data,
    .send_status = tcp_send_status,
    .receive_control_cmd = tcp_receive_control_cmd,
    .receive_config = NULL, // 简化实现
    .process_task = tcp_process_task,
    .error_handler = NULL,
    .is_connected = tcp_is_connected,
    .get_stats = NULL
};

//=============================================================================
// 4. EtherCAT协议实现部分
//=============================================================================

/* EtherCAT PDO结构 (来自design1.txt) */
typedef struct {
    /* 传感器数据区 */
    int16_t temperature[3];      // 温度值
    uint16_t pressure[4];        // 压力值
    uint16_t level[4];           // 液位值
    uint16_t flow_rate;          // 流量值
    uint8_t sensor_status;       // 传感器状态

    /* 系统状态区 */
    uint16_t system_state;       // 系统状态字
    uint16_t alarm_word;         // 告警状态字
    uint16_t fault_code;         // 故障代码
    uint8_t cpu_usage;           // CPU使用率
    uint8_t temperature_mcu;     // MCU温度

    /* 执行器反馈区 */
    uint16_t valve_feedback[2];
    uint16_t heater_feedback[3];
    uint16_t pump_speed_fb[2];
    uint16_t pump_dc_status[2];

    uint32_t timestamp;
} __attribute__((packed)) ethercat_input_pdo_t;

typedef struct {
    /* 控制命令区 */
    uint16_t control_word;
    uint8_t system_command;
    uint8_t operation_mode;
    int16_t temp_setpoint[3];
    uint16_t pressure_setpoint[4];
    uint16_t level_setpoint[3];
    uint16_t flow_setpoint;

    /* 执行器控制区 */
    uint16_t valve_control[2];
    uint16_t heater_control[3];
    uint16_t pump_speed[2];
    uint16_t pump_dc_control[2];

    /* PID参数区 */
    float kp, ki, kd;
    uint8_t loop_id;
    uint8_t pid_enable;
    uint8_t reserved[2];
} __attribute__((packed)) ethercat_output_pdo_t;

/* EtherCAT通信上下文 */
typedef struct {
    ethercat_input_pdo_t* input_pdo;      // 输入PDO指针
    ethercat_output_pdo_t* output_pdo;    // 输出PDO指针
    bool slave_ready;
    uint32_t cycle_count;
    uint32_t last_sync_time;
    SemaphoreHandle_t pdo_mutex;
} ethercat_context_t;

static ethercat_context_t ecat_ctx = {0};

/* EtherCAT协议具体实现函数 */
static int ethercat_init(void) {
    /* 初始化EtherCAT从站堆栈 */
    if (ec_slave_init() != 0) {
        return -1;
    }

    /* 获取PDO映射指针 */
    ecat_ctx.input_pdo = ec_slave_get_input_pdo();
    ecat_ctx.output_pdo = ec_slave_get_output_pdo();

    if (!ecat_ctx.input_pdo || !ecat_ctx.output_pdo) {
        return -2;
    }

    /* 创建同步信号量 */
    ecat_ctx.pdo_mutex = xSemaphoreCreateMutex();

    /* 注册SYNC0中断回调 */
    ec_slave_register_sync_callback(ethercat_sync0_callback);

    return 0;
}

static int ethercat_send_sensor_data(const sensor_data_t* data) {
    if (!ecat_ctx.slave_ready) return -1;

    /* 获取PDO互斥锁 */
    if (xSemaphoreTake(ecat_ctx.pdo_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        /* 直接更新输入PDO数据 */
        memcpy(ecat_ctx.input_pdo->temperature, data->temperature, sizeof(data->temperature));
        memcpy(ecat_ctx.input_pdo->pressure, data->pressure, sizeof(data->pressure));
        memcpy(ecat_ctx.input_pdo->level, data->level, sizeof(data->level));
        ecat_ctx.input_pdo->flow_rate = data->flow_rate;
        ecat_ctx.input_pdo->sensor_status = data->sensor_status;
        ecat_ctx.input_pdo->timestamp = data->timestamp;

        xSemaphoreGive(ecat_ctx.pdo_mutex);
        return 0;
    }
    return -1;
}

static int ethercat_send_status(const system_status_t* status) {
    if (!ecat_ctx.slave_ready) return -1;

    if (xSemaphoreTake(ecat_ctx.pdo_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        /* 更新系统状态到输入PDO */
        ecat_ctx.input_pdo->system_state = status->system_state;
        ecat_ctx.input_pdo->alarm_word = status->alarm_word;
        ecat_ctx.input_pdo->fault_code = status->fault_code;
        ecat_ctx.input_pdo->cpu_usage = status->cpu_usage;
        ecat_ctx.input_pdo->temperature_mcu = status->temperature_mcu;

        xSemaphoreGive(ecat_ctx.pdo_mutex);
        return 0;
    }
    return -1;
}

static int ethercat_receive_control_cmd(actuator_control_t* control) {
    if (!ecat_ctx.slave_ready) return -1;

    if (xSemaphoreTake(ecat_ctx.pdo_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        /* 从输出PDO读取控制命令 */
        memcpy(control->valve_control, ecat_ctx.output_pdo->valve_control,
               sizeof(control->valve_control));
        memcpy(control->heater_control, ecat_ctx.output_pdo->heater_control,
               sizeof(control->heater_control));
        memcpy(control->pump_speed, ecat_ctx.output_pdo->pump_speed,
               sizeof(control->pump_speed));
        memcpy(control->pump_dc_control, ecat_ctx.output_pdo->pump_dc_control,
               sizeof(control->pump_dc_control));

        control->control_word = ecat_ctx.output_pdo->control_word;
        control->operation_mode = ecat_ctx.output_pdo->operation_mode;

        xSemaphoreGive(ecat_ctx.pdo_mutex);
        return 0;
    }
    return -1;
}

static void ethercat_process_task(void) {
    /* EtherCAT处理任务 (主要是状态机处理) */
    ec_slave_process_state_machine();

    /* 检查从站状态 */
    ecat_ctx.slave_ready = (ec_slave_get_state() == EC_STATE_OP);

    /* 更新周期计数 */
    ecat_ctx.cycle_count++;
}

/* SYNC0中断回调 (1ms周期) */
void ethercat_sync0_callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* 记录同步时间 */
    ecat_ctx.last_sync_time = xTaskGetTickCountFromISR();

    /* 触发传感器数据采集任务 */
    vTaskNotifyGiveFromISR(sensor_task_handle, &xHigherPriorityTaskWoken);

    /* 触发控制任务 */
    vTaskNotifyGiveFromISR(control_task_handle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static bool ethercat_is_connected(void) {
    return ecat_ctx.slave_ready;
}

/* EtherCAT协议接口实例 */
static comm_interface_t ethercat_interface = {
    .protocol_type = COMM_PROTOCOL_ETHERCAT,
    .init = ethercat_init,
    .send_sensor_data = ethercat_send_sensor_data,
    .send_status = ethercat_send_status,
    .receive_control_cmd = ethercat_receive_control_cmd,
    .receive_config = NULL, // 通过邮箱通信实现
    .process_task = ethercat_process_task,
    .error_handler = NULL,
    .is_connected = ethercat_is_connected,
    .get_stats = NULL
};

//=============================================================================
// 5. 协议切换和初始化管理
//=============================================================================

/* 协议配置 */
typedef struct {
    comm_protocol_t protocol;
    bool auto_switch_enable;
    uint32_t switch_timeout;
} protocol_config_t;

static protocol_config_t g_protocol_config = {
    .protocol = COMM_PROTOCOL_TCP,  // 默认TCP
    .auto_switch_enable = false,
    .switch_timeout = 5000
};

/* 协议初始化函数 */
int communication_init(comm_protocol_t protocol) {
    /* 选择协议接口 */
    switch (protocol) {
        case COMM_PROTOCOL_TCP:
            g_comm_interface = &tcp_interface;
            break;
        case COMM_PROTOCOL_ETHERCAT:
            g_comm_interface = &ethercat_interface;
            break;
        default:
            return -1;
    }

    /* 初始化选中的协议 */
    return g_comm_interface->init();
}

/* 协议切换函数 */
int communication_switch_protocol(comm_protocol_t new_protocol) {
    if (new_protocol == g_protocol_config.protocol) {
        return 0; // 已经是目标协议
    }

    /* 停止当前协议 */
    // 这里应该有协议清理代码

    /* 切换到新协议 */
    g_protocol_config.protocol = new_protocol;
    return communication_init(new_protocol);
}

//=============================================================================
// 6. 应用层统一接口 (兼容性的体现)
//=============================================================================

/* 应用层发送传感器数据 */
int app_send_sensor_data(const sensor_data_t* data) {
    if (g_comm_interface && g_comm_interface->send_sensor_data) {
        return g_comm_interface->send_sensor_data(data);
    }
    return -1;
}

/* 应用层发送系统状态 */
int app_send_system_status(const system_status_t* status) {
    if (g_comm_interface && g_comm_interface->send_status) {
        return g_comm_interface->send_status(status);
    }
    return -1;
}

/* 应用层接收控制命令 */
int app_receive_control_command(actuator_control_t* control) {
    if (g_comm_interface && g_comm_interface->receive_control_cmd) {
        return g_comm_interface->receive_control_cmd(control);
    }
    return -1;
}

/* 应用层检查连接状态 */
bool app_is_communication_active(void) {
    if (g_comm_interface && g_comm_interface->is_connected) {
        return g_comm_interface->is_connected();
    }
    return false;
}

//=============================================================================
// 7. 任务实现示例
//=============================================================================

/* 通信处理任务 */
void communication_task(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms周期

    while (1) {
        /* 处理协议相关事务 */
        if (g_comm_interface && g_comm_interface->process_task) {
            g_comm_interface->process_task();
        }

        /* 等待下一个周期 */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* 传感器数据发送任务 */
void sensor_data_task(void* parameter) {
    sensor_data_t sensor_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms周期

    while (1) {
        /* 采集传感器数据 */
        collect_sensor_data(&sensor_data);

        /* 发送数据 (自动适配协议) */
        app_send_sensor_data(&sensor_data);

        /* 等待下一个周期 */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* 控制命令处理任务 */
void control_command_task(void* parameter) {
    actuator_control_t control_cmd;

    while (1) {
        /* 接收控制命令 (自动适配协议) */
        if (app_receive_control_command(&control_cmd) == 0) {
            /* 执行控制命令 */
            execute_actuator_control(&control_cmd);
        }

        /* 短暂延时 */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//=============================================================================
// 8. 系统初始化
//=============================================================================

void system_communication_init(void) {
    /* 读取配置决定使用哪种协议 */
    comm_protocol_t protocol = read_protocol_config();

    /* 初始化通信协议 */
    if (communication_init(protocol) != 0) {
        /* 初始化失败，尝试备用协议 */
        comm_protocol_t backup_protocol = (protocol == COMM_PROTOCOL_TCP) ?
                                         COMM_PROTOCOL_ETHERCAT : COMM_PROTOCOL_TCP;
        communication_init(backup_protocol);
    }

    /* 创建通信相关任务 */
    xTaskCreate(communication_task, "CommTask", 2048, NULL, 5, NULL);
    xTaskCreate(sensor_data_task, "SensorTask", 1024, NULL, 8, NULL);
    xTaskCreate(control_command_task, "ControlTask", 1024, NULL, 12, NULL);
}

//=============================================================================
// 9. 辅助函数
//=============================================================================

/* CRC校验计算 */
static uint16_t calculate_checksum(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* CRC校验验证 */
static bool verify_checksum(const uint8_t* data, uint16_t length) {
    uint16_t calculated_crc = calculate_checksum(data, length - 2);
    uint16_t received_crc = *(uint16_t*)(data + length - 2);
    return calculated_crc == received_crc;
}

/* 传感器数据采集 (模拟实现) */
static void collect_sensor_data(sensor_data_t* data) {
    /* 这里应该是实际的ADC读取和数据处理代码 */
    // 温度采集
    data->temperature[0] = adc_read_temperature(0) * 100;
    data->temperature[1] = adc_read_temperature(1) * 100;
    data->temperature[2] = adc_read_temperature(2) * 100;

    // 压力采集
    data->pressure[0] = adc_read_pressure(0) * 10;
    data->pressure[1] = adc_read_pressure(1) * 10;
    data->pressure[2] = adc_read_pressure(2) * 10;
    data->pressure[3] = adc_read_pressure(3) * 10;

    // 液位采集
    data->level[0] = adc_read_level(0) * 10;
    data->level[1] = adc_read_level(1) * 10;
    data->level[2] = adc_read_level(2) * 10;
    data->level[3] = adc_read_level_analog() * 10;

    // 流量采集 (I2C)
    data->flow_rate = i2c_read_flow_sensor() * 100;

    // 传感器状态
    data->sensor_status = get_sensor_health_status();
    data->timestamp = xTaskGetTickCount();
}

/* 执行器控制执行 (模拟实现) */
static void execute_actuator_control(const actuator_control_t* control) {
    /* 这里应该是实际的执行器控制代码 */
    // 电磁阀控制
    gpio_write_valve(0, control->valve_control[0]);
    gpio_write_valve(1, control->valve_control[1]);

    // 加热器控制
    relay_control_heater(0, control->heater_control[0]);
    relay_control_heater(1, control->heater_control[1]);
    relay_control_heater(2, control->heater_control[2]);

    // 泵控制
    pwm_set_pump_speed(0, control->pump_speed[0]);
    pwm_set_pump_speed(1, control->pump_speed[1]);
    gpio_write_dc_pump(0, control->pump_dc_control[0]);
    gpio_write_dc_pump(1, control->pump_dc_control[1]);
}

/* 协议配置读取 */
static comm_protocol_t read_protocol_config(void) {
    /* 从EEPROM或Flash读取配置 */
    // 这里简化为返回默认值
    return COMM_PROTOCOL_TCP;
}