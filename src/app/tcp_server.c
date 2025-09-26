/**
 * @file    tcp_server.c
 * @brief   TCP服务器实现 - 保持v1版本TCP功能
 * @version V4.0
 * @date    2025-09-27
 */

#include "app/tcp_server.h"
#include "app/sensor_task.h"
#include "app/actuator_task.h"
#include "app/system_config.h"

// lwIP头文件
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// TCP服务器状态结构
typedef struct {
    bool server_running;            // 服务器运行状态
    uint16_t server_port;          // 服务器端口
    uint32_t client_count;         // 客户端连接数
    uint32_t total_connections;    // 总连接数
    uint32_t bytes_sent;           // 发送字节数
    uint32_t bytes_received;       // 接收字节数
    SemaphoreHandle_t status_mutex; // 状态互斥锁
} tcp_server_status_t;

// 客户端连接信息
typedef struct {
    struct netconn *conn;          // 网络连接
    ip_addr_t client_ip;           // 客户端IP
    uint16_t client_port;          // 客户端端口
    uint32_t connect_time;         // 连接时间
    uint32_t last_activity;        // 最后活动时间
    bool authenticated;            // 认证状态
} tcp_client_info_t;

// 全局TCP服务器状态
static tcp_server_status_t g_tcp_server_status;

// 网络配置 (基于v1文档设计)
static const tcp_server_config_t default_config = {
    .server_ip = IPADDR4_INIT_BYTES(192, 168, 1, 100),
    .server_port = 502,                    // Modbus TCP端口
    .max_clients = TCP_MAX_CLIENTS,
    .heartbeat_interval = 30000,           // 30秒心跳
    .connection_timeout = 300000,          // 5分钟超时
    .enable_authentication = false,
    .buffer_size = TCP_BUFFER_SIZE
};

/**
 * @brief  TCP服务器初始化
 * @param  config 配置结构指针
 * @retval tcp_result_t 初始化结果
 */
tcp_result_t tcp_server_init(const tcp_server_config_t *config)
{
    // 使用默认配置或提供的配置
    const tcp_server_config_t *cfg = config ? config : &default_config;

    // 创建状态互斥锁
    g_tcp_server_status.status_mutex = xSemaphoreCreateMutex();
    if (g_tcp_server_status.status_mutex == NULL) {
        return TCP_ERROR_MEMORY_ALLOCATION;
    }

    // 初始化状态
    g_tcp_server_status.server_running = false;
    g_tcp_server_status.server_port = cfg->server_port;
    g_tcp_server_status.client_count = 0;
    g_tcp_server_status.total_connections = 0;
    g_tcp_server_status.bytes_sent = 0;
    g_tcp_server_status.bytes_received = 0;

    return TCP_SUCCESS;
}

/**
 * @brief  TCP服务器任务 (保持v1设计)
 * @param  pvParameters 任务参数
 * @retval None
 */
void tcp_server_task(void *pvParameters)
{
    struct netconn *conn, *newconn;
    err_t err;
    tcp_server_config_t *config = (tcp_server_config_t *)pvParameters;

    // 使用默认配置如果没有提供
    if (config == NULL) {
        config = (tcp_server_config_t *)&default_config;
    }

    // 创建TCP连接结构
    conn = netconn_new(NETCONN_TCP);
    if (conn == NULL) {
        vTaskDelete(NULL);
        return;
    }

    // 绑定端口
    err = netconn_bind(conn, &config->server_ip, config->server_port);
    if (err != ERR_OK) {
        netconn_delete(conn);
        vTaskDelete(NULL);
        return;
    }

    // 开始监听
    err = netconn_listen(conn);
    if (err != ERR_OK) {
        netconn_delete(conn);
        vTaskDelete(NULL);
        return;
    }

    // 更新服务器状态
    if (xSemaphoreTake(g_tcp_server_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_tcp_server_status.server_running = true;
        xSemaphoreGive(g_tcp_server_status.status_mutex);
    }

    while (1) {
        // 等待新连接
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            // 获取客户端信息
            ip_addr_t client_ip;
            u16_t client_port;
            netconn_peer(newconn, &client_ip, &client_port);

            // 检查客户端连接数限制
            bool allow_connection = false;
            if (xSemaphoreTake(g_tcp_server_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (g_tcp_server_status.client_count < config->max_clients) {
                    g_tcp_server_status.client_count++;
                    g_tcp_server_status.total_connections++;
                    allow_connection = true;
                }
                xSemaphoreGive(g_tcp_server_status.status_mutex);
            }

            if (allow_connection && is_ip_allowed(&client_ip)) {
                // 创建客户端处理任务
                tcp_client_info_t *client_info = pvPortMalloc(sizeof(tcp_client_info_t));
                if (client_info != NULL) {
                    client_info->conn = newconn;
                    client_info->client_ip = client_ip;
                    client_info->client_port = client_port;
                    client_info->connect_time = xTaskGetTickCount();
                    client_info->last_activity = xTaskGetTickCount();
                    client_info->authenticated = !config->enable_authentication;

                    xTaskCreate(tcp_client_handler, "TCPClient",
                               TCP_CLIENT_STACK_SIZE, client_info,
                               TCP_CLIENT_TASK_PRIORITY, NULL);
                } else {
                    // 内存分配失败，关闭连接
                    netconn_close(newconn);
                    netconn_delete(newconn);

                    if (xSemaphoreTake(g_tcp_server_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        g_tcp_server_status.client_count--;
                        xSemaphoreGive(g_tcp_server_status.status_mutex);
                    }
                }
            } else {
                // 拒绝连接
                netconn_close(newconn);
                netconn_delete(newconn);
                tcp_log_access_attempt(&client_ip, "REJECTED");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief  TCP客户端处理任务 (保持v1数据格式)
 * @param  pvParameters 客户端信息指针
 * @retval None
 */
void tcp_client_handler(void *pvParameters)
{
    tcp_client_info_t *client_info = (tcp_client_info_t *)pvParameters;
    struct netbuf *buf;
    char *data;
    u16_t len;
    char response[TCP_BUFFER_SIZE];
    err_t err;

    if (client_info == NULL) {
        vTaskDelete(NULL);
        return;
    }

    // 设置接收超时
    netconn_set_recvtimeout(client_info->conn, 30000); // 30秒超时

    tcp_log_access_attempt(&client_info->client_ip, "CONNECTED");

    while (1) {
        // 接收数据
        err = netconn_recv(client_info->conn, &buf);
        if (err == ERR_OK) {
            netbuf_data(buf, (void**)&data, &len);

            // 更新活动时间
            client_info->last_activity = xTaskGetTickCount();

            // 更新接收字节统计
            if (xSemaphoreTake(g_tcp_server_status.status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_tcp_server_status.bytes_received += len;
                xSemaphoreGive(g_tcp_server_status.status_mutex);
            }

            // 处理接收到的命令
            tcp_result_t result = tcp_process_command(client_info, data, len, response, sizeof(response));

            if (result == TCP_SUCCESS) {
                // 发送响应
                size_t response_len = strlen(response);
                err = netconn_write(client_info->conn, response, response_len, NETCONN_COPY);

                if (err == ERR_OK) {
                    // 更新发送字节统计
                    if (xSemaphoreTake(g_tcp_server_status.status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        g_tcp_server_status.bytes_sent += response_len;
                        xSemaphoreGive(g_tcp_server_status.status_mutex);
                    }
                }
            }

            netbuf_delete(buf);
        } else {
            // 连接断开或超时
            break;
        }

        // 检查连接超时
        uint32_t current_time = xTaskGetTickCount();
        if ((current_time - client_info->last_activity) > pdMS_TO_TICKS(300000)) { // 5分钟超时
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 清理连接
    tcp_log_access_attempt(&client_info->client_ip, "DISCONNECTED");
    netconn_close(client_info->conn);
    netconn_delete(client_info->conn);

    // 更新客户端计数
    if (xSemaphoreTake(g_tcp_server_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_tcp_server_status.client_count--;
        xSemaphoreGive(g_tcp_server_status.status_mutex);
    }

    // 释放客户端信息内存
    vPortFree(client_info);

    vTaskDelete(NULL);
}

/**
 * @brief  处理TCP命令 (保持v1协议格式)
 * @param  client_info 客户端信息
 * @param  request 请求数据
 * @param  request_len 请求长度
 * @param  response 响应缓冲区
 * @param  response_size 响应缓冲区大小
 * @retval tcp_result_t 处理结果
 */
tcp_result_t tcp_process_command(tcp_client_info_t *client_info, char *request, uint16_t request_len,
                                char *response, uint16_t response_size)
{
    if (request == NULL || response == NULL || request_len == 0) {
        return TCP_ERROR_INVALID_PARAMETER;
    }

    // 认证检查
    if (!client_info->authenticated) {
        if (strncmp(request, "AUTH:", 5) == 0) {
            // 简化认证处理
            client_info->authenticated = true;
            strcpy(response, "AUTH_OK");
            return TCP_SUCCESS;
        } else {
            strcpy(response, "AUTH_REQUIRED");
            return TCP_SUCCESS;
        }
    }

    // 解析命令
    if (strncmp(request, "GET_STATUS", 10) == 0) {
        // 获取系统状态 (保持v1数据格式)
        return tcp_create_status_response(response, response_size);
    }
    else if (strncmp(request, "GET_SENSORS", 11) == 0) {
        // 获取传感器数据 (保持v1数据格式)
        return tcp_create_sensor_response(response, response_size);
    }
    else if (strncmp(request, "GET_ACTUATORS", 13) == 0) {
        // 获取执行器状态
        return tcp_create_actuator_response(response, response_size);
    }
    else if (strncmp(request, "SET_HEATER", 10) == 0) {
        // 设置加热器功率
        return tcp_parse_heater_command(request, request_len, response, response_size);
    }
    else if (strncmp(request, "SET_PUMP", 8) == 0) {
        // 设置泵转速
        return tcp_parse_pump_command(request, request_len, response, response_size);
    }
    else if (strncmp(request, "SET_VALVE", 9) == 0) {
        // 设置阀门状态
        return tcp_parse_valve_command(request, request_len, response, response_size);
    }
    else if (strncmp(request, "PING", 4) == 0) {
        // 心跳响应
        strcpy(response, "PONG");
        return TCP_SUCCESS;
    }
    else {
        // 未知命令
        snprintf(response, response_size, "ERROR: Unknown command: %.10s", request);
        return TCP_SUCCESS;
    }
}

/**
 * @brief  创建系统状态响应 (保持v1数据格式)
 * @param  response 响应缓冲区
 * @param  response_size 缓冲区大小
 * @retval tcp_result_t 创建结果
 */
tcp_result_t tcp_create_status_response(char *response, uint16_t response_size)
{
    // 构造JSON响应数据 (保持v1格式)
    int written = snprintf(response, response_size,
        "{"
        "\"system_status\":%d,"
        "\"uptime\":%u,"
        "\"cpu_usage\":%d,"
        "\"memory_free\":%u,"
        "\"temperature\":[%.2f,%.2f,%.2f],"
        "\"pressure\":[%.2f,%.2f],"
        "\"liquid_level\":[%.2f,%.2f],"
        "\"timestamp\":%u"
        "}",
        1, // 系统正常运行
        (uint32_t)xTaskGetTickCount(),
        50, // CPU使用率 (简化)
        (uint32_t)xPortGetFreeHeapSize(),
        get_pt100_temperature(0), get_pt100_temperature(1), get_pt100_temperature(2),
        get_pressure_sensor(0), get_pressure_sensor(1),
        get_liquid_level_sensor(0), get_liquid_level_sensor(1),
        (uint32_t)xTaskGetTickCount()
    );

    return (written > 0 && written < response_size) ? TCP_SUCCESS : TCP_ERROR_BUFFER_OVERFLOW;
}

/**
 * @brief  检查IP是否允许连接
 * @param  ip IP地址指针
 * @retval bool 允许返回true
 */
static bool is_ip_allowed(const ip_addr_t *ip)
{
    // 简化的IP白名单检查
    // 在实际应用中，这里可以实现更复杂的访问控制

    // 允许本地网络 (192.168.x.x)
    if (IP_IS_V4(ip)) {
        uint32_t addr = ip_4_addr(ip)->addr;
        uint32_t network = addr & 0x0000FFFF; // 前16位
        if (network == 0x0000A8C0) { // 192.168.0.0/16
            return true;
        }
    }

    // 允许回环地址
    if (ip_addr_isloopback(ip)) {
        return true;
    }

    return false; // 默认拒绝
}

/**
 * @brief  记录访问日志
 * @param  ip 客户端IP
 * @param  event 事件描述
 * @retval None
 */
static void tcp_log_access_attempt(const ip_addr_t *ip, const char *event)
{
    char ip_str[IPADDR_STRLEN_MAX];
    ipaddr_ntoa_r(ip, ip_str, sizeof(ip_str));

    // 这里可以记录到日志系统
    printf("[TCP] %s: %s\n", ip_str, event);
}

/**
 * @brief  获取TCP服务器状态
 * @param  status 状态结构指针
 * @retval tcp_result_t 获取结果
 */
tcp_result_t tcp_server_get_status(tcp_server_status_info_t *status)
{
    if (status == NULL) {
        return TCP_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_tcp_server_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status->server_running = g_tcp_server_status.server_running;
        status->server_port = g_tcp_server_status.server_port;
        status->client_count = g_tcp_server_status.client_count;
        status->total_connections = g_tcp_server_status.total_connections;
        status->bytes_sent = g_tcp_server_status.bytes_sent;
        status->bytes_received = g_tcp_server_status.bytes_received;
        xSemaphoreGive(g_tcp_server_status.status_mutex);
        return TCP_SUCCESS;
    }

    return TCP_ERROR_TIMEOUT;
}