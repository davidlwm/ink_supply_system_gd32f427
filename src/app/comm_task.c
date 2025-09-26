/**
 * @file    comm_task.c
 * @brief   通信任务主控制器 - 管理TCP/IP和EtherCAT双协议
 * @version V4.0
 * @date    2025-09-27
 */

#include "app/comm_task.h"
#include "app/tcp_server.h"
#include "app/ethercat_app.h"
#include "app/system_config.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

// lwIP头文件
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "netif/etharp.h"

// 通信系统状态
typedef struct {
    bool tcp_server_running;       // TCP服务器运行状态
    bool ethercat_running;         // EtherCAT运行状态
    bool network_initialized;     // 网络初始化状态
    uint32_t tcp_client_count;    // TCP客户端数量
    uint32_t ethercat_cycle_count; // EtherCAT循环计数
    uint32_t total_tx_bytes;      // 总发送字节数
    uint32_t total_rx_bytes;      // 总接收字节数
    SemaphoreHandle_t status_mutex; // 状态互斥锁
} comm_system_status_t;

static comm_system_status_t g_comm_status = {0};

// 网络接口
static struct netif gnetif;

// 任务句柄
static TaskHandle_t tcp_server_task_handle = NULL;
static TaskHandle_t ethercat_task_handle = NULL;

// 网络监控定时器
static TimerHandle_t network_monitor_timer = NULL;

/**
 * @brief  通信系统初始化
 * @param  None
 * @retval comm_result_t 初始化结果
 */
comm_result_t comm_manager_init(void)
{
    // 创建状态互斥锁
    g_comm_status.status_mutex = xSemaphoreCreateMutex();
    if (g_comm_status.status_mutex == NULL) {
        return COMM_ERROR_MEMORY_ALLOCATION;
    }

    // 初始化状态
    g_comm_status.tcp_server_running = false;
    g_comm_status.ethercat_running = false;
    g_comm_status.network_initialized = false;
    g_comm_status.tcp_client_count = 0;
    g_comm_status.ethercat_cycle_count = 0;
    g_comm_status.total_tx_bytes = 0;
    g_comm_status.total_rx_bytes = 0;

    return COMM_SUCCESS;
}

/**
 * @brief  通信主任务
 * @param  pvParameters 任务参数
 * @retval None
 */
void comm_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    uint32_t task_counter = 0;

    // 等待系统初始化完成
    vTaskDelay(pdMS_TO_TICKS(200));

    // 初始化通信系统
    if (comm_manager_init() != COMM_SUCCESS) {
        vTaskDelete(NULL);
        return;
    }

    // 初始化网络系统
    if (comm_network_init() != COMM_SUCCESS) {
        vTaskDelete(NULL);
        return;
    }

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 50ms周期执行
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(COMM_TASK_PERIOD_MS));

        task_counter++;

        // 1. 监控网络状态
        comm_monitor_network_status();

        // 2. 监控通信任务状态
        comm_monitor_tasks_status();

        // 3. 更新通信统计信息
        comm_update_statistics();

        // 4. 每秒执行一次维护操作
        if (task_counter % (1000 / COMM_TASK_PERIOD_MS) == 0) {
            comm_periodic_maintenance();
        }

        // 5. 每10秒报告一次状态
        if (task_counter % (10000 / COMM_TASK_PERIOD_MS) == 0) {
            comm_report_status();
        }
    }
}

/**
 * @brief  网络系统初始化
 * @param  None
 * @retval comm_result_t 初始化结果
 */
static comm_result_t comm_network_init(void)
{
    ip4_addr_t ipaddr, netmask, gw;

    // 初始化lwIP TCP/IP协议栈
    tcpip_init(NULL, NULL);

    // 配置网络参数
    IP4_ADDR(&ipaddr, 192, 168, 1, 100);  // 控制器IP
    IP4_ADDR(&netmask, 255, 255, 255, 0); // 子网掩码
    IP4_ADDR(&gw, 192, 168, 1, 1);        // 网关

    // 添加网络接口
    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

    // 注册默认网络接口
    netif_set_default(&gnetif);

    // 启用网络接口
    if (netif_is_link_up(&gnetif)) {
        netif_set_up(&gnetif);
    } else {
        netif_set_down(&gnetif);
    }

    // 设置链路状态改变回调
    netif_set_link_callback(&gnetif, comm_link_callback);

    // 更新状态
    if (xSemaphoreTake(g_comm_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_comm_status.network_initialized = true;
        xSemaphoreGive(g_comm_status.status_mutex);
    }

    // 创建TCP服务器任务
    BaseType_t result = xTaskCreate(
        tcp_server_task,
        "TCPServer",
        TCP_SERVER_STACK_SIZE,
        NULL,
        TCP_SERVER_TASK_PRIORITY,
        &tcp_server_task_handle
    );

    if (result != pdPASS) {
        return COMM_ERROR_TASK_CREATE;
    }

    // 创建EtherCAT任务
    result = xTaskCreate(
        ethercat_app_task,
        "EtherCAT",
        ETHERCAT_STACK_SIZE,
        NULL,
        ETHERCAT_PRIORITY,
        &ethercat_task_handle
    );

    if (result != pdPASS) {
        return COMM_ERROR_TASK_CREATE;
    }

    // 创建网络监控定时器 (每5秒)
    network_monitor_timer = xTimerCreate(
        "NetMonitor",
        pdMS_TO_TICKS(5000),
        pdTRUE, // 自动重载
        NULL,
        comm_network_monitor_callback
    );

    if (network_monitor_timer != NULL) {
        xTimerStart(network_monitor_timer, 0);
    }

    return COMM_SUCCESS;
}

/**
 * @brief  监控网络状态
 * @param  None
 * @retval None
 */
static void comm_monitor_network_status(void)
{
    // 检查网络接口状态
    bool link_up = netif_is_link_up(&gnetif);
    bool interface_up = netif_is_up(&gnetif);

    if (!link_up || !interface_up) {
        // 网络断开，尝试重新连接
        if (link_up && !interface_up) {
            netif_set_up(&gnetif);
        }
    }
}

/**
 * @brief  监控通信任务状态
 * @param  None
 * @retval None
 */
static void comm_monitor_tasks_status(void)
{
    bool tcp_running = (tcp_server_task_handle != NULL) &&
                       (eTaskGetState(tcp_server_task_handle) == eRunning);

    bool ethercat_running = (ethercat_task_handle != NULL) &&
                           (eTaskGetState(ethercat_task_handle) == eRunning);

    if (xSemaphoreTake(g_comm_status.status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_comm_status.tcp_server_running = tcp_running;
        g_comm_status.ethercat_running = ethercat_running;
        xSemaphoreGive(g_comm_status.status_mutex);
    }
}

/**
 * @brief  更新通信统计信息
 * @param  None
 * @retval None
 */
static void comm_update_statistics(void)
{
    tcp_server_status_info_t tcp_status;
    ethercat_status_info_t ethercat_status;

    // 获取TCP服务器状态
    if (tcp_server_get_status(&tcp_status) == TCP_SUCCESS) {
        if (xSemaphoreTake(g_comm_status.status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_comm_status.tcp_client_count = tcp_status.client_count;
            g_comm_status.total_tx_bytes += tcp_status.bytes_sent;
            g_comm_status.total_rx_bytes += tcp_status.bytes_received;
            xSemaphoreGive(g_comm_status.status_mutex);
        }
    }

    // 获取EtherCAT状态
    if (ethercat_get_status(&ethercat_status) == ETHERCAT_SUCCESS) {
        if (xSemaphoreTake(g_comm_status.status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_comm_status.ethercat_cycle_count = ethercat_status.cycle_count;
            xSemaphoreGive(g_comm_status.status_mutex);
        }
    }
}

/**
 * @brief  定期维护操作
 * @param  None
 * @retval None
 */
static void comm_periodic_maintenance(void)
{
    // 清理无效的网络连接
    // 检查内存使用情况
    // 更新网络统计信息

    size_t free_heap = xPortGetFreeHeapSize();
    if (free_heap < 8192) { // 少于8KB可用内存时警告
        printf("[COMM] Warning: Low memory, free heap: %u bytes\n", free_heap);
    }
}

/**
 * @brief  报告通信系统状态
 * @param  None
 * @retval None
 */
static void comm_report_status(void)
{
    comm_system_status_t status;

    if (xSemaphoreTake(g_comm_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status = g_comm_status;
        xSemaphoreGive(g_comm_status.status_mutex);

        printf("[COMM] Status: TCP=%s EtherCAT=%s Clients=%u Cycles=%u\n",
               status.tcp_server_running ? "UP" : "DOWN",
               status.ethercat_running ? "UP" : "DOWN",
               status.tcp_client_count,
               status.ethercat_cycle_count);
    }
}

/**
 * @brief  网络链路状态回调
 * @param  netif 网络接口指针
 * @retval None
 */
static void comm_link_callback(struct netif *netif)
{
    if (netif_is_link_up(netif)) {
        printf("[COMM] Network link UP\n");
        netif_set_up(netif);
    } else {
        printf("[COMM] Network link DOWN\n");
        netif_set_down(netif);
    }
}

/**
 * @brief  网络监控定时器回调
 * @param  xTimer 定时器句柄
 * @retval None
 */
static void comm_network_monitor_callback(TimerHandle_t xTimer)
{
    // 检查网络状态并记录统计信息
    struct netif *netif = netif_default;
    if (netif != NULL) {
        printf("[COMM] Network stats: IP=%s Link=%s\n",
               ip4addr_ntoa(netif_ip4_addr(netif)),
               netif_is_link_up(netif) ? "UP" : "DOWN");
    }
}

/**
 * @brief  获取通信系统状态
 * @param  status 状态结构指针
 * @retval comm_result_t 获取结果
 */
comm_result_t comm_get_system_status(comm_system_status_info_t *status)
{
    if (status == NULL) {
        return COMM_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_comm_status.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status->tcp_server_running = g_comm_status.tcp_server_running;
        status->ethercat_running = g_comm_status.ethercat_running;
        status->network_initialized = g_comm_status.network_initialized;
        status->tcp_client_count = g_comm_status.tcp_client_count;
        status->ethercat_cycle_count = g_comm_status.ethercat_cycle_count;
        status->total_tx_bytes = g_comm_status.total_tx_bytes;
        status->total_rx_bytes = g_comm_status.total_rx_bytes;
        xSemaphoreGive(g_comm_status.status_mutex);
        return COMM_SUCCESS;
    }

    return COMM_ERROR_TIMEOUT;
}