/**
 * @file    ethercat_app.c
 * @brief   EtherCAT应用实现 - 保持v1版本EtherCAT功能
 * @version V4.0
 * @date    2025-09-27
 */

#include "app/ethercat_app.h"
#include "app/sensor_task.h"
#include "app/actuator_task.h"
#include "app/system_config.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// EtherCAT协议栈头文件 (使用开源SSC)
#include "app/esc.h"
#include "app/ecat_slv.h"
#include "app/ecatappl.h"

// EtherCAT过程数据定义 (保持v1文档完全一致)
typedef struct {
    // 输入数据 (传感器 -> 主站) - 16字节
    struct {
        // 模拟量输入 (12字节)
        uint16_t liquid_level_1;     // 液位传感器1 (0-2000mm, 0.1mm精度)
        uint16_t liquid_level_2;     // 液位传感器2 (0-2000mm, 0.1mm精度)
        uint16_t pressure_1;         // 压力传感器1 (±100MPa, 0.01MPa精度)
        uint16_t pressure_2;         // 压力传感器2 (±100MPa, 0.01MPa精度)
        uint16_t temperature_1;      // 温度传感器1 (0-600°C, 0.1°C精度)
        uint16_t temperature_2;      // 温度传感器2 (0-600°C, 0.1°C精度)

        // 数字量输入 (2字节)
        uint16_t digital_inputs;     // 开关量状态位

        // 状态信息 (2字节)
        uint16_t system_status;      // 系统状态字
        uint16_t fault_code;         // 故障代码
    } inputs;

    // 输出数据 (主站 -> 执行器) - 12字节
    struct {
        // 模拟量输出 (8字节)
        uint16_t heater_power_1;     // 加热器1功率 (0-1000, 0.1%精度)
        uint16_t heater_power_2;     // 加热器2功率 (0-1000, 0.1%精度)
        uint16_t pump_speed_1;       // 泵1转速 (200-5000RPM)
        uint16_t pump_speed_2;       // 泵2转速 (200-5000RPM)

        // 数字量输出 (2字节)
        uint16_t digital_outputs;    // 阀门控制位

        // 控制命令 (2字节)
        uint16_t control_command;    // 系统控制命令
    } outputs;

} __attribute__((packed)) ethercat_process_data_t;

// 全局EtherCAT过程数据
static ethercat_process_data_t g_ethercat_data;

// EtherCAT状态管理
typedef struct {
    bool initialized;              // 初始化状态
    bool link_up;                 // 链路状态
    uint8_t al_state;            // 应用层状态
    uint32_t cycle_count;        // 循环计数
    uint32_t error_count;        // 错误计数
    uint32_t max_cycle_time_us;  // 最大循环时间(微秒)
    SemaphoreHandle_t data_mutex; // 数据互斥锁
} ethercat_status_t;

static ethercat_status_t g_ethercat_status = {0};

// EtherCAT任务参数 (保持v1严格实时性)
#define ETHERCAT_CYCLE_TIME_MS      1       // 1ms严格周期
#define ETHERCAT_PRIORITY           8       // 最高优先级(高于安全任务)
#define ETHERCAT_STACK_SIZE         2048    // 栈大小
#define ETHERCAT_SYNC_TIMEOUT_US    50      // 同步超时50微秒

/**
 * @brief  EtherCAT应用初始化
 * @param  None
 * @retval ethercat_result_t 初始化结果
 */
ethercat_result_t ethercat_app_init(void)
{
    // 创建数据互斥锁
    g_ethercat_status.data_mutex = xSemaphoreCreateMutex();
    if (g_ethercat_status.data_mutex == NULL) {
        return ETHERCAT_ERROR_MEMORY_ALLOCATION;
    }

    // 初始化EtherCAT ESC硬件
    if (ESC_Init() != 0) {
        return ETHERCAT_ERROR_HARDWARE_INIT;
    }

    // 初始化应用层
    ECAT_Application_Init();

    // 清零过程数据
    memset(&g_ethercat_data, 0, sizeof(ethercat_process_data_t));

    // 初始化状态
    g_ethercat_status.initialized = true;
    g_ethercat_status.link_up = false;
    g_ethercat_status.al_state = 0;
    g_ethercat_status.cycle_count = 0;
    g_ethercat_status.error_count = 0;
    g_ethercat_status.max_cycle_time_us = 0;

    return ETHERCAT_SUCCESS;
}

/**
 * @brief  EtherCAT应用任务 (保持v1设计，严格1ms周期)
 * @param  pvParameters 任务参数
 * @retval None
 */
void ethercat_app_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    uint32_t cycle_start_time, cycle_end_time, cycle_duration;
    uint32_t cycle_overrun_count = 0;
    uint32_t performance_report_counter = 0;

    // 等待系统初始化完成
    vTaskDelay(pdMS_TO_TICKS(100));

    // 初始化EtherCAT
    if (ethercat_app_init() != ETHERCAT_SUCCESS) {
        vTaskDelete(NULL);
        return;
    }

    // 设置任务为最高优先级
    vTaskPrioritySet(NULL, ETHERCAT_PRIORITY);

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        cycle_start_time = DWT->CYCCNT; // 使用DWT计数器获取高精度时间

        // 严格1ms周期延时 (保持v1实时性要求)
        BaseType_t xWasDelayed = vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ETHERCAT_CYCLE_TIME_MS));

        if (xWasDelayed == pdFALSE) {
            cycle_overrun_count++;
            g_ethercat_status.error_count++;
        }

        // EtherCAT数据处理 (关键实时代码)
        taskENTER_CRITICAL();

        // 1. EtherCAT主循环处理
        ECAT_Application();

        // 2. 更新传感器数据到EtherCAT输入 (保持v1接口)
        ethercat_update_inputs_from_sensors();

        // 3. 从EtherCAT输出数据更新执行器 (保持v1接口)
        ethercat_update_actuators_from_outputs();

        // 4. 更新状态信息
        ethercat_update_status_info();

        taskEXIT_CRITICAL();

        cycle_end_time = DWT->CYCCNT;
        cycle_duration = (cycle_end_time - cycle_start_time) * 1000000 / SystemCoreClock; // 转换为微秒

        // 更新性能统计
        if (cycle_duration > g_ethercat_status.max_cycle_time_us) {
            g_ethercat_status.max_cycle_time_us = cycle_duration;
        }

        g_ethercat_status.cycle_count++;

        // 性能监控 (每1000个周期报告一次)
        if (++performance_report_counter >= 1000) {
            ethercat_report_performance(cycle_overrun_count);
            performance_report_counter = 0;
            cycle_overrun_count = 0;
            g_ethercat_status.max_cycle_time_us = 0; // 重置统计
        }
    }
}

/**
 * @brief  更新传感器数据到EtherCAT输入 (保持v1接口)
 * @param  None
 * @retval None
 */
static void ethercat_update_inputs_from_sensors(void)
{
    sensor_data_t sensor_data;
    actuator_status_t actuator_status;

    // 获取传感器数据
    if (sensor_get_all_data(&sensor_data) == SENSOR_SUCCESS) {
        // 获取互斥锁保护数据访问
        if (xSemaphoreTake(g_ethercat_status.data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            // 液位传感器数据 (保持v1数据格式: mm * 10)
            g_ethercat_data.inputs.liquid_level_1 = (uint16_t)(sensor_data.liquid_level[0] * 10.0f);
            g_ethercat_data.inputs.liquid_level_2 = (uint16_t)(sensor_data.liquid_level[1] * 10.0f);

            // 压力传感器数据 (保持v1数据格式: (kPa + 100) * 100)
            g_ethercat_data.inputs.pressure_1 = (uint16_t)((sensor_data.pressure[0] + 100.0f) * 100.0f);
            g_ethercat_data.inputs.pressure_2 = (uint16_t)((sensor_data.pressure[1] + 100.0f) * 100.0f);

            // 温度传感器数据 (保持v1数据格式: °C * 10)
            g_ethercat_data.inputs.temperature_1 = (uint16_t)(sensor_data.temperature[0] * 10.0f);
            g_ethercat_data.inputs.temperature_2 = (uint16_t)(sensor_data.temperature[1] * 10.0f);

            // 数字输入状态 (保持v1数字IO定义)
            g_ethercat_data.inputs.digital_inputs = sensor_get_digital_input_status();

            // 系统状态和故障代码
            g_ethercat_data.inputs.system_status = system_get_status_word();
            g_ethercat_data.inputs.fault_code = system_get_current_fault_code();

            xSemaphoreGive(g_ethercat_status.data_mutex);
        }
    }
}

/**
 * @brief  从EtherCAT输出更新执行器 (保持v1接口)
 * @param  None
 * @retval None
 */
static void ethercat_update_actuators_from_outputs(void)
{
    // 获取互斥锁保护数据访问
    if (xSemaphoreTake(g_ethercat_status.data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        // 加热器功率控制 (保持v1接口: 0-1000 对应 0-100.0%)
        float heater_power_1 = (float)g_ethercat_data.outputs.heater_power_1 / 10.0f;
        float heater_power_2 = (float)g_ethercat_data.outputs.heater_power_2 / 10.0f;

        actuator_set_heater_power(0, heater_power_1);
        actuator_set_heater_power(1, heater_power_2);

        // 泵转速控制 (保持v1接口: 直接RPM值)
        uint16_t pump_speed_1 = g_ethercat_data.outputs.pump_speed_1;
        uint16_t pump_speed_2 = g_ethercat_data.outputs.pump_speed_2;

        actuator_set_pump_speed(0, pump_speed_1);
        actuator_set_pump_speed(1, pump_speed_2);

        // 数字输出控制 (保持v1数字IO定义)
        actuator_set_digital_output_status(g_ethercat_data.outputs.digital_outputs);

        // 控制命令处理
        ethercat_process_control_commands(g_ethercat_data.outputs.control_command);

        xSemaphoreGive(g_ethercat_status.data_mutex);
    }
}

/**
 * @brief  更新EtherCAT状态信息
 * @param  None
 * @retval None
 */
static void ethercat_update_status_info(void)
{
    // 读取ESC状态
    uint8_t al_status = ESC_ALStatusRead();
    g_ethercat_status.al_state = al_status & 0x0F;

    // 检查链路状态
    uint16_t link_status = ESC_Read(ESC_DL_STATUS_OFFSET);
    g_ethercat_status.link_up = (link_status & 0x0001) ? true : false;
}

/**
 * @brief  处理EtherCAT控制命令
 * @param  command 控制命令字
 * @retval None
 */
static void ethercat_process_control_commands(uint16_t command)
{
    switch (command) {
        case ETHERCAT_CMD_SYSTEM_RESET:
            // 系统复位命令
            system_request_reset();
            break;

        case ETHERCAT_CMD_EMERGENCY_STOP:
            // 紧急停机命令
            safety_trigger_emergency_stop();
            break;

        case ETHERCAT_CMD_CLEAR_FAULTS:
            // 清除故障命令
            system_clear_all_faults();
            break;

        case ETHERCAT_CMD_CALIBRATE_SENSORS:
            // 传感器校准命令
            sensor_start_calibration();
            break;

        case ETHERCAT_CMD_SAVE_CONFIG:
            // 保存配置命令
            config_save_to_flash();
            break;

        default:
            // 未知命令，记录错误
            g_ethercat_status.error_count++;
            break;
    }
}

/**
 * @brief  EtherCAT性能报告
 * @param  overrun_count 周期超限次数
 * @retval None
 */
static void ethercat_report_performance(uint32_t overrun_count)
{
    // 可以发送到日志系统或上位机
    if (overrun_count > 0) {
        printf("[EtherCAT] Performance: %u overruns, max cycle: %u us\n",
               overrun_count, g_ethercat_status.max_cycle_time_us);
    }
}

/**
 * @brief  获取EtherCAT状态
 * @param  status 状态结构指针
 * @retval ethercat_result_t 获取结果
 */
ethercat_result_t ethercat_get_status(ethercat_status_info_t *status)
{
    if (status == NULL) {
        return ETHERCAT_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_ethercat_status.data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        status->initialized = g_ethercat_status.initialized;
        status->link_up = g_ethercat_status.link_up;
        status->al_state = g_ethercat_status.al_state;
        status->cycle_count = g_ethercat_status.cycle_count;
        status->error_count = g_ethercat_status.error_count;
        status->max_cycle_time_us = g_ethercat_status.max_cycle_time_us;
        xSemaphoreGive(g_ethercat_status.data_mutex);
        return ETHERCAT_SUCCESS;
    }

    return ETHERCAT_ERROR_TIMEOUT;
}

/**
 * @brief  EtherCAT应用层回调函数 (SSC要求实现)
 * @param  None
 * @retval None
 */
void APPL_Application(void)
{
    // 应用层状态机处理
    // 这个函数由EtherCAT协议栈调用
}

/**
 * @brief  EtherCAT输入数据处理回调 (SSC要求实现)
 * @param  pData 数据指针
 * @param  Size 数据大小
 * @retval None
 */
void APPL_InputMapping(UINT16 *pData, UINT16 Size)
{
    if (Size >= sizeof(g_ethercat_data.inputs)) {
        memcpy(pData, &g_ethercat_data.inputs, sizeof(g_ethercat_data.inputs));
    }
}

/**
 * @brief  EtherCAT输出数据处理回调 (SSC要求实现)
 * @param  pData 数据指针
 * @param  Size 数据大小
 * @retval None
 */
void APPL_OutputMapping(UINT16 *pData, UINT16 Size)
{
    if (Size >= sizeof(g_ethercat_data.outputs)) {
        memcpy(&g_ethercat_data.outputs, pData, sizeof(g_ethercat_data.outputs));
    }
}