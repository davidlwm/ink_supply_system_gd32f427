/**
 * @file    hmi_task.c
 * @brief   人机界面任务实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "app/hmi_task.h"
#include "app/system_config.h"
#include "app/sensor_task.h"
#include "app/actuator_task.h"
#include "app/control_task.h"
#include "app/safety_task.h"
#include "app/config_task.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// 标准库
#include <string.h>
#include <stdio.h>

// HMI状态全局变量
static hmi_status_t g_hmi_status = {0};
static system_display_info_t g_display_info = {0};
static SemaphoreHandle_t g_hmi_mutex = NULL;
static QueueHandle_t g_key_queue = NULL;

// 消息显示相关
static char g_message_buffer[128] = {0};
static uint32_t g_message_start_time = 0;
static uint32_t g_message_duration = 0;

// 页面内容缓存
static char g_display_buffer[8][21] = {0}; // 8行x20字符LCD显示

/**
 * @brief  HMI管理器初始化
 * @param  None
 * @retval hmi_result_t 初始化结果
 */
hmi_result_t hmi_manager_init(void)
{
    // 创建互斥锁
    g_hmi_mutex = xSemaphoreCreateMutex();
    if (g_hmi_mutex == NULL) {
        return HMI_ERROR_MEMORY_ALLOCATION;
    }

    // 创建按键队列
    g_key_queue = xQueueCreate(10, sizeof(key_code_t));
    if (g_key_queue == NULL) {
        return HMI_ERROR_MEMORY_ALLOCATION;
    }

    // 初始化HMI状态
    g_hmi_status.current_page = HMI_PAGE_MAIN;
    g_hmi_status.display_enabled = true;
    g_hmi_status.keypad_enabled = true;
    g_hmi_status.brightness = 80; // 80%亮度
    g_hmi_status.last_activity_time = xTaskGetTickCount();
    g_hmi_status.screensaver_active = false;

    // 初始化硬件
    lcd_init();
    keypad_init();
    led_driver_init();
    buzzer_init();

    // 设置初始亮度
    lcd_set_brightness(g_hmi_status.brightness);

    // 显示启动画面
    lcd_clear();
    lcd_print(2, 0, "Ink Supply System");
    lcd_print(3, 0, "    Version 4.0    ");
    lcd_print(4, 0, "   Initializing... ");

    // 初始化LED指示灯
    hmi_set_led(LED_POWER, true);
    hmi_set_led(LED_RUN, false);
    hmi_set_led(LED_ERROR, false);
    hmi_set_led(LED_COMM, false);
    hmi_set_led(LED_ALARM, false);

    // 蜂鸣器提示初始化完成
    hmi_set_buzzer(BUZZER_SINGLE_BEEP);

    return HMI_SUCCESS;
}

/**
 * @brief  HMI任务主函数
 * @param  pvParameters 任务参数
 * @retval None
 */
void hmi_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    key_code_t key;
    uint32_t task_counter = 0;

    // 等待系统初始化完成
    vTaskDelay(pdMS_TO_TICKS(500));

    // 初始化HMI
    if (hmi_manager_init() != HMI_SUCCESS) {
        vTaskDelete(NULL);
        return;
    }

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 严格200ms周期执行
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HMI_TASK_PERIOD_MS));

        task_counter++;

        // 1. 处理按键输入
        if (keypad_get_key(&key)) {
            if (xQueueSend(g_key_queue, &key, 0) == pdTRUE) {
                g_hmi_status.last_activity_time = xTaskGetTickCount();
                if (g_hmi_status.screensaver_active) {
                    g_hmi_status.screensaver_active = false;
                    lcd_set_brightness(g_hmi_status.brightness);
                }
            }
        }

        // 2. 处理队列中的按键事件
        while (xQueueReceive(g_key_queue, &key, 0) == pdTRUE) {
            hmi_process_key(key);
        }

        // 3. 更新显示内容 (每周期)
        hmi_update_display();

        // 4. 更新LED指示灯 (每周期)
        hmi_update_indicators();

        // 5. 检查屏保 (每5秒)
        if (task_counter % (5000 / HMI_TASK_PERIOD_MS) == 0) {
            hmi_check_screensaver();
        }

        // 6. 更新系统信息 (每秒)
        if (task_counter % (1000 / HMI_TASK_PERIOD_MS) == 0) {
            hmi_update_system_data();
        }
    }
}

/**
 * @brief  处理按键输入
 * @param  key 按键代码
 * @retval hmi_result_t 处理结果
 */
hmi_result_t hmi_process_key(key_code_t key)
{
    if (xSemaphoreTake(g_hmi_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return HMI_ERROR_TIMEOUT;
    }

    switch (key) {
        case KEY_MENU:
            // 菜单键：返回主页面
            g_hmi_status.current_page = HMI_PAGE_MAIN;
            break;

        case KEY_UP:
        case KEY_DOWN:
        case KEY_LEFT:
        case KEY_RIGHT:
            // 导航键处理
            hmi_handle_navigation(key);
            break;

        case KEY_ENTER:
            // 确认键处理
            hmi_handle_enter();
            break;

        case KEY_ESC:
            // 取消键：返回上一页面
            if (g_hmi_status.current_page != HMI_PAGE_MAIN) {
                g_hmi_status.current_page = HMI_PAGE_MAIN;
            }
            break;

        case KEY_F1:
            // 功能键1：传感器状态页面
            g_hmi_status.current_page = HMI_PAGE_SENSOR_STATUS;
            break;

        case KEY_F2:
            // 功能键2：执行器状态页面
            g_hmi_status.current_page = HMI_PAGE_ACTUATOR_STATUS;
            break;

        case KEY_F3:
            // 功能键3：系统信息页面
            g_hmi_status.current_page = HMI_PAGE_SYSTEM_INFO;
            break;

        default:
            break;
    }

    // 按键音反馈
    hmi_set_buzzer(BUZZER_SINGLE_BEEP);

    xSemaphoreGive(g_hmi_mutex);
    return HMI_SUCCESS;
}

/**
 * @brief  更新显示内容
 * @param  None
 * @retval hmi_result_t 更新结果
 */
hmi_result_t hmi_update_display(void)
{
    if (g_hmi_status.screensaver_active || !g_hmi_status.display_enabled) {
        return HMI_SUCCESS;
    }

    // 检查是否有消息需要显示
    if (g_message_duration > 0) {
        uint32_t current_time = xTaskGetTickCount();
        if ((current_time - g_message_start_time) < pdMS_TO_TICKS(g_message_duration)) {
            // 继续显示消息
            return HMI_SUCCESS;
        } else {
            // 消息显示时间结束，清除消息
            g_message_duration = 0;
            memset(g_message_buffer, 0, sizeof(g_message_buffer));
        }
    }

    // 根据当前页面更新显示
    switch (g_hmi_status.current_page) {
        case HMI_PAGE_MAIN:
            hmi_display_main_page();
            break;

        case HMI_PAGE_SENSOR_STATUS:
            hmi_display_sensor_page();
            break;

        case HMI_PAGE_ACTUATOR_STATUS:
            hmi_display_actuator_page();
            break;

        case HMI_PAGE_SYSTEM_INFO:
            hmi_display_system_page();
            break;

        case HMI_PAGE_FAULT_LOG:
            hmi_display_fault_page();
            break;

        default:
            hmi_display_main_page();
            break;
    }

    return HMI_SUCCESS;
}

/**
 * @brief  显示主页面
 * @param  None
 * @retval None
 */
static void hmi_display_main_page(void)
{
    char temp_str[21];

    lcd_clear();

    // 标题行
    lcd_print(0, 0, "=== MAIN STATUS ===");

    // 系统状态行
    snprintf(temp_str, sizeof(temp_str), "SYS: %s  EFF:%.1f%%",
             g_display_info.system_state == 2 ? "RUN" : "IDLE",
             g_display_info.system_efficiency);
    lcd_print(1, 0, temp_str);

    // 温度状态行
    snprintf(temp_str, sizeof(temp_str), "T1:%.1f T2:%.1f T3:%.1f",
             g_display_info.temperatures[0],
             g_display_info.temperatures[1],
             g_display_info.temperatures[2]);
    lcd_print(2, 0, temp_str);

    // 压力状态行
    snprintf(temp_str, sizeof(temp_str), "P1:%.1fkPa P2:%.1fkPa",
             g_display_info.pressures[0],
             g_display_info.pressures[1]);
    lcd_print(3, 0, temp_str);

    // 液位状态行
    snprintf(temp_str, sizeof(temp_str), "L1:%.1f%% L2:%.1f%%",
             g_display_info.liquid_levels[0],
             g_display_info.liquid_levels[1]);
    lcd_print(4, 0, temp_str);

    // 运行时间行
    uint32_t hours = g_display_info.uptime / 3600;
    uint32_t minutes = (g_display_info.uptime % 3600) / 60;
    snprintf(temp_str, sizeof(temp_str), "Uptime: %02luh:%02lum", hours, minutes);
    lcd_print(5, 0, temp_str);

    // 功能按键提示
    lcd_print(7, 0, "F1:SEN F2:ACT F3:SYS");
}

/**
 * @brief  显示传感器页面
 * @param  None
 * @retval None
 */
static void hmi_display_sensor_page(void)
{
    char temp_str[21];

    lcd_clear();

    // 标题行
    lcd_print(0, 0, "=== SENSOR DATA ===");

    // 温度传感器
    snprintf(temp_str, sizeof(temp_str), "Temp1: %.2f C %s",
             g_display_info.temperatures[0],
             g_display_info.fault_status[4] ? "ERR" : "OK");
    lcd_print(1, 0, temp_str);

    snprintf(temp_str, sizeof(temp_str), "Temp2: %.2f C %s",
             g_display_info.temperatures[1],
             g_display_info.fault_status[5] ? "ERR" : "OK");
    lcd_print(2, 0, temp_str);

    snprintf(temp_str, sizeof(temp_str), "Temp3: %.2f C %s",
             g_display_info.temperatures[2],
             g_display_info.fault_status[6] ? "ERR" : "OK");
    lcd_print(3, 0, temp_str);

    // 压力传感器
    snprintf(temp_str, sizeof(temp_str), "Pres1: %.1f kPa %s",
             g_display_info.pressures[0],
             g_display_info.fault_status[2] ? "ERR" : "OK");
    lcd_print(4, 0, temp_str);

    snprintf(temp_str, sizeof(temp_str), "Pres2: %.1f kPa %s",
             g_display_info.pressures[1],
             g_display_info.fault_status[3] ? "ERR" : "OK");
    lcd_print(5, 0, temp_str);

    // 液位传感器
    snprintf(temp_str, sizeof(temp_str), "Level1: %.1f%% %s",
             g_display_info.liquid_levels[0],
             g_display_info.fault_status[0] ? "ERR" : "OK");
    lcd_print(6, 0, temp_str);

    snprintf(temp_str, sizeof(temp_str), "Level2: %.1f%% %s",
             g_display_info.liquid_levels[1],
             g_display_info.fault_status[1] ? "ERR" : "OK");
    lcd_print(7, 0, temp_str);
}

/**
 * @brief  显示执行器页面
 * @param  None
 * @retval None
 */
static void hmi_display_actuator_page(void)
{
    char temp_str[21];

    lcd_clear();

    // 标题行
    lcd_print(0, 0, "== ACTUATOR STATUS ==");

    // 加热器状态
    float heater1_power = get_heater_power(0);
    float heater2_power = get_heater_power(1);
    float heater3_power = get_heater_power(2);

    snprintf(temp_str, sizeof(temp_str), "Heat1: %.1f%% %s",
             heater1_power,
             get_actuator_fault_status(ACTUATOR_TYPE_HEATER, 0) ? "ERR" : "OK");
    lcd_print(1, 0, temp_str);

    snprintf(temp_str, sizeof(temp_str), "Heat2: %.1f%% %s",
             heater2_power,
             get_actuator_fault_status(ACTUATOR_TYPE_HEATER, 1) ? "ERR" : "OK");
    lcd_print(2, 0, temp_str);

    snprintf(temp_str, sizeof(temp_str), "Heat3: %.1f%% %s",
             heater3_power,
             get_actuator_fault_status(ACTUATOR_TYPE_HEATER, 2) ? "ERR" : "OK");
    lcd_print(3, 0, temp_str);

    // 泵状态
    uint16_t pump1_speed = get_pump_speed(0);
    uint16_t pump2_speed = get_pump_speed(1);

    snprintf(temp_str, sizeof(temp_str), "Pump1: %d RPM %s",
             pump1_speed,
             get_actuator_fault_status(ACTUATOR_TYPE_PUMP, 0) ? "ERR" : "OK");
    lcd_print(4, 0, temp_str);

    snprintf(temp_str, sizeof(temp_str), "Pump2: %d RPM %s",
             pump2_speed,
             get_actuator_fault_status(ACTUATOR_TYPE_PUMP, 1) ? "ERR" : "OK");
    lcd_print(5, 0, temp_str);

    // 阀门状态
    bool valve1_state = get_valve_state(0);
    bool valve2_state = get_valve_state(1);

    snprintf(temp_str, sizeof(temp_str), "Valve1: %s  Valve2: %s",
             valve1_state ? "OPEN" : "CLOSE",
             valve2_state ? "OPEN" : "CLOSE");
    lcd_print(6, 0, temp_str);

    lcd_print(7, 0, "MENU:Back ESC:Exit");
}

/**
 * @brief  显示系统信息页面
 * @param  None
 * @retval None
 */
static void hmi_display_system_page(void)
{
    char temp_str[21];

    lcd_clear();

    // 标题行
    lcd_print(0, 0, "=== SYSTEM INFO ===");

    // CPU使用率和内存信息
    snprintf(temp_str, sizeof(temp_str), "CPU: %.1f%%", system_get_cpu_usage());
    lcd_print(1, 0, temp_str);

    // 系统效率
    snprintf(temp_str, sizeof(temp_str), "Efficiency: %.1f%%", g_display_info.system_efficiency);
    lcd_print(2, 0, temp_str);

    // 运行时间
    uint32_t days = g_display_info.uptime / 86400;
    uint32_t hours = (g_display_info.uptime % 86400) / 3600;
    uint32_t minutes = (g_display_info.uptime % 3600) / 60;

    snprintf(temp_str, sizeof(temp_str), "Uptime: %lud %02luh:%02lum", days, hours, minutes);
    lcd_print(3, 0, temp_str);

    // 版本信息
    lcd_print(4, 0, "HW Ver: V1.0");
    lcd_print(5, 0, "SW Ver: V4.0");
    lcd_print(6, 0, "Build: 2025-09-27");

    lcd_print(7, 0, "MENU:Back ESC:Exit");
}

/**
 * @brief  更新系统数据
 * @param  None
 * @retval None
 */
static void hmi_update_system_data(void)
{
    // 获取传感器数据
    for (int i = 0; i < 3; i++) {
        g_display_info.temperatures[i] = get_pt100_temperature(i + 4);
    }

    for (int i = 0; i < 2; i++) {
        g_display_info.pressures[i] = get_pressure_sensor(i + 2);
        g_display_info.liquid_levels[i] = get_liquid_level_sensor(i);
    }

    // 获取故障状态
    for (int i = 0; i < 7; i++) {
        if (i < 2) {
            g_display_info.fault_status[i] = get_sensor_fault_status(SENSOR_TYPE_LIQUID_LEVEL, i);
        } else if (i < 4) {
            g_display_info.fault_status[i] = get_sensor_fault_status(SENSOR_TYPE_PRESSURE, i - 2);
        } else {
            g_display_info.fault_status[i] = get_sensor_fault_status(SENSOR_TYPE_TEMPERATURE, i - 4);
        }
    }

    // 获取系统状态
    g_display_info.system_state = (uint8_t)system_get_state();
    g_display_info.uptime = system_get_uptime();

    // 获取控制系统效率
    control_system_status_info_t control_status;
    if (control_get_system_status(&control_status) == CONTROL_SUCCESS) {
        g_display_info.system_efficiency = control_status.system_efficiency;
    }
}

/**
 * @brief  更新LED指示灯
 * @param  None
 * @retval None
 */
static void hmi_update_indicators(void)
{
    static uint32_t led_counter = 0;
    led_counter++;

    // 电源指示灯常亮
    hmi_set_led(LED_POWER, true);

    // 运行指示灯
    if (g_display_info.system_state == 2) { // RUNNING
        hmi_set_led(LED_RUN, true);
    } else {
        // 闪烁表示待机
        hmi_set_led(LED_RUN, (led_counter % 10) < 5);
    }

    // 错误指示灯
    bool has_fault = false;
    for (int i = 0; i < 16; i++) {
        if (g_display_info.fault_status[i]) {
            has_fault = true;
            break;
        }
    }

    if (has_fault) {
        // 快速闪烁表示故障
        hmi_set_led(LED_ERROR, (led_counter % 4) < 2);
    } else {
        hmi_set_led(LED_ERROR, false);
    }

    // 通信指示灯 (根据通信模块状态设置)
    // 这里简化处理，实际应该从通信模块获取状态
    hmi_set_led(LED_COMM, (led_counter % 20) < 1); // 慢闪表示通信正常

    // 报警指示灯
    if (safety_is_emergency_stop_active()) {
        hmi_set_led(LED_ALARM, true);
    } else {
        hmi_set_led(LED_ALARM, false);
    }
}

/**
 * @brief  设置LED状态
 * @param  led LED编号
 * @param  on 开关状态
 * @retval hmi_result_t 设置结果
 */
hmi_result_t hmi_set_led(led_indicator_t led, bool on)
{
    led_driver_set((uint8_t)led, on);
    return HMI_SUCCESS;
}

/**
 * @brief  设置蜂鸣器模式
 * @param  mode 蜂鸣器模式
 * @retval hmi_result_t 设置结果
 */
hmi_result_t hmi_set_buzzer(buzzer_mode_t mode)
{
    buzzer_beep(mode);
    return HMI_SUCCESS;
}

/**
 * @brief  显示消息
 * @param  message 消息内容
 * @param  duration_ms 显示时长
 * @retval hmi_result_t 显示结果
 */
hmi_result_t hmi_show_message(const char *message, uint32_t duration_ms)
{
    if (message == NULL) {
        return HMI_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(g_hmi_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return HMI_ERROR_TIMEOUT;
    }

    // 保存消息
    strncpy(g_message_buffer, message, sizeof(g_message_buffer) - 1);
    g_message_buffer[sizeof(g_message_buffer) - 1] = '\0';

    g_message_start_time = xTaskGetTickCount();
    g_message_duration = duration_ms;

    // 立即显示消息
    lcd_clear();
    lcd_print(2, 0, "=== MESSAGE ===");
    lcd_print(4, 0, g_message_buffer);

    xSemaphoreGive(g_hmi_mutex);
    return HMI_SUCCESS;
}

/**
 * @brief  检查屏保
 * @param  None
 * @retval None
 */
static void hmi_check_screensaver(void)
{
    uint32_t current_time = xTaskGetTickCount();
    uint32_t idle_time = current_time - g_hmi_status.last_activity_time;

    // 5分钟无操作启动屏保
    if (idle_time > pdMS_TO_TICKS(300000) && !g_hmi_status.screensaver_active) {
        g_hmi_status.screensaver_active = true;
        lcd_set_brightness(10); // 降低亮度
        lcd_clear();
        lcd_print(3, 0, "  Screen Saver  ");
        lcd_print(4, 0, " Press any key  ");
    }
}

// 其他内部辅助函数的简化实现
static void hmi_handle_navigation(key_code_t key)
{
    // 导航逻辑的简化实现
    switch (key) {
        case KEY_UP:
            if (g_hmi_status.current_page > 0) {
                g_hmi_status.current_page--;
            }
            break;
        case KEY_DOWN:
            if (g_hmi_status.current_page < HMI_PAGE_CALIBRATION) {
                g_hmi_status.current_page++;
            }
            break;
        default:
            break;
    }
}

static void hmi_handle_enter(void)
{
    // 确认键处理的简化实现
    // 根据当前页面执行相应操作
}

static void hmi_display_fault_page(void)
{
    // 故障页面显示的简化实现
    lcd_clear();
    lcd_print(0, 0, "=== FAULT LOG ===");
    lcd_print(7, 0, "MENU:Back ESC:Exit");
}