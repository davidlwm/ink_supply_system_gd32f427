/**
 * @file    valve_control.c
 * @brief   电磁阀控制驱动实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "valve_control.h"
#include "gpio_hal.h"
#include "system_config.h"

// 阀门配置参数
typedef struct {
    gpio_output_id_t control_pin;      // 控制引脚
    valve_type_t type;                 // 阀门类型
    uint32_t min_action_time;          // 最小动作时间 (ms)
    uint32_t max_continuous_time;      // 最大连续工作时间 (ms)
    bool normally_open;                // 常开类型
} valve_config_t;

// 阀门实例配置
static const valve_config_t valve_configs[8] = {
    // 阀门1-4 (主供墨阀门) - 常闭
    {GPIO_VALVE_1, VALVE_TYPE_SOLENOID_2WAY, 100, 300000, false},
    {GPIO_VALVE_2, VALVE_TYPE_SOLENOID_2WAY, 100, 300000, false},
    {GPIO_VALVE_3, VALVE_TYPE_SOLENOID_2WAY, 100, 300000, false},
    {GPIO_VALVE_4, VALVE_TYPE_SOLENOID_2WAY, 100, 300000, false},

    // 阀门5-6 (排气阀门) - 常开
    {GPIO_VALVE_5, VALVE_TYPE_SOLENOID_2WAY, 50, 60000, true},
    {GPIO_VALVE_6, VALVE_TYPE_SOLENOID_2WAY, 50, 60000, true},

    // 阀门7-8 (循环阀门) - 常闭
    {GPIO_VALVE_7, VALVE_TYPE_SOLENOID_3WAY, 100, 600000, false},
    {GPIO_VALVE_8, VALVE_TYPE_SOLENOID_3WAY, 100, 600000, false}
};

// 阀门状态
static valve_status_t valve_status[8];

// 动作时间记录
static uint32_t valve_action_times[8] = {0};
static uint32_t valve_open_times[8] = {0};

/**
 * @brief  阀门控制初始化
 * @param  None
 * @retval None
 */
void valve_control_init(void)
{
    // 初始化GPIO和阀门状态
    for (int i = 0; i < 8; i++) {
        // 配置控制引脚
        gpio_hal_config_output(valve_configs[i].control_pin);

        // 初始化状态
        valve_status[i].is_open = valve_configs[i].normally_open;
        valve_status[i].enabled = true;
        valve_status[i].fault = false;
        valve_status[i].overcurrent = false;
        valve_status[i].stuck = false;
        valve_status[i].action_count = 0;
        valve_status[i].total_open_time = 0;
        valve_status[i].initialized = true;

        // 设置初始状态
        control_valve(i, valve_status[i].is_open);

        // 重置计时器
        valve_action_times[i] = 0;
        valve_open_times[i] = 0;
    }
}

/**
 * @brief  控制阀门开关
 * @param  valve_id 阀门ID (0-7)
 * @param  open 开关状态 (true=开, false=关)
 * @retval None
 */
void control_valve(uint8_t valve_id, bool open)
{
    if (valve_id >= 8) {
        return;
    }

    const valve_config_t *config = &valve_configs[valve_id];

    // 检查使能状态和故障状态
    if (!valve_status[valve_id].enabled || valve_status[valve_id].fault) {
        return;
    }

    // 检查最小动作时间间隔
    uint32_t current_time = xTaskGetTickCount();
    if (valve_action_times[valve_id] != 0) {
        uint32_t time_since_action = current_time - valve_action_times[valve_id];
        if (time_since_action < pdMS_TO_TICKS(config->min_action_time)) {
            return; // 动作过快，忽略
        }
    }

    // 检查连续工作时间限制
    if (open && valve_status[valve_id].is_open) {
        if (valve_open_times[valve_id] != 0) {
            uint32_t open_duration = current_time - valve_open_times[valve_id];
            if (open_duration > pdMS_TO_TICKS(config->max_continuous_time)) {
                // 超过最大连续工作时间，强制关闭
                open = false;
                valve_status[valve_id].fault = true;
            }
        }
    }

    // 更新状态
    bool state_changed = (valve_status[valve_id].is_open != open);
    valve_status[valve_id].is_open = open;

    // 控制GPIO (考虑常开/常闭类型)
    bool gpio_state = config->normally_open ? !open : open;
    gpio_hal_write_pin(config->control_pin, gpio_state);

    // 更新计时器和统计
    if (state_changed) {
        valve_action_times[valve_id] = current_time;
        valve_status[valve_id].action_count++;

        if (open) {
            valve_open_times[valve_id] = current_time;
        } else {
            // 关闭时累计开启时间
            if (valve_open_times[valve_id] != 0) {
                valve_status[valve_id].total_open_time +=
                    (current_time - valve_open_times[valve_id]) / 1000; // 转换为秒
                valve_open_times[valve_id] = 0;
            }
        }
    }
}

/**
 * @brief  获取阀门状态
 * @param  valve_id 阀门ID (0-7)
 * @retval valve_status_t 阀门状态
 */
valve_status_t valve_get_status(uint8_t valve_id)
{
    valve_status_t status = {0};

    if (valve_id < 8) {
        status = valve_status[valve_id];
    }

    return status;
}

/**
 * @brief  设置阀门状态
 * @param  valve_id 阀门ID (0-7)
 * @param  open 开关状态
 * @retval bool 设置成功返回true
 */
bool valve_set_state(uint8_t valve_id, bool open)
{
    if (valve_id >= 8) {
        return false;
    }

    control_valve(valve_id, open);
    return true;
}

/**
 * @brief  获取阀门开关状态
 * @param  valve_id 阀门ID (0-7)
 * @retval bool 阀门状态 (true=开, false=关)
 */
bool valve_get_state(uint8_t valve_id)
{
    if (valve_id >= 8) {
        return false;
    }

    return valve_status[valve_id].is_open;
}

/**
 * @brief  使能/禁用阀门
 * @param  valve_id 阀门ID (0-7)
 * @param  enable 使能状态
 * @retval bool 操作成功返回true
 */
bool valve_enable(uint8_t valve_id, bool enable)
{
    if (valve_id >= 8) {
        return false;
    }

    valve_status[valve_id].enabled = enable;

    if (!enable) {
        // 禁用时关闭阀门
        control_valve(valve_id, false);
    }

    return true;
}

/**
 * @brief  检查阀门故障
 * @param  valve_id 阀门ID (0-7)
 * @retval bool 有故障返回true
 */
bool valve_check_fault(uint8_t valve_id)
{
    if (valve_id >= 8) {
        return true;
    }

    // 检查是否卡死 (长时间无动作但有控制信号)
    uint32_t current_time = xTaskGetTickCount();
    if (valve_action_times[valve_id] != 0) {
        uint32_t time_since_action = current_time - valve_action_times[valve_id];
        if (time_since_action > pdMS_TO_TICKS(10000)) { // 10秒无动作
            // 这里可以添加更复杂的卡死检测逻辑
        }
    }

    return valve_status[valve_id].fault ||
           valve_status[valve_id].overcurrent ||
           valve_status[valve_id].stuck;
}

/**
 * @brief  重置阀门故障
 * @param  valve_id 阀门ID (0-7)
 * @retval None
 */
void valve_reset_fault(uint8_t valve_id)
{
    if (valve_id < 8) {
        valve_status[valve_id].fault = false;
        valve_status[valve_id].overcurrent = false;
        valve_status[valve_id].stuck = false;
        valve_action_times[valve_id] = 0;
        valve_open_times[valve_id] = 0;
    }
}

/**
 * @brief  紧急关闭所有阀门
 * @param  None
 * @retval None
 */
void valve_emergency_close_all(void)
{
    for (int i = 0; i < 8; i++) {
        // 立即关闭所有阀门
        bool gpio_state = valve_configs[i].normally_open ? true : false;
        gpio_hal_write_pin(valve_configs[i].control_pin, gpio_state);

        // 更新状态
        valve_status[i].is_open = false;
        valve_status[i].fault = true;
    }
}

/**
 * @brief  批量设置阀门状态
 * @param  valve_mask 阀门掩码 (bit0-7对应阀门0-7)
 * @param  open_mask 开关掩码 (对应位为1表示开启)
 * @retval None
 */
void valve_set_batch_state(uint8_t valve_mask, uint8_t open_mask)
{
    for (int i = 0; i < 8; i++) {
        if (valve_mask & (1 << i)) {
            bool open_state = (open_mask & (1 << i)) != 0;
            control_valve(i, open_state);
        }
    }
}

/**
 * @brief  获取批量阀门状态
 * @param  None
 * @retval uint8_t 阀门状态掩码 (bit0-7对应阀门0-7状态)
 */
uint8_t valve_get_batch_state(void)
{
    uint8_t state_mask = 0;

    for (int i = 0; i < 8; i++) {
        if (valve_status[i].is_open) {
            state_mask |= (1 << i);
        }
    }

    return state_mask;
}

/**
 * @brief  执行阀门自检
 * @param  valve_id 阀门ID (0-7)
 * @retval bool 自检通过返回true
 */
bool valve_self_test(uint8_t valve_id)
{
    if (valve_id >= 8) {
        return false;
    }

    bool original_state = valve_status[valve_id].is_open;

    // 执行开关动作测试
    control_valve(valve_id, true);
    vTaskDelay(pdMS_TO_TICKS(200));

    control_valve(valve_id, false);
    vTaskDelay(pdMS_TO_TICKS(200));

    // 恢复原状态
    control_valve(valve_id, original_state);

    // 检查是否有故障产生
    return !valve_status[valve_id].fault;
}

/**
 * @brief  获取阀门动作次数
 * @param  valve_id 阀门ID (0-7)
 * @retval uint32_t 动作次数
 */
uint32_t valve_get_action_count(uint8_t valve_id)
{
    if (valve_id >= 8) {
        return 0;
    }

    return valve_status[valve_id].action_count;
}

/**
 * @brief  获取阀门总开启时间
 * @param  valve_id 阀门ID (0-7)
 * @retval uint32_t 总开启时间 (秒)
 */
uint32_t valve_get_total_open_time(uint8_t valve_id)
{
    if (valve_id >= 8) {
        return 0;
    }

    // 如果当前是开启状态，需要加上当前开启时间
    uint32_t total_time = valve_status[valve_id].total_open_time;

    if (valve_status[valve_id].is_open && valve_open_times[valve_id] != 0) {
        uint32_t current_time = xTaskGetTickCount();
        total_time += (current_time - valve_open_times[valve_id]) / 1000;
    }

    return total_time;
}