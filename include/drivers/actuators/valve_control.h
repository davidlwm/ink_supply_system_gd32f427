/**
 * @file    valve_control.h
 * @brief   电磁阀控制驱动头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __VALVE_CONTROL_H
#define __VALVE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// 阀门类型定义
typedef enum {
    VALVE_TYPE_SOLENOID_2WAY = 0,   // 二通电磁阀
    VALVE_TYPE_SOLENOID_3WAY = 1,   // 三通电磁阀
    VALVE_TYPE_PROPORTIONAL = 2     // 比例阀
} valve_type_t;

// 阀门状态结构
typedef struct {
    bool is_open;           // 开关状态
    bool enabled;          // 使能状态
    bool fault;           // 故障状态
    bool overcurrent;     // 过流故障
    bool stuck;           // 卡死故障
    uint32_t action_count; // 动作次数
    uint32_t total_open_time; // 总开启时间 (秒)
    bool initialized;     // 初始化状态
} valve_status_t;

// 函数声明
void valve_control_init(void);
void control_valve(uint8_t valve_id, bool open);
valve_status_t valve_get_status(uint8_t valve_id);
bool valve_set_state(uint8_t valve_id, bool open);
bool valve_get_state(uint8_t valve_id);
bool valve_enable(uint8_t valve_id, bool enable);
bool valve_check_fault(uint8_t valve_id);
void valve_reset_fault(uint8_t valve_id);
void valve_emergency_close_all(void);
void valve_set_batch_state(uint8_t valve_mask, uint8_t open_mask);
uint8_t valve_get_batch_state(void);
bool valve_self_test(uint8_t valve_id);
uint32_t valve_get_action_count(uint8_t valve_id);
uint32_t valve_get_total_open_time(uint8_t valve_id);

#endif /* __VALVE_CONTROL_H */