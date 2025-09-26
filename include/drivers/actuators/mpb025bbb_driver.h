/**
 * @file    mpb025bbb_driver.h
 * @brief   MPB025BBB微型齿轮泵驱动头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __MPB025BBB_DRIVER_H
#define __MPB025BBB_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

// 泵方向定义
typedef enum {
    PUMP_DIRECTION_FORWARD = 0,     // 正向
    PUMP_DIRECTION_REVERSE = 1      // 反向
} pump_direction_t;

// 泵状态结构
typedef struct {
    uint16_t speed_rpm;         // 当前转速 (RPM)
    uint16_t target_speed;      // 目标转速 (RPM)
    bool enabled;              // 使能状态
    pump_direction_t direction; // 旋转方向
    bool fault;               // 故障状态
    bool stalled;             // 堵转故障
    bool overcurrent;         // 过流故障
    float actual_current;     // 实际电流 (A)
    float flow_rate_ml_min;   // 流量 (ml/min)
    bool initialized;         // 初始化状态
} mpb025bbb_status_t;

// 函数声明
void mpb025bbb_pump_init(void);
void control_pump_speed_mpb025bbb(uint8_t pump_id, uint16_t target_rpm);
mpb025bbb_status_t mpb025bbb_get_status(uint8_t pump_id);
bool mpb025bbb_set_speed(uint8_t pump_id, uint16_t speed_rpm);
uint16_t mpb025bbb_get_speed(uint8_t pump_id);
bool mpb025bbb_set_direction(uint8_t pump_id, pump_direction_t direction);
bool mpb025bbb_enable(uint8_t pump_id, bool enable);
bool mpb025bbb_check_fault(uint8_t pump_id);
void mpb025bbb_reset_fault(uint8_t pump_id);
void mpb025bbb_emergency_stop(void);
float mpb025bbb_get_flow_rate(uint8_t pump_id);
bool mpb025bbb_self_test(uint8_t pump_id);
uint16_t mpb025bbb_get_max_speed(uint8_t pump_id);
uint16_t mpb025bbb_get_min_speed(uint8_t pump_id);

#endif /* __MPB025BBB_DRIVER_H */