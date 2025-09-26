/**
 * @file    mra23d3_driver.h
 * @brief   MRA-23D3固态继电器加热器驱动头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __MRA23D3_DRIVER_H
#define __MRA23D3_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

// 加热器状态结构
typedef struct {
    float power_percent;    // 功率百分比 (0-100%)
    bool enabled;          // 使能状态
    bool fault;           // 故障状态
    bool overtemp;        // 过温故障
    bool overcurrent;     // 过流故障
    float actual_current; // 实际电流 (A)
    float actual_voltage; // 实际电压 (V)
    bool initialized;     // 初始化状态
} mra23d3_status_t;

// 函数声明
void mra23d3_heater_init(void);
void control_heater_mra23d3(uint8_t heater_id, float power_percent);
mra23d3_status_t mra23d3_get_status(uint8_t heater_id);
bool mra23d3_set_power(uint8_t heater_id, float power_percent);
float mra23d3_get_power(uint8_t heater_id);
bool mra23d3_enable(uint8_t heater_id, bool enable);
bool mra23d3_check_fault(uint8_t heater_id);
void mra23d3_reset_fault(uint8_t heater_id);
void mra23d3_set_overtemp_protection(uint8_t heater_id, bool overtemp);
void mra23d3_emergency_shutdown(void);
float mra23d3_get_max_power(uint8_t heater_id);
bool mra23d3_set_max_power(uint8_t heater_id, float max_power);
bool mra23d3_self_test(uint8_t heater_id);

#endif /* __MRA23D3_DRIVER_H */