/**
 * @file    frd8061_driver.h
 * @brief   FRD-8061液位传感器驱动头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __FRD8061_DRIVER_H
#define __FRD8061_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

// 传感器状态结构
typedef struct {
    float level_percent;    // 液位百分比 (0-100%)
    float voltage;          // 电压值 (V)
    bool fault;            // 故障状态
    bool initialized;      // 初始化状态
} frd8061_status_t;

// 函数声明
void frd8061_init(void);
float read_liquid_level_frd8061(uint8_t sensor_id);
frd8061_status_t frd8061_get_status(uint8_t sensor_id);
bool frd8061_calibrate(uint8_t sensor_id, float actual_level);
bool frd8061_set_calibration(uint8_t sensor_id, float offset, float scale);
bool frd8061_check_connection(uint8_t sensor_id);
float frd8061_get_voltage(uint8_t sensor_id);
void frd8061_reset_fault(uint8_t sensor_id);

#endif /* __FRD8061_DRIVER_H */