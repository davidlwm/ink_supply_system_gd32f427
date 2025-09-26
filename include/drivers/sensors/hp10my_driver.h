/**
 * @file    hp10my_driver.h
 * @brief   HP10MY压力传感器驱动头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __HP10MY_DRIVER_H
#define __HP10MY_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

// 传感器状态结构
typedef struct {
    float pressure_kpa;     // 压力值 (kPa)
    float voltage;          // 电压值 (V)
    bool fault;            // 故障状态
    bool initialized;      // 初始化状态
} hp10my_status_t;

// 函数声明
void hp10my_init(void);
float read_pressure_hp10my(uint8_t sensor_id);
hp10my_status_t hp10my_get_status(uint8_t sensor_id);
bool hp10my_calibrate(uint8_t sensor_id, float actual_pressure);
bool hp10my_set_calibration(uint8_t sensor_id, float offset, float scale);
bool hp10my_check_connection(uint8_t sensor_id);
float hp10my_get_voltage(uint8_t sensor_id);
void hp10my_reset_fault(uint8_t sensor_id);
float hp10my_convert_pressure_unit(float pressure_kpa, uint8_t target_unit);

#endif /* __HP10MY_DRIVER_H */