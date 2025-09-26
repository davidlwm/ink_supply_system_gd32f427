/**
 * @file    ftt518_driver.h
 * @brief   FTT518 PT100温度传感器驱动头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __FTT518_DRIVER_H
#define __FTT518_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

// 传感器状态结构
typedef struct {
    float temperature;      // 温度值 (°C)
    float resistance;       // 电阻值 (Ω)
    bool fault;            // 故障状态
    bool open_circuit;     // 开路故障
    bool short_circuit;    // 短路故障
    bool initialized;      // 初始化状态
} ftt518_status_t;

// 函数声明
void ftt518_init(void);
float read_temperature_ftt518_pt100(uint8_t sensor_id);
float get_pt100_resistance_value(uint8_t sensor_id);
ftt518_status_t ftt518_get_status(uint8_t sensor_id);
bool ftt518_calibrate(uint8_t sensor_id, float actual_temperature);
bool ftt518_set_calibration(uint8_t sensor_id, float offset, float scale);
bool ftt518_check_connection(uint8_t sensor_id);
float ftt518_get_resistance(uint8_t sensor_id);
void ftt518_reset_fault(uint8_t sensor_id);
float ftt518_convert_temperature_unit(float temperature_c, uint8_t target_unit);
bool ftt518_self_test(uint8_t sensor_id);

#endif /* __FTT518_DRIVER_H */