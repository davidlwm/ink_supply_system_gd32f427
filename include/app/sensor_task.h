/**
 * @file    sensor_task.h
 * @brief   传感器任务头文件 - 8周v4标准
 * @version V4.0
 * @date    2024-12-27
 */

#ifndef __SENSOR_TASK_H
#define __SENSOR_TASK_H

#include "app_types.h"
#include "middleware/filter.h"
#include <stdint.h>
#include <stdbool.h>

/* 传感器任务配置 */
#define SENSOR_TASK_PRIORITY        APP_PRIORITY_NORMAL
#define SENSOR_TASK_STACK_SIZE      APP_STACK_SIZE_MEDIUM
#define SENSOR_TASK_PERIOD_MS       50      /* 传感器采集周期50ms */

/* 传感器数量定义 */
#define LIQUID_LEVEL_SENSOR_COUNT   2       /* FRD-8061液位传感器数量 */
#define PRESSURE_SENSOR_COUNT       2       /* HP10MY压力传感器数量 */
#define TEMPERATURE_SENSOR_COUNT    3       /* FTT518温度传感器数量 */
#define TOTAL_SENSOR_COUNT          (LIQUID_LEVEL_SENSOR_COUNT + PRESSURE_SENSOR_COUNT + TEMPERATURE_SENSOR_COUNT)

/* 传感器状态定义 */
typedef enum {
    SENSOR_STATE_OFFLINE = 0,
    SENSOR_STATE_ONLINE = 1,
    SENSOR_STATE_FAULT = 2,
    SENSOR_STATE_CALIBRATING = 3
} sensor_state_t;

/* 液位传感器数据结构 */
typedef struct {
    app_sensor_data_t base;         /* 基础传感器数据 */
    moving_average_filter_t filter; /* 滤波器 */
    sensor_state_t state;           /* 传感器状态 */
    float level_mm;                 /* 液位值(mm) */
    float level_percent;            /* 液位百分比 */
    float tank_height_mm;           /* 水箱高度 */
    float alarm_high_level;         /* 高位报警 */
    float alarm_low_level;          /* 低位报警 */
} liquid_level_sensor_t;

/* 压力传感器数据结构 */
typedef struct {
    app_sensor_data_t base;         /* 基础传感器数据 */
    moving_average_filter_t filter; /* 滤波器 */
    sensor_state_t state;           /* 传感器状态 */
    float pressure_kpa;             /* 压力值(kPa) */
    float pressure_bar;             /* 压力值(bar) */
    float alarm_high_pressure;      /* 高压报警 */
    float alarm_low_pressure;       /* 低压报警 */
} pressure_sensor_t;

/* 温度传感器数据结构 */
typedef struct {
    app_sensor_data_t base;         /* 基础传感器数据 */
    moving_average_filter_t filter; /* 滤波器 */
    sensor_state_t state;           /* 传感器状态 */
    float temperature_c;            /* 温度值(°C) */
    float resistance_ohm;           /* PT100电阻值(Ω) */
    float alarm_high_temp;          /* 高温报警 */
    float alarm_low_temp;           /* 低温报警 */
} temperature_sensor_t;

/* 传感器系统结构 */
typedef struct {
    liquid_level_sensor_t level_sensors[LIQUID_LEVEL_SENSOR_COUNT];
    pressure_sensor_t pressure_sensors[PRESSURE_SENSOR_COUNT];
    temperature_sensor_t temp_sensors[TEMPERATURE_SENSOR_COUNT];
    uint32_t read_count;            /* 读取次数统计 */
    uint32_t error_count;           /* 错误次数统计 */
    bool system_fault;              /* 系统故障状态 */
} sensor_system_t;

/* 控制任务兼容的传感器数据结构 */
typedef struct {
    float liquid_level[LIQUID_LEVEL_SENSOR_COUNT];     /* 液位传感器数据 */
    float pressure[PRESSURE_SENSOR_COUNT];             /* 压力传感器数据 */
    float temperature[TEMPERATURE_SENSOR_COUNT];       /* 温度传感器数据 */
    bool fault_status[TOTAL_SENSOR_COUNT];             /* 故障状态数组 */
    uint32_t timestamp;                                /* 时间戳 */
} sensor_data_t;

/* 任务函数 */
void sensor_task(void *pvParameters);
app_result_t sensor_manager_init(void);
app_result_t sensor_manager_deinit(void);

/* 传感器操作接口 */
app_result_t sensor_read_all_data(void);
app_result_t sensor_get_level_data(uint8_t sensor_id, liquid_level_sensor_t* data);
app_result_t sensor_get_pressure_data(uint8_t sensor_id, pressure_sensor_t* data);
app_result_t sensor_get_temperature_data(uint8_t sensor_id, temperature_sensor_t* data);

/* 控制任务接口 */
app_result_t sensor_get_all_data(sensor_data_t* data);

/* 传感器校准接口 */
app_result_t sensor_calibrate_level(uint8_t sensor_id, float reference_level);
app_result_t sensor_calibrate_pressure(uint8_t sensor_id, float reference_pressure);
app_result_t sensor_calibrate_temperature(uint8_t sensor_id, float reference_temp);

/* 传感器配置接口 */
app_result_t sensor_set_level_range(uint8_t sensor_id, float tank_height);
app_result_t sensor_set_pressure_range(uint8_t sensor_id, float max_pressure);
app_result_t sensor_set_temp_range(uint8_t sensor_id, float min_temp, float max_temp);

/* 报警设置接口 */
app_result_t sensor_set_level_alarms(uint8_t sensor_id, float high_alarm, float low_alarm);
app_result_t sensor_set_pressure_alarms(uint8_t sensor_id, float high_alarm, float low_alarm);
app_result_t sensor_set_temp_alarms(uint8_t sensor_id, float high_alarm, float low_alarm);

/* 状态查询接口 */
app_result_t sensor_get_system_status(sensor_system_t* status);
app_result_t sensor_get_statistics(app_statistics_t* stats);
bool sensor_is_system_healthy(void);

/* 诊断接口 */
app_result_t sensor_self_test(void);
app_result_t sensor_check_wiring(uint8_t sensor_id, app_sensor_type_t type);
app_result_t sensor_reset_error_count(void);

#endif /* __SENSOR_TASK_H */