/**
 * @file    control_task.h
 * @brief   控制任务头文件 - 8周v4标准
 * @version V4.0
 * @date    2024-12-27
 */

#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "app_types.h"
#include "middleware/pid.h"
#include "middleware/filter.h"
#include <stdint.h>
#include <stdbool.h>

/* 控制任务特有配置 */
#define CONTROL_TASK_PRIORITY       APP_PRIORITY_HIGH
#define CONTROL_TASK_STACK_SIZE     APP_STACK_SIZE_LARGE
#define CONTROL_TASK_PERIOD_MS      20      /* 控制周期20ms */

/* 控制器数量定义 */
#define TEMPERATURE_CONTROLLER_COUNT    3   /* 温度控制器数量 */
#define PRESSURE_CONTROLLER_COUNT       2   /* 压力控制器数量 */
#define LEVEL_CONTROLLER_COUNT          2   /* 液位控制器数量 */

/* 控制模式定义 */
typedef enum {
    CONTROL_MODE_MANUAL = 0,
    CONTROL_MODE_AUTOMATIC,
    CONTROL_MODE_CALIBRATION,
    CONTROL_MODE_TEST,
    CONTROL_MODE_SAFETY
} control_mode_t;

/* 控制结果定义 */
typedef enum {
    CONTROL_SUCCESS = 0,
    CONTROL_ERROR = 1,
    CONTROL_ERROR_MEMORY_ALLOCATION = 2,
    CONTROL_ERROR_TIMEOUT = 3,
    CONTROL_ERROR_INVALID_PARAMETER = 4,
    CONTROL_ERROR_NOT_INITIALIZED = 5
} control_result_t;

/* 传感器结果常量定义 */
#define SENSOR_SUCCESS  0

/* 控制配置结构 */
typedef struct {
    struct {
        struct { float kp, ki, kd; } pid_params[TEMPERATURE_CONTROLLER_COUNT];
        float target_temps[TEMPERATURE_CONTROLLER_COUNT];
        float max_powers[TEMPERATURE_CONTROLLER_COUNT];
        float temp_tolerance;
        float overshoot_limit;
    } temperature_control;

    struct {
        struct { float kp, ki, kd; } pid_params[PRESSURE_CONTROLLER_COUNT];
        float target_pressures[PRESSURE_CONTROLLER_COUNT];
        uint16_t max_speeds[PRESSURE_CONTROLLER_COUNT];
        float pressure_tolerance;
        uint16_t min_pump_speed;
    } pressure_control;

    struct {
        float target_levels[LEVEL_CONTROLLER_COUNT];
        float hysteresis[LEVEL_CONTROLLER_COUNT];
        uint32_t min_action_interval;
    } level_control;
} control_config_t;

/* 控制系统状态信息 */
typedef struct {
    bool control_enabled;
    bool auto_mode;
    uint8_t control_mode;
    uint32_t control_cycle_count;
    float system_efficiency;

    struct {
        float target;
        float current;
        float output;
        bool enabled;
    } temperature_status[TEMPERATURE_CONTROLLER_COUNT];

    struct {
        float target;
        float current;
        uint16_t output;
        bool enabled;
    } pressure_status[PRESSURE_CONTROLLER_COUNT];

} control_system_status_info_t;

/* 温度控制器结构 */
typedef struct {
    pid_controller_t pid;           /* PID控制器 */
    moving_average_filter_t filter; /* 滤波器 */
    float target_temp;              /* 目标温度 */
    float current_temp;             /* 当前温度 */
    float output_power;             /* 输出功率 */
    bool enable;                    /* 使能状态 */
    uint32_t stable_count;          /* 稳定计数 */
    float max_power;                /* 最大功率限制 */
    uint8_t heater_id;              /* 对应加热器ID */
} temperature_controller_t;

/* 压力控制器结构 */
typedef struct {
    pid_controller_t pid;           /* PID控制器 */
    moving_average_filter_t filter; /* 滤波器 */
    float target_pressure;          /* 目标压力 */
    float current_pressure;         /* 当前压力 */
    uint16_t pump_speed;            /* 泵转速输出 */
    bool enable;                    /* 使能状态 */
    uint32_t stable_count;          /* 稳定计数 */
    uint16_t max_speed;             /* 最大转速限制 */
    uint8_t pump_id;                /* 对应泵ID */
} pressure_controller_t;

/* 液位控制器结构 */
typedef struct {
    float target_level;             /* 目标液位 */
    float current_level;            /* 当前液位 */
    float hysteresis;               /* 滞环 */
    bool valve_state;               /* 阀门状态 */
    uint32_t last_action_time;      /* 上次动作时间 */
    uint32_t min_action_interval;   /* 最小动作间隔 */
    uint8_t valve_id;               /* 对应阀门ID */
} level_controller_t;

/* 控制系统状态 */
typedef struct {
    bool control_enabled;           /* 控制使能状态 */
    bool auto_mode;                 /* 自动/手动模式 */
    control_mode_t control_mode;    /* 控制模式 */
    uint32_t control_cycle_count;   /* 控制周期计数 */
    float system_efficiency;        /* 系统效率 */
    temperature_controller_t temp_controllers[TEMPERATURE_CONTROLLER_COUNT];
    pressure_controller_t pressure_controllers[PRESSURE_CONTROLLER_COUNT];
    level_controller_t level_controllers[LEVEL_CONTROLLER_COUNT];
} control_system_t;

/* 任务函数 */
void control_task(void *pvParameters);
app_result_t control_manager_init(void);
app_result_t control_manager_deinit(void);

/* 控制系统操作函数 */
app_result_t control_enable_system(void);
app_result_t control_disable_system(void);
app_result_t control_set_mode(control_mode_t mode);
control_mode_t control_get_mode(void);

/* 温度控制接口 */
app_result_t control_set_temperature_target(uint8_t controller_id, float target_temp);
app_result_t control_get_temperature_status(uint8_t controller_id, app_controller_params_t* status);
app_result_t control_enable_temperature_controller(uint8_t controller_id, bool enable);

/* 压力控制接口 */
app_result_t control_set_pressure_target(uint8_t controller_id, float target_pressure);
app_result_t control_get_pressure_status(uint8_t controller_id, app_controller_params_t* status);
app_result_t control_enable_pressure_controller(uint8_t controller_id, bool enable);

/* 液位控制接口 */
app_result_t control_set_level_target(uint8_t controller_id, float target_level);
app_result_t control_get_level_status(uint8_t controller_id, app_controller_params_t* status);
app_result_t control_set_level_hysteresis(uint8_t controller_id, float hysteresis);

/* 系统状态接口 */
app_result_t control_get_system_status(control_system_t* status);
app_result_t control_get_statistics(app_statistics_t* stats);

/* PID参数调整接口 */
app_result_t control_set_pid_params(uint8_t controller_id, app_actuator_type_t type, float kp, float ki, float kd);
app_result_t control_get_pid_params(uint8_t controller_id, app_actuator_type_t type, float* kp, float* ki, float* kd);
app_result_t control_reset_pid_integral(uint8_t controller_id, app_actuator_type_t type);

#endif /* __CONTROL_TASK_H */