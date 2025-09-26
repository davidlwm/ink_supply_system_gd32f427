/**
 * @file app_types.h
 * @brief 应用层通用类型定义 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 应用层所有模块使用的通用数据类型和常量定义
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "gd32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 系统配置常量 */
#define APP_MAX_SENSORS             8       /* 最大传感器数量 */
#define APP_MAX_ACTUATORS           6       /* 最大执行器数量 */
#define APP_MAX_CONTROLLERS         5       /* 最大控制器数量 */
#define APP_MAX_TASK_NAME_LEN       16      /* 任务名称最大长度 */
#define APP_MAX_ERROR_MSG_LEN       64      /* 错误消息最大长度 */

/* 传感器类型定义 */
typedef enum {
    SENSOR_TYPE_LIQUID_LEVEL = 0,   /* 液位传感器 */
    SENSOR_TYPE_PRESSURE = 1,       /* 压力传感器 */
    SENSOR_TYPE_TEMPERATURE = 2,    /* 温度传感器 */
    SENSOR_TYPE_FLOW = 3,           /* 流量传感器 */
    SENSOR_TYPE_COUNT               /* 传感器类型数量 */
} app_sensor_type_t;

/* 执行器类型定义 */
typedef enum {
    ACTUATOR_TYPE_HEATER = 0,       /* 加热器 */
    ACTUATOR_TYPE_PUMP = 1,         /* 泵 */
    ACTUATOR_TYPE_VALVE = 2,        /* 阀门 */
    ACTUATOR_TYPE_FAN = 3,          /* 风扇 */
    ACTUATOR_TYPE_COUNT             /* 执行器类型数量 */
} app_actuator_type_t;

/* 通用错误码 */
typedef enum {
    APP_RESULT_OK = 0,
    APP_RESULT_ERROR = 1,
    APP_RESULT_BUSY = 2,
    APP_RESULT_TIMEOUT = 3,
    APP_RESULT_INVALID_PARAM = 4,
    APP_RESULT_NOT_INITIALIZED = 5,
    APP_RESULT_NOT_SUPPORTED = 6,
    APP_RESULT_OUT_OF_MEMORY = 7,
    APP_RESULT_SENSOR_FAULT = 8,
    APP_RESULT_ACTUATOR_FAULT = 9,
    APP_RESULT_COMMUNICATION_ERROR = 10,
    APP_RESULT_SAFETY_INTERLOCK = 11
} app_result_t;

/* 传感器数据结构 */
typedef struct {
    uint8_t id;                     /* 传感器ID */
    app_sensor_type_t type;         /* 传感器类型 */
    float raw_value;                /* 原始值 */
    float filtered_value;           /* 滤波值 */
    float calibrated_value;         /* 校准值 */
    uint32_t timestamp;             /* 时间戳 */
    bool fault_status;              /* 故障状态 */
    uint16_t fault_code;            /* 故障代码 */
    const char* name;               /* 传感器名称 */
    const char* unit;               /* 单位 */
} app_sensor_data_t;

/* 执行器状态结构 */
typedef struct {
    uint8_t id;                     /* 执行器ID */
    app_actuator_type_t type;       /* 执行器类型 */
    float command_value;            /* 命令值 */
    float actual_value;             /* 实际值 */
    float output_percent;           /* 输出百分比 */
    bool enable_status;             /* 使能状态 */
    bool fault_status;              /* 故障状态 */
    uint16_t fault_code;            /* 故障代码 */
    uint32_t timestamp;             /* 时间戳 */
    const char* name;               /* 执行器名称 */
    const char* unit;               /* 单位 */
} app_actuator_status_t;

/* 控制器参数结构 */
typedef struct {
    uint8_t id;                     /* 控制器ID */
    float target_value;             /* 目标值 */
    float current_value;            /* 当前值 */
    float output_value;             /* 输出值 */
    float kp, ki, kd;               /* PID参数 */
    float integral_limit;           /* 积分限幅 */
    float output_limit_min;         /* 输出下限 */
    float output_limit_max;         /* 输出上限 */
    bool enable;                    /* 使能状态 */
    bool auto_mode;                 /* 自动模式 */
    uint32_t stable_time_ms;        /* 稳定时间 */
    const char* name;               /* 控制器名称 */
} app_controller_params_t;

/* 系统状态结构 */
typedef struct {
    bool system_ready;              /* 系统就绪状态 */
    bool emergency_stop;            /* 急停状态 */
    bool safety_interlock;          /* 安全联锁状态 */
    uint32_t fault_count;           /* 故障计数 */
    uint32_t warning_count;         /* 警告计数 */
    uint32_t uptime_seconds;        /* 运行时间 */
    float system_efficiency;        /* 系统效率 */
    uint8_t operation_mode;         /* 运行模式 */
} app_system_status_t;

/* 配置参数结构 */
typedef struct {
    /* 传感器配置 */
    struct {
        float calibration_offset;   /* 校准偏移 */
        float calibration_scale;    /* 校准比例 */
        float alarm_high_limit;     /* 高报警限值 */
        float alarm_low_limit;      /* 低报警限值 */
        uint16_t filter_window;     /* 滤波窗口 */
        bool enable;                /* 使能状态 */
    } sensor_config[APP_MAX_SENSORS];

    /* 执行器配置 */
    struct {
        float output_limit_min;     /* 输出下限 */
        float output_limit_max;     /* 输出上限 */
        float ramp_rate;            /* 变化率限制 */
        uint16_t fault_timeout_ms;  /* 故障超时 */
        bool enable;                /* 使能状态 */
    } actuator_config[APP_MAX_ACTUATORS];

    /* 控制器配置 */
    struct {
        float kp, ki, kd;           /* PID参数 */
        float integral_limit;       /* 积分限幅 */
        float deadband;             /* 死区 */
        uint16_t control_period_ms; /* 控制周期 */
        bool enable;                /* 使能状态 */
    } controller_config[APP_MAX_CONTROLLERS];

} app_config_params_t;

/* 统计信息结构 */
typedef struct {
    uint32_t sensor_read_count;     /* 传感器读取次数 */
    uint32_t actuator_write_count;  /* 执行器写入次数 */
    uint32_t control_loop_count;    /* 控制循环次数 */
    uint32_t communication_count;   /* 通信次数 */
    uint32_t error_count;           /* 错误次数 */
    uint32_t warning_count;         /* 警告次数 */
    float average_loop_time_ms;     /* 平均循环时间 */
    float max_loop_time_ms;         /* 最大循环时间 */
    float cpu_usage_percent;        /* CPU使用率 */
    uint32_t free_heap_bytes;       /* 剩余堆内存 */
} app_statistics_t;

/* 事件类型定义 */
typedef enum {
    APP_EVENT_SENSOR_FAULT = 0,
    APP_EVENT_ACTUATOR_FAULT = 1,
    APP_EVENT_CONTROL_FAULT = 2,
    APP_EVENT_COMMUNICATION_FAULT = 3,
    APP_EVENT_SYSTEM_START = 4,
    APP_EVENT_SYSTEM_STOP = 5,
    APP_EVENT_EMERGENCY_STOP = 6,
    APP_EVENT_SAFETY_INTERLOCK = 7,
    APP_EVENT_CONFIG_CHANGED = 8,
    APP_EVENT_CALIBRATION_DONE = 9,
    APP_EVENT_COUNT
} app_event_type_t;

/* 事件数据结构 */
typedef struct {
    app_event_type_t type;          /* 事件类型 */
    uint8_t source_id;              /* 事件源ID */
    uint32_t timestamp;             /* 时间戳 */
    uint32_t data;                  /* 事件数据 */
    char message[APP_MAX_ERROR_MSG_LEN]; /* 事件消息 */
} app_event_data_t;

/* 回调函数类型定义 */
typedef void (*app_sensor_callback_t)(const app_sensor_data_t* data);
typedef void (*app_actuator_callback_t)(const app_actuator_status_t* status);
typedef void (*app_event_callback_t)(const app_event_data_t* event);
typedef void (*app_error_callback_t)(const char* source, uint32_t error_code, const char* message);

#ifdef __cplusplus
}
#endif

#endif /* APP_TYPES_H */