/**
 * @file pid.h
 * @brief PID控制器头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description PID控制器模块，支持多种PID算法和高级功能
 */

#ifndef PID_H
#define PID_H

#include "middleware_common.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =================================================================== */
/* PID控制器类型定义 */
/* =================================================================== */
typedef enum {
    PID_TYPE_STANDARD = 0,              /* 标准PID */
    PID_TYPE_INCREMENTAL = 1,           /* 增量式PID */
    PID_TYPE_VELOCITY = 2,              /* 速度式PID */
    PID_TYPE_ADAPTIVE = 3,              /* 自适应PID */
    PID_TYPE_COUNT                      /* PID类型数量 */
} pid_type_t;

/* PID控制器状态定义 */
typedef enum {
    PID_STATE_IDLE = 0,                 /* 空闲状态 */
    PID_STATE_RUNNING = 1,              /* 运行状态 */
    PID_STATE_SATURATED = 2,            /* 饱和状态 */
    PID_STATE_ERROR = 3,                /* 错误状态 */
    PID_STATE_TUNING = 4                /* 调试状态 */
} pid_state_t;

/* PID工作模式定义 */
typedef enum {
    PID_MODE_MANUAL = 0,                /* 手动模式 */
    PID_MODE_AUTOMATIC = 1,             /* 自动模式 */
    PID_MODE_CASCADE = 2,               /* 串级模式 */
    PID_MODE_FEEDFORWARD = 3            /* 前馈模式 */
} pid_mode_t;

/* =================================================================== */
/* PID参数结构 */
/* =================================================================== */
typedef struct {
    float kp;                           /* 比例系数 */
    float ki;                           /* 积分系数 */
    float kd;                           /* 微分系数 */
    float dt;                           /* 采样时间(秒) */
} pid_params_t;

/* PID限制参数结构 */
typedef struct {
    float output_min;                   /* 输出下限 */
    float output_max;                   /* 输出上限 */
    float integral_min;                 /* 积分下限 */
    float integral_max;                 /* 积分上限 */
    float derivative_limit;             /* 微分限制 */
    float setpoint_rate_limit;          /* 设定值变化率限制 */
} pid_limits_t;

/* =================================================================== */
/* 标准PID控制器结构 */
/* =================================================================== */
typedef struct {
    /* 基础参数 */
    pid_params_t params;                /* PID参数 */
    pid_limits_t limits;                /* 限制参数 */
    pid_type_t type;                    /* PID类型 */
    pid_mode_t mode;                    /* 工作模式 */
    pid_state_t state;                  /* 控制器状态 */

    /* 控制变量 */
    float setpoint;                     /* 设定值 */
    float feedback;                     /* 反馈值 */
    float output;                       /* 控制输出 */
    float error;                        /* 当前误差 */
    float prev_error;                   /* 前一次误差 */
    float integral;                     /* 积分项 */
    float derivative;                   /* 微分项 */

    /* 增量式PID专用 */
    float prev_feedback;                /* 前一次反馈值 */
    float prev_prev_error;              /* 前前次误差 */

    /* 自适应参数 */
    float adaptive_factor;              /* 自适应因子 */
    float error_threshold;              /* 误差阈值 */

    /* 状态标志 */
    bool initialized;                   /* 初始化标志 */
    bool enabled;                       /* 使能标志 */
    bool anti_windup;                   /* 抗积分饱和使能 */
    bool derivative_on_measurement;     /* 微分作用于测量值 */

    /* 统计和诊断 */
    mw_statistics_t stats;              /* 统计信息 */
    uint32_t compute_count;             /* 计算次数 */
    uint32_t saturation_count;          /* 饱和次数 */
    float max_error;                    /* 最大误差 */
    float min_error;                    /* 最小误差 */
    float avg_error;                    /* 平均误差 */
} pid_controller_t;

/* =================================================================== */
/* PID系统管理结构 */
/* =================================================================== */
typedef struct {
    pid_type_t type;                    /* PID类型 */
    uint8_t id;                         /* PID控制器ID */
    bool in_use;                        /* 使用标志 */
    const char* name;                   /* 控制器名称 */
    pid_controller_t controller;        /* PID控制器实例 */
    mw_timestamp_t created_time;        /* 创建时间 */
    mw_timestamp_t last_update_time;    /* 最后更新时间 */
} pid_instance_t;

/* PID系统状态 */
typedef struct {
    uint8_t total_controllers;          /* 总控制器数量 */
    uint8_t active_controllers;         /* 激活控制器数量 */
    uint8_t standard_controllers;       /* 标准PID数量 */
    uint8_t incremental_controllers;    /* 增量式PID数量 */
    uint8_t velocity_controllers;       /* 速度式PID数量 */
    uint8_t adaptive_controllers;       /* 自适应PID数量 */
    mw_statistics_t total_stats;        /* 总体统计信息 */
} pid_system_status_t;

/* =================================================================== */
/* PID模块接口函数 */
/* =================================================================== */

/* PID系统初始化/反初始化 */
mw_result_t pid_system_init(void);
mw_result_t pid_system_deinit(void);

/* =================================================================== */
/* 基础PID控制器接口 */
/* =================================================================== */

/* 初始化和配置 */
mw_result_t pid_init(pid_controller_t* pid, pid_type_t type, const pid_params_t* params);
mw_result_t pid_reset(pid_controller_t* pid);
mw_result_t pid_deinit(pid_controller_t* pid);

/* 核心控制函数 */
mw_result_t pid_compute(pid_controller_t* pid, float setpoint, float feedback, float* output);
mw_result_t pid_compute_manual(pid_controller_t* pid, float manual_output);

/* 参数设置 */
mw_result_t pid_set_params(pid_controller_t* pid, const pid_params_t* params);
mw_result_t pid_get_params(const pid_controller_t* pid, pid_params_t* params);
mw_result_t pid_set_limits(pid_controller_t* pid, const pid_limits_t* limits);
mw_result_t pid_get_limits(const pid_controller_t* pid, pid_limits_t* limits);

/* 工作模式控制 */
mw_result_t pid_set_mode(pid_controller_t* pid, pid_mode_t mode);
pid_mode_t pid_get_mode(const pid_controller_t* pid);
mw_result_t pid_enable(pid_controller_t* pid, bool enable);
bool pid_is_enabled(const pid_controller_t* pid);

/* 高级功能设置 */
mw_result_t pid_set_anti_windup(pid_controller_t* pid, bool enable);
mw_result_t pid_set_derivative_on_measurement(pid_controller_t* pid, bool enable);
mw_result_t pid_set_adaptive_params(pid_controller_t* pid, float factor, float threshold);

/* 状态查询 */
pid_state_t pid_get_state(const pid_controller_t* pid);
mw_result_t pid_get_outputs(const pid_controller_t* pid, float* output, float* error, float* integral, float* derivative);
mw_result_t pid_get_statistics(const pid_controller_t* pid, mw_statistics_t* stats);

/* 统计和诊断 */
mw_result_t pid_reset_statistics(pid_controller_t* pid);
mw_result_t pid_get_error_statistics(const pid_controller_t* pid, float* max_error, float* min_error, float* avg_error);

/* =================================================================== */
/* 高级PID管理接口 */
/* =================================================================== */

/* PID实例管理 */
int pid_create(pid_type_t type, const char* name, const pid_params_t* params);
mw_result_t pid_destroy(int pid_id);
mw_result_t pid_compute_by_id(int pid_id, float setpoint, float feedback, float* output);
mw_result_t pid_reset_by_id(int pid_id);

/* 参数管理 */
mw_result_t pid_set_params_by_id(int pid_id, const pid_params_t* params);
mw_result_t pid_get_params_by_id(int pid_id, pid_params_t* params);
mw_result_t pid_set_limits_by_id(int pid_id, const pid_limits_t* limits);
mw_result_t pid_get_limits_by_id(int pid_id, pid_limits_t* limits);

/* 模式控制 */
mw_result_t pid_set_mode_by_id(int pid_id, pid_mode_t mode);
pid_mode_t pid_get_mode_by_id(int pid_id);
mw_result_t pid_enable_by_id(int pid_id, bool enable);
bool pid_is_enabled_by_id(int pid_id);

/* 快捷设置函数 */
mw_result_t pid_set_tuning_by_id(int pid_id, float kp, float ki, float kd);
mw_result_t pid_set_output_limits_by_id(int pid_id, float min, float max);
mw_result_t pid_set_sample_time_by_id(int pid_id, float dt);

/* =================================================================== */
/* 兼容性接口 - 为应用层提供简化接口 */
/* =================================================================== */

/* 简化的兼容接口 */
mw_result_t pid_controller_init(pid_controller_t* pid, float kp, float ki, float kd);
mw_result_t pid_controller_update(pid_controller_t* pid, float error, float dt);

/* 系统状态查询 */
mw_result_t pid_get_system_status(pid_system_status_t* status);
uint8_t pid_get_count(void);
uint8_t pid_get_count_by_type(pid_type_t type);

/* 批量操作 */
mw_result_t pid_reset_all(void);
mw_result_t pid_destroy_all(void);
mw_result_t pid_enable_all(bool enable);
mw_result_t pid_reset_all_statistics(void);

/* 查找和枚举 */
int pid_find_by_name(const char* name);
mw_result_t pid_get_info(int pid_id, pid_instance_t* info);
mw_result_t pid_enum_all(int* pid_ids, uint8_t max_count, uint8_t* actual_count);

/* 自动调优接口 */
mw_result_t pid_auto_tune_start(int pid_id, float amplitude, float frequency);
mw_result_t pid_auto_tune_stop(int pid_id);
bool pid_auto_tune_is_running(int pid_id);
mw_result_t pid_auto_tune_get_results(int pid_id, pid_params_t* tuned_params);

/* 诊断和调试 */
mw_result_t pid_self_test(void);
bool pid_is_valid_id(int pid_id);
mw_result_t pid_dump_status(void);

/* =================================================================== */
/* 工具函数 */
/* =================================================================== */

/* 参数转换和计算 */
mw_result_t pid_convert_to_incremental_params(const pid_params_t* standard, pid_params_t* incremental);
mw_result_t pid_convert_to_velocity_params(const pid_params_t* standard, pid_params_t* velocity);
mw_result_t pid_calculate_settling_time(const pid_controller_t* pid, float tolerance, float* settling_time);
mw_result_t pid_calculate_overshoot(const pid_controller_t* pid, float* overshoot_percent);

/* 性能评估 */
mw_result_t pid_evaluate_performance(const pid_controller_t* pid, float* ise, float* iae, float* itae);
mw_result_t pid_get_controller_health(const pid_controller_t* pid, float* health_score);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */