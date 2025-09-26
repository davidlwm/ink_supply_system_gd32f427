/**
 * @file filter.h
 * @brief 数字滤波器头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 数字滤波器模块，支持滑动平均和卡尔曼滤波
 */

#ifndef FILTER_H
#define FILTER_H

#include "middleware_common.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =================================================================== */
/* 滤波器类型定义 */
/* =================================================================== */
typedef enum {
    FILTER_TYPE_MOVING_AVERAGE = 0,     /* 滑动平均滤波器 */
    FILTER_TYPE_KALMAN = 1,             /* 卡尔曼滤波器 */
    FILTER_TYPE_LOW_PASS = 2,           /* 低通滤波器 */
    FILTER_TYPE_HIGH_PASS = 3,          /* 高通滤波器 */
    FILTER_TYPE_COUNT                   /* 滤波器类型数量 */
} filter_type_t;

/* 滤波器状态定义 */
typedef enum {
    FILTER_STATE_IDLE = 0,              /* 空闲状态 */
    FILTER_STATE_ACTIVE = 1,            /* 激活状态 */
    FILTER_STATE_SATURATED = 2,         /* 饱和状态 */
    FILTER_STATE_ERROR = 3              /* 错误状态 */
} filter_state_t;

/* =================================================================== */
/* 滑动平均滤波器 */
/* =================================================================== */
#define MOVING_AVERAGE_MAX_WINDOW   64  /* 最大窗口大小 */

typedef struct {
    float buffer[MOVING_AVERAGE_MAX_WINDOW]; /* 数据缓冲区 */
    uint8_t window_size;                /* 窗口大小 */
    uint8_t index;                      /* 当前索引 */
    uint8_t count;                      /* 有效数据数量 */
    float sum;                          /* 数据总和 */
    float output;                       /* 当前输出 */
    filter_state_t state;               /* 滤波器状态 */
    bool initialized;                   /* 初始化标志 */
    mw_statistics_t stats;              /* 统计信息 */
} moving_average_filter_t;

/* =================================================================== */
/* 卡尔曼滤波器 */
/* =================================================================== */
typedef struct {
    /* 卡尔曼滤波器参数 */
    float Q;                            /* 过程噪声协方差 */
    float R;                            /* 测量噪声协方差 */
    float P;                            /* 估计误差协方差 */
    float K;                            /* 卡尔曼增益 */
    float x;                            /* 状态估计值 */
    float x_pred;                       /* 预测状态值 */

    /* 状态信息 */
    float output;                       /* 当前输出 */
    filter_state_t state;               /* 滤波器状态 */
    bool initialized;                   /* 初始化标志 */
    mw_statistics_t stats;              /* 统计信息 */
} kalman_filter_t;

/* =================================================================== */
/* 一阶低通滤波器 */
/* =================================================================== */
typedef struct {
    float alpha;                        /* 滤波系数 (0-1) */
    float cutoff_freq;                  /* 截止频率 */
    float sample_freq;                  /* 采样频率 */
    float output;                       /* 当前输出 */
    float prev_output;                  /* 前一次输出 */
    filter_state_t state;               /* 滤波器状态 */
    bool initialized;                   /* 初始化标志 */
    mw_statistics_t stats;              /* 统计信息 */
} low_pass_filter_t;

/* =================================================================== */
/* 一阶高通滤波器 */
/* =================================================================== */
typedef struct {
    float alpha;                        /* 滤波系数 (0-1) */
    float cutoff_freq;                  /* 截止频率 */
    float sample_freq;                  /* 采样频率 */
    float output;                       /* 当前输出 */
    float prev_input;                   /* 前一次输入 */
    float prev_output;                  /* 前一次输出 */
    filter_state_t state;               /* 滤波器状态 */
    bool initialized;                   /* 初始化标志 */
    mw_statistics_t stats;              /* 统计信息 */
} high_pass_filter_t;

/* =================================================================== */
/* 通用滤波器管理结构 */
/* =================================================================== */
typedef struct {
    filter_type_t type;                 /* 滤波器类型 */
    uint8_t id;                         /* 滤波器ID */
    bool in_use;                        /* 使用标志 */
    const char* name;                   /* 滤波器名称 */

    union {
        moving_average_filter_t ma;     /* 滑动平均滤波器 */
        kalman_filter_t kalman;         /* 卡尔曼滤波器 */
        low_pass_filter_t lowpass;      /* 低通滤波器 */
        high_pass_filter_t highpass;    /* 高通滤波器 */
    } filter;

    mw_timestamp_t created_time;        /* 创建时间 */
    mw_timestamp_t last_update_time;    /* 最后更新时间 */
} filter_instance_t;

/* 滤波器系统状态 */
typedef struct {
    uint8_t total_filters;              /* 总滤波器数量 */
    uint8_t active_filters;             /* 激活滤波器数量 */
    uint8_t ma_filters;                 /* 滑动平均滤波器数量 */
    uint8_t kalman_filters;             /* 卡尔曼滤波器数量 */
    uint8_t lowpass_filters;            /* 低通滤波器数量 */
    uint8_t highpass_filters;           /* 高通滤波器数量 */
    mw_statistics_t total_stats;        /* 总体统计信息 */
} filter_system_status_t;

/* =================================================================== */
/* 滤波器模块接口函数 */
/* =================================================================== */

/* 滤波器系统初始化/反初始化 */
mw_result_t filter_system_init(void);
mw_result_t filter_system_deinit(void);

/* =================================================================== */
/* 滑动平均滤波器接口 */
/* =================================================================== */

/* 基础操作 */
mw_result_t ma_filter_init(moving_average_filter_t* filter, uint8_t window_size);
mw_result_t ma_filter_reset(moving_average_filter_t* filter);
mw_result_t ma_filter_process(moving_average_filter_t* filter, float input, float* output);

/* 参数设置 */
mw_result_t ma_filter_set_window_size(moving_average_filter_t* filter, uint8_t window_size);
uint8_t ma_filter_get_window_size(const moving_average_filter_t* filter);
filter_state_t ma_filter_get_state(const moving_average_filter_t* filter);

/* 统计信息 */
mw_result_t ma_filter_get_statistics(const moving_average_filter_t* filter, mw_statistics_t* stats);
mw_result_t ma_filter_reset_statistics(moving_average_filter_t* filter);

/* =================================================================== */
/* 卡尔曼滤波器接口 */
/* =================================================================== */

/* 基础操作 */
mw_result_t kalman_filter_init(kalman_filter_t* filter, float Q, float R, float initial_estimate);
mw_result_t kalman_filter_reset(kalman_filter_t* filter, float initial_estimate);
mw_result_t kalman_filter_process(kalman_filter_t* filter, float measurement, float* output);

/* 参数设置 */
mw_result_t kalman_filter_set_noise(kalman_filter_t* filter, float process_noise, float measurement_noise);
mw_result_t kalman_filter_get_noise(const kalman_filter_t* filter, float* process_noise, float* measurement_noise);
mw_result_t kalman_filter_get_gain(const kalman_filter_t* filter, float* gain);
filter_state_t kalman_filter_get_state(const kalman_filter_t* filter);

/* 统计信息 */
mw_result_t kalman_filter_get_statistics(const kalman_filter_t* filter, mw_statistics_t* stats);
mw_result_t kalman_filter_reset_statistics(kalman_filter_t* filter);

/* =================================================================== */
/* 低通滤波器接口 */
/* =================================================================== */

/* 基础操作 */
mw_result_t lowpass_filter_init(low_pass_filter_t* filter, float cutoff_freq, float sample_freq);
mw_result_t lowpass_filter_reset(low_pass_filter_t* filter);
mw_result_t lowpass_filter_process(low_pass_filter_t* filter, float input, float* output);

/* 参数设置 */
mw_result_t lowpass_filter_set_cutoff_freq(low_pass_filter_t* filter, float cutoff_freq);
float lowpass_filter_get_cutoff_freq(const low_pass_filter_t* filter);
filter_state_t lowpass_filter_get_state(const low_pass_filter_t* filter);

/* =================================================================== */
/* 高通滤波器接口 */
/* =================================================================== */

/* 基础操作 */
mw_result_t highpass_filter_init(high_pass_filter_t* filter, float cutoff_freq, float sample_freq);
mw_result_t highpass_filter_reset(high_pass_filter_t* filter);
mw_result_t highpass_filter_process(high_pass_filter_t* filter, float input, float* output);

/* 参数设置 */
mw_result_t highpass_filter_set_cutoff_freq(high_pass_filter_t* filter, float cutoff_freq);
float highpass_filter_get_cutoff_freq(const high_pass_filter_t* filter);
filter_state_t highpass_filter_get_state(const high_pass_filter_t* filter);

/* =================================================================== */
/* 高级滤波器管理接口 */
/* =================================================================== */

/* 滤波器实例管理 */
int filter_create(filter_type_t type, const char* name);
mw_result_t filter_destroy(int filter_id);
mw_result_t filter_process_by_id(int filter_id, float input, float* output);
mw_result_t filter_reset_by_id(int filter_id);

/* 滑动平均滤波器管理 */
int filter_create_moving_average(const char* name, uint8_t window_size);
mw_result_t filter_ma_set_window_size_by_id(int filter_id, uint8_t window_size);

/* 卡尔曼滤波器管理 */
int filter_create_kalman(const char* name, float Q, float R, float initial_estimate);
mw_result_t filter_kalman_set_noise_by_id(int filter_id, float process_noise, float measurement_noise);

/* 低通滤波器管理 */
int filter_create_lowpass(const char* name, float cutoff_freq, float sample_freq);
mw_result_t filter_lowpass_set_cutoff_by_id(int filter_id, float cutoff_freq);

/* 高通滤波器管理 */
int filter_create_highpass(const char* name, float cutoff_freq, float sample_freq);
mw_result_t filter_highpass_set_cutoff_by_id(int filter_id, float cutoff_freq);

/* =================================================================== */
/* 系统级接口 */
/* =================================================================== */

/* 系统状态查询 */
mw_result_t filter_get_system_status(filter_system_status_t* status);
uint8_t filter_get_count(void);
uint8_t filter_get_count_by_type(filter_type_t type);

/* 批量操作 */
mw_result_t filter_reset_all(void);
mw_result_t filter_destroy_all(void);
mw_result_t filter_reset_all_statistics(void);

/* 查找和枚举 */
int filter_find_by_name(const char* name);
mw_result_t filter_get_info(int filter_id, filter_instance_t* info);
mw_result_t filter_enum_all(int* filter_ids, uint8_t max_count, uint8_t* actual_count);

/* 诊断和调试 */
mw_result_t filter_self_test(void);
bool filter_is_valid_id(int filter_id);
mw_result_t filter_dump_status(void);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */