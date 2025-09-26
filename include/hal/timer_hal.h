/**
 * @file timer_hal.h
 * @brief 定时器硬件抽象层头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统定时器硬件抽象层，基于GD32F4xx HAL库
 */

#ifndef TIMER_HAL_H
#define TIMER_HAL_H

#include "hal/hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 定时器模式 */
typedef enum {
    TIMER_MODE_BASE = 0,        /* 基本定时器模式 */
    TIMER_MODE_OUTPUT_COMPARE = 1, /* 输出比较模式 */
    TIMER_MODE_INPUT_CAPTURE = 2,  /* 输入捕获模式 */
    TIMER_MODE_PWM = 3,         /* PWM模式 */
    TIMER_MODE_ONE_PULSE = 4,   /* 单脉冲模式 */
    TIMER_MODE_ENCODER = 5      /* 编码器模式 */
} timer_mode_t;

/* 定时器计数方向 */
typedef enum {
    TIMER_COUNT_UP = 0,         /* 向上计数 */
    TIMER_COUNT_DOWN = 1,       /* 向下计数 */
    TIMER_COUNT_CENTER_ALIGNED1 = 2, /* 中心对齐模式1 */
    TIMER_COUNT_CENTER_ALIGNED2 = 3, /* 中心对齐模式2 */
    TIMER_COUNT_CENTER_ALIGNED3 = 4  /* 中心对齐模式3 */
} timer_count_mode_t;

/* 定时器时钟分频 */
typedef enum {
    TIMER_CLOCK_DIV_1 = 0,      /* 不分频 */
    TIMER_CLOCK_DIV_2 = 1,      /* 2分频 */
    TIMER_CLOCK_DIV_4 = 2       /* 4分频 */
} timer_clock_division_t;

/* 定时器中断源 */
typedef enum {
    TIMER_IT_NONE = 0x00,       /* 无中断 */
    TIMER_IT_UPDATE = 0x01,     /* 更新中断 */
    TIMER_IT_CC1 = 0x02,        /* 捕获/比较1中断 */
    TIMER_IT_CC2 = 0x04,        /* 捕获/比较2中断 */
    TIMER_IT_CC3 = 0x08,        /* 捕获/比较3中断 */
    TIMER_IT_CC4 = 0x10,        /* 捕获/比较4中断 */
    TIMER_IT_TRIGGER = 0x40,    /* 触发中断 */
    TIMER_IT_BREAK = 0x80       /* 刹车中断 */
} timer_interrupt_t;

/* 定时器配置 */
typedef struct {
    hal_config_base_t base;         /* 基础配置 */
    uint32_t instance;              /* 定时器实例 */
    uint32_t prescaler;             /* 预分频器 */
    uint32_t period;                /* 自动重载值 */
    timer_count_mode_t count_mode;  /* 计数模式 */
    timer_clock_division_t clock_div; /* 时钟分频 */
    uint8_t repetition_counter;     /* 重复计数器 */
    bool auto_reload_preload;       /* 自动重载预装载使能 */
    timer_interrupt_t interrupts;   /* 中断使能 */
    hal_callback_t period_elapsed_callback; /* 周期结束回调 */
    hal_callback_t trigger_callback; /* 触发回调 */
    hal_callback_t break_callback;  /* 刹车回调 */
} timer_config_t;

/* 定时器句柄 */
typedef struct {
    hal_handle_base_t base;     /* 基础句柄 */
    timer_config_t* config;     /* 定时器配置 */
    timer_mode_t mode;          /* 当前模式 */
    volatile uint32_t counter;  /* 计数器值 */
    bool auto_reload;           /* 自动重载状态 */
    uint32_t channel_state;     /* 通道状态 */
} timer_handle_t;

/* 输出比较配置 */
typedef struct {
    uint32_t oc_mode;           /* 输出比较模式 */
    uint32_t pulse;             /* 脉冲值 */
    uint32_t oc_polarity;       /* 输出极性 */
    uint32_t oc_npolarity;      /* 互补输出极性 */
    uint32_t oc_fast_mode;      /* 快速模式 */
    uint32_t oc_idle_state;     /* 空闲状态 */
    uint32_t ocn_idle_state;    /* 互补输出空闲状态 */
} timer_oc_config_t;

/* 输入捕获配置 */
typedef struct {
    uint32_t ic_polarity;       /* 输入极性 */
    uint32_t ic_selection;      /* 输入选择 */
    uint32_t ic_prescaler;      /* 输入预分频 */
    uint32_t ic_filter;         /* 输入滤波 */
} timer_ic_config_t;

/* 基础定时器操作接口 */
hal_result_t timer_hal_init(timer_handle_t* handle, const timer_config_t* config);
hal_result_t timer_hal_deinit(timer_handle_t* handle);
hal_result_t timer_hal_start(timer_handle_t* handle);
hal_result_t timer_hal_stop(timer_handle_t* handle);
hal_result_t timer_hal_start_it(timer_handle_t* handle);
hal_result_t timer_hal_stop_it(timer_handle_t* handle);

/* 计数器操作接口 */
hal_result_t timer_hal_set_counter(timer_handle_t* handle, uint32_t counter);
hal_result_t timer_hal_get_counter(timer_handle_t* handle, uint32_t* counter);
hal_result_t timer_hal_set_prescaler(timer_handle_t* handle, uint32_t prescaler);
hal_result_t timer_hal_set_period(timer_handle_t* handle, uint32_t period);
hal_result_t timer_hal_generate_event(timer_handle_t* handle, uint32_t event_source);

/* 输出比较接口 */
hal_result_t timer_hal_oc_config_channel(timer_handle_t* handle, uint32_t channel, const timer_oc_config_t* config);
hal_result_t timer_hal_oc_start(timer_handle_t* handle, uint32_t channel);
hal_result_t timer_hal_oc_stop(timer_handle_t* handle, uint32_t channel);
hal_result_t timer_hal_oc_start_it(timer_handle_t* handle, uint32_t channel);
hal_result_t timer_hal_oc_stop_it(timer_handle_t* handle, uint32_t channel);

/* 输入捕获接口 */
hal_result_t timer_hal_ic_config_channel(timer_handle_t* handle, uint32_t channel, const timer_ic_config_t* config);
hal_result_t timer_hal_ic_start(timer_handle_t* handle, uint32_t channel);
hal_result_t timer_hal_ic_stop(timer_handle_t* handle, uint32_t channel);
hal_result_t timer_hal_ic_start_it(timer_handle_t* handle, uint32_t channel);
hal_result_t timer_hal_ic_stop_it(timer_handle_t* handle, uint32_t channel);
hal_result_t timer_hal_ic_get_value(timer_handle_t* handle, uint32_t channel, uint32_t* value);

/* 供墨系统专用定时器功能 */
typedef enum {
    TIMER_FUNCTION_SYSTEM_TICK = 0,     /* 系统时钟 */
    TIMER_FUNCTION_SENSOR_SAMPLE = 1,   /* 传感器采样定时 */
    TIMER_FUNCTION_CONTROL_LOOP = 2,    /* 控制环路定时 */
    TIMER_FUNCTION_COMMUNICATION = 3,   /* 通信超时定时 */
    TIMER_FUNCTION_SAFETY_CHECK = 4,    /* 安全检查定时 */
    TIMER_FUNCTION_LED_BLINK = 5,       /* LED闪烁定时 */
    TIMER_FUNCTION_WATCHDOG_FEED = 6,   /* 看门狗喂狗定时 */
    TIMER_FUNCTION_COUNT                /* 功能总数 */
} timer_function_id_t;

/* 供墨系统专用接口 */
hal_result_t timer_hal_system_init(void);
hal_result_t timer_hal_system_deinit(void);
hal_result_t timer_hal_setup_function_timer(timer_function_id_t function, uint32_t period_ms, hal_callback_t callback);
hal_result_t timer_hal_start_function_timer(timer_function_id_t function);
hal_result_t timer_hal_stop_function_timer(timer_function_id_t function);
hal_result_t timer_hal_get_system_time_ms(uint32_t* time_ms);
hal_result_t timer_hal_delay_precise_us(uint32_t delay_us);

/* 高精度时间测量 */
hal_result_t timer_hal_start_time_measurement(timer_handle_t* handle);
hal_result_t timer_hal_stop_time_measurement(timer_handle_t* handle, uint32_t* elapsed_us);
hal_result_t timer_hal_get_timestamp_us(uint64_t* timestamp);

/* 频率测量 */
hal_result_t timer_hal_measure_frequency(timer_handle_t* handle, uint32_t channel, uint32_t* frequency_hz);
hal_result_t timer_hal_measure_duty_cycle(timer_handle_t* handle, uint32_t channel, uint16_t* duty_cycle);

/* 编码器接口 */
hal_result_t timer_hal_encoder_init(timer_handle_t* handle, uint32_t encoder_mode);
hal_result_t timer_hal_encoder_start(timer_handle_t* handle);
hal_result_t timer_hal_encoder_stop(timer_handle_t* handle);
hal_result_t timer_hal_encoder_get_counter(timer_handle_t* handle, int32_t* counter);
hal_result_t timer_hal_encoder_reset_counter(timer_handle_t* handle);

/* 诊断和测试接口 */
hal_result_t timer_hal_self_test(timer_handle_t* handle);
hal_result_t timer_hal_calibrate_oscillator(void);
hal_result_t timer_hal_check_frequency_accuracy(timer_handle_t* handle, float* accuracy_percent);

/* 回调函数类型 */
typedef void (*timer_period_elapsed_callback_t)(timer_handle_t* handle);
typedef void (*timer_trigger_callback_t)(timer_handle_t* handle);
typedef void (*timer_break_callback_t)(timer_handle_t* handle);
typedef void (*timer_ic_capture_callback_t)(timer_handle_t* handle, uint32_t channel, uint32_t value);

/* 回调注册接口 */
hal_result_t timer_hal_register_period_callback(timer_handle_t* handle, timer_period_elapsed_callback_t callback);
hal_result_t timer_hal_register_trigger_callback(timer_handle_t* handle, timer_trigger_callback_t callback);
hal_result_t timer_hal_register_break_callback(timer_handle_t* handle, timer_break_callback_t callback);
hal_result_t timer_hal_register_ic_callback(timer_handle_t* handle, timer_ic_capture_callback_t callback);
hal_result_t timer_hal_unregister_callbacks(timer_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif /* TIMER_HAL_H */