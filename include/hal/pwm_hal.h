/**
 * @file pwm_hal.h
 * @brief PWM硬件抽象层头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统PWM硬件抽象层，基于GD32F4xx HAL库
 */

#ifndef PWM_HAL_H
#define PWM_HAL_H

#include "hal/hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PWM通道模式 */
typedef enum {
    PWM_MODE_PWM1 = 0,          /* PWM模式1 */
    PWM_MODE_PWM2 = 1,          /* PWM模式2 */
    PWM_MODE_COMPARE = 2,       /* 比较输出模式 */
    PWM_MODE_CAPTURE = 3        /* 输入捕获模式 */
} pwm_mode_t;

/* PWM极性 */
typedef enum {
    PWM_POLARITY_HIGH = 0,      /* 高电平有效 */
    PWM_POLARITY_LOW = 1        /* 低电平有效 */
} pwm_polarity_t;

/* PWM对齐模式 */
typedef enum {
    PWM_ALIGN_EDGE = 0,         /* 边沿对齐 */
    PWM_ALIGN_CENTER = 1        /* 中心对齐 */
} pwm_align_mode_t;

/* PWM中断源 */
typedef enum {
    PWM_IT_NONE = 0x00,         /* 无中断 */
    PWM_IT_UPDATE = 0x01,       /* 更新中断 */
    PWM_IT_CC1 = 0x02,          /* 捕获/比较1中断 */
    PWM_IT_CC2 = 0x04,          /* 捕获/比较2中断 */
    PWM_IT_CC3 = 0x08,          /* 捕获/比较3中断 */
    PWM_IT_CC4 = 0x10,          /* 捕获/比较4中断 */
    PWM_IT_BREAK = 0x20,        /* 刹车中断 */
    PWM_IT_TRIGGER = 0x40       /* 触发中断 */
} pwm_interrupt_t;

/* PWM通道配置 */
typedef struct {
    uint8_t channel;            /* 通道号 (1-4) */
    pwm_mode_t mode;            /* PWM模式 */
    pwm_polarity_t polarity;    /* 极性 */
    uint32_t pulse;             /* 脉冲宽度 */
    bool output_enable;         /* 输出使能 */
    bool complementary_enable;  /* 互补输出使能 */
    uint16_t dead_time;         /* 死区时间 */
} pwm_channel_config_t;

/* PWM配置 */
typedef struct {
    hal_config_base_t base;         /* 基础配置 */
    uint32_t instance;              /* 定时器实例 */
    uint32_t prescaler;             /* 预分频器 */
    uint32_t period;                /* 周期值 */
    uint32_t clock_division;        /* 时钟分频 */
    pwm_align_mode_t align_mode;    /* 对齐模式 */
    uint8_t repetition_counter;     /* 重复计数器 */
    bool auto_reload_preload;       /* 自动重载预装载 */
    pwm_interrupt_t interrupts;     /* 中断使能 */
    pwm_channel_config_t* channels; /* 通道配置数组 */
    uint8_t channel_count;          /* 通道数量 */
    hal_callback_t period_elapsed_callback; /* 周期结束回调 */
    hal_callback_t pulse_finished_callback; /* 脉冲完成回调 */
} pwm_config_t;

/* PWM句柄 */
typedef struct {
    hal_handle_base_t base;     /* 基础句柄 */
    pwm_config_t* config;       /* PWM配置 */
    uint32_t active_channels;   /* 活动通道掩码 */
    bool break_enabled;         /* 刹车功能使能 */
    uint32_t error_flags;       /* 错误标志 */
} pwm_handle_t;

/* 供墨系统PWM通道定义 */
typedef enum {
    /* 泵控制 */
    PWM_CHANNEL_PUMP_1 = 0,         /* 泵1调速控制 (MPB025BBB) */
    PWM_CHANNEL_PUMP_2,             /* 泵2调速控制 (MPB025BBB) */
    PWM_CHANNEL_PUMP_3,             /* 泵3调速控制 */
    PWM_CHANNEL_PUMP_4,             /* 泵4调速控制 */

    /* 加热器控制 */
    PWM_CHANNEL_HEATER_1,           /* 加热器1功率控制 (MRA-23D3) */
    PWM_CHANNEL_HEATER_2,           /* 加热器2功率控制 (MRA-23D3) */
    PWM_CHANNEL_HEATER_3,           /* 加热器3功率控制 (MRA-23D3) */

    /* 风扇控制 */
    PWM_CHANNEL_FAN_1,              /* 风扇1调速 */
    PWM_CHANNEL_FAN_2,              /* 风扇2调速 */

    /* LED亮度控制 */
    PWM_CHANNEL_LED_1,              /* LED1亮度控制 */
    PWM_CHANNEL_LED_2,              /* LED2亮度控制 */
    PWM_CHANNEL_LED_3,              /* LED3亮度控制 */
    PWM_CHANNEL_LED_4,              /* LED4亮度控制 */

    /* 备用通道 */
    PWM_CHANNEL_SPARE_1,            /* 备用通道1 */
    PWM_CHANNEL_SPARE_2,            /* 备用通道2 */

    PWM_CHANNEL_COUNT               /* 通道总数 */
} pwm_channel_id_t;

/* PWM输出状态 */
typedef struct {
    bool enabled;               /* 输出使能 */
    uint32_t frequency;         /* 频率(Hz) */
    uint16_t duty_cycle;        /* 占空比(0-10000, 表示0.00%-100.00%) */
    uint32_t pulse_width_us;    /* 脉冲宽度(微秒) */
    bool fault_state;           /* 故障状态 */
} pwm_output_state_t;

/* PWM测量结果 */
typedef struct {
    uint32_t frequency;         /* 测量频率 */
    uint16_t duty_cycle;        /* 测量占空比 */
    uint32_t pulse_width;       /* 脉冲宽度 */
    uint32_t period;            /* 周期 */
    bool valid;                 /* 测量数据有效性 */
} pwm_measurement_t;

/* 基础PWM操作接口 */
hal_result_t pwm_hal_init(pwm_handle_t* handle, const pwm_config_t* config);
hal_result_t pwm_hal_deinit(pwm_handle_t* handle);
hal_result_t pwm_hal_start(pwm_handle_t* handle, uint32_t channel_mask);
hal_result_t pwm_hal_stop(pwm_handle_t* handle, uint32_t channel_mask);
hal_result_t pwm_hal_start_it(pwm_handle_t* handle, uint32_t channel_mask);
hal_result_t pwm_hal_stop_it(pwm_handle_t* handle, uint32_t channel_mask);
hal_result_t pwm_hal_start_dma(pwm_handle_t* handle, uint32_t channel_mask, uint32_t* pulse_buffer, uint16_t length);
hal_result_t pwm_hal_stop_dma(pwm_handle_t* handle, uint32_t channel_mask);

/* PWM输出控制接口 */
hal_result_t pwm_hal_set_duty_cycle(pwm_handle_t* handle, pwm_channel_id_t channel, uint16_t duty_cycle);
hal_result_t pwm_hal_set_frequency(pwm_handle_t* handle, uint32_t frequency);
hal_result_t pwm_hal_set_pulse_width(pwm_handle_t* handle, pwm_channel_id_t channel, uint32_t pulse_width_us);
hal_result_t pwm_hal_set_polarity(pwm_handle_t* handle, pwm_channel_id_t channel, pwm_polarity_t polarity);

/* PWM状态查询接口 */
hal_result_t pwm_hal_get_output_state(pwm_handle_t* handle, pwm_channel_id_t channel, pwm_output_state_t* state);
hal_result_t pwm_hal_get_duty_cycle(pwm_handle_t* handle, pwm_channel_id_t channel, uint16_t* duty_cycle);
hal_result_t pwm_hal_get_frequency(pwm_handle_t* handle, uint32_t* frequency);

/* PWM输入捕获接口 */
hal_result_t pwm_hal_start_input_capture(pwm_handle_t* handle, pwm_channel_id_t channel);
hal_result_t pwm_hal_stop_input_capture(pwm_handle_t* handle, pwm_channel_id_t channel);
hal_result_t pwm_hal_get_capture_value(pwm_handle_t* handle, pwm_channel_id_t channel, uint32_t* value);
hal_result_t pwm_hal_measure_pwm(pwm_handle_t* handle, pwm_channel_id_t channel, pwm_measurement_t* measurement);

/* 供墨系统专用接口 */
hal_result_t pwm_hal_system_init(void);
hal_result_t pwm_hal_system_deinit(void);
hal_result_t pwm_hal_set_pump_speed(uint8_t pump_id, uint16_t speed_percent);
hal_result_t pwm_hal_set_heater_power(uint8_t heater_id, uint16_t power_percent);
hal_result_t pwm_hal_set_fan_speed(uint8_t fan_id, uint16_t speed_percent);
hal_result_t pwm_hal_set_led_brightness(uint8_t led_id, uint16_t brightness_percent);
hal_result_t pwm_hal_emergency_stop_all(void);

/* 高级功能接口 */
hal_result_t pwm_hal_enable_break_function(pwm_handle_t* handle, bool enable);
hal_result_t pwm_hal_set_dead_time(pwm_handle_t* handle, uint16_t dead_time);
hal_result_t pwm_hal_configure_complementary_output(pwm_handle_t* handle, pwm_channel_id_t channel, bool enable);

/* 同步和相位控制 */
hal_result_t pwm_hal_synchronize_timers(pwm_handle_t* master, pwm_handle_t* slave);
hal_result_t pwm_hal_set_phase_shift(pwm_handle_t* handle, pwm_channel_id_t channel, uint16_t phase_deg);

/* PWM波形生成 */
hal_result_t pwm_hal_generate_single_pulse(pwm_handle_t* handle, pwm_channel_id_t channel, uint32_t pulse_width_us);
hal_result_t pwm_hal_generate_burst(pwm_handle_t* handle, pwm_channel_id_t channel, uint16_t burst_count, uint32_t burst_period_us);

/* 诊断和测试接口 */
hal_result_t pwm_hal_self_test(pwm_handle_t* handle);
hal_result_t pwm_hal_test_channel_output(pwm_handle_t* handle, pwm_channel_id_t channel);
hal_result_t pwm_hal_check_signal_integrity(pwm_handle_t* handle, pwm_channel_id_t channel, bool* integrity_ok);

/* 回调函数类型 */
typedef void (*pwm_period_elapsed_callback_t)(pwm_handle_t* handle);
typedef void (*pwm_pulse_finished_callback_t)(pwm_handle_t* handle, pwm_channel_id_t channel);
typedef void (*pwm_break_callback_t)(pwm_handle_t* handle);

/* 回调注册接口 */
hal_result_t pwm_hal_register_period_callback(pwm_handle_t* handle, pwm_period_elapsed_callback_t callback);
hal_result_t pwm_hal_register_pulse_callback(pwm_handle_t* handle, pwm_pulse_finished_callback_t callback);
hal_result_t pwm_hal_register_break_callback(pwm_handle_t* handle, pwm_break_callback_t callback);
hal_result_t pwm_hal_unregister_callbacks(pwm_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif /* PWM_HAL_H */