/**
 * @file gpio_hal.h
 * @brief GPIO硬件抽象层头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统GPIO硬件抽象层，基于GD32F4xx HAL库
 */

#ifndef GPIO_HAL_H
#define GPIO_HAL_H

#include "hal/hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GPIO引脚模式 */
typedef enum {
    GPIO_MODE_INPUT = 0,        /* 输入模式 */
    GPIO_MODE_OUTPUT = 1,       /* 输出模式 */
    GPIO_MODE_AF = 2,           /* 复用模式 */
    GPIO_MODE_ANALOG = 3        /* 模拟模式 */
} gpio_mode_t;

/* GPIO输出类型 */
typedef enum {
    GPIO_OUTPUT_PUSH_PULL = 0,  /* 推挽输出 */
    GPIO_OUTPUT_OPEN_DRAIN = 1  /* 开漏输出 */
} gpio_output_type_t;

/* GPIO上下拉 */
typedef enum {
    GPIO_PULL_NONE = 0,         /* 无上下拉 */
    GPIO_PULL_UP = 1,           /* 上拉 */
    GPIO_PULL_DOWN = 2          /* 下拉 */
} gpio_pull_t;

/* GPIO速度 */
typedef enum {
    GPIO_SPEED_LOW = 0,         /* 低速 */
    GPIO_SPEED_MEDIUM = 1,      /* 中速 */
    GPIO_SPEED_HIGH = 2,        /* 高速 */
    GPIO_SPEED_VERY_HIGH = 3    /* 极高速 */
} gpio_speed_t;

/* GPIO电平 */
typedef enum {
    GPIO_LEVEL_LOW = 0,         /* 低电平 */
    GPIO_LEVEL_HIGH = 1         /* 高电平 */
} gpio_level_t;

/* GPIO中断触发模式 */
typedef enum {
    GPIO_IT_NONE = 0,           /* 无中断 */
    GPIO_IT_RISING = 1,         /* 上升沿触发 */
    GPIO_IT_FALLING = 2,        /* 下降沿触发 */
    GPIO_IT_BOTH = 3            /* 双边沿触发 */
} gpio_interrupt_mode_t;

/* GPIO引脚配置 */
typedef struct {
    hal_config_base_t base;         /* 基础配置 */
    uint32_t port;                  /* GPIO端口 */
    uint32_t pin;                   /* GPIO引脚 */
    gpio_mode_t mode;               /* 引脚模式 */
    gpio_output_type_t output_type; /* 输出类型 */
    gpio_pull_t pull;               /* 上下拉 */
    gpio_speed_t speed;             /* 速度 */
    uint32_t alternate;             /* 复用功能 */
    gpio_interrupt_mode_t int_mode; /* 中断模式 */
    hal_callback_t int_callback;    /* 中断回调 */
} gpio_config_t;

/* GPIO句柄 */
typedef struct {
    hal_handle_base_t base;     /* 基础句柄 */
    gpio_config_t* config;      /* GPIO配置 */
    gpio_level_t current_level; /* 当前电平 */
    bool interrupt_enabled;     /* 中断使能状态 */
} gpio_handle_t;

/* 供墨系统GPIO引脚定义 */
typedef enum {
    /* 执行器控制引脚 */
    GPIO_HEATER_1 = 0,          /* 加热器1控制 */
    GPIO_HEATER_2,              /* 加热器2控制 */
    GPIO_HEATER_3,              /* 加热器3控制 */
    GPIO_VALVE_1,               /* 电磁阀1控制 */
    GPIO_VALVE_2,               /* 电磁阀2控制 */
    GPIO_VALVE_3,               /* 电磁阀3控制 */
    GPIO_VALVE_4,               /* 电磁阀4控制 */
    GPIO_VALVE_5,               /* 电磁阀5控制 */
    GPIO_VALVE_6,               /* 电磁阀6控制 */
    GPIO_VALVE_7,               /* 电磁阀7控制 */
    GPIO_VALVE_8,               /* 电磁阀8控制 */

    /* LED指示灯 */
    GPIO_LED_POWER,             /* 电源指示LED */
    GPIO_LED_NETWORK,           /* 网络状态LED */
    GPIO_LED_RUNNING,           /* 运行状态LED */
    GPIO_LED_COMMUNICATION,     /* 通信状态LED */
    GPIO_LED_FAULT,             /* 故障报警LED */

    /* 系统控制 */
    GPIO_SYSTEM_ENABLE,         /* 系统使能 */
    GPIO_WATCHDOG_FEED,         /* 看门狗喂狗 */

    /* 传感器输入 */
    GPIO_LEVEL_SWITCH_1,        /* 液位开关1 */
    GPIO_LEVEL_SWITCH_2,        /* 液位开关2 */
    GPIO_LEVEL_SWITCH_3,        /* 液位开关3 */
    GPIO_LEVEL_SWITCH_4,        /* 液位开关4 */
    GPIO_EMERGENCY_STOP,        /* 紧急停止 */
    GPIO_SYSTEM_RESET,          /* 系统复位 */
    GPIO_EXTERNAL_FAULT,        /* 外部故障 */
    GPIO_DOOR_LOCK,             /* 门锁检测 */

    /* 备用引脚 */
    GPIO_SPARE_1,               /* 备用1 */
    GPIO_SPARE_2,               /* 备用2 */

    GPIO_PIN_COUNT              /* 引脚总数 */
} gpio_pin_id_t;

/* GPIO组管理 */
typedef struct {
    gpio_handle_t* handles[GPIO_PIN_COUNT]; /* GPIO句柄数组 */
    uint32_t initialized_count;             /* 已初始化计数 */
    SemaphoreHandle_t group_mutex;          /* 组互斥锁 */
} gpio_group_t;

/* 基础GPIO操作接口 */
hal_result_t gpio_hal_init(gpio_handle_t* handle, const gpio_config_t* config);
hal_result_t gpio_hal_deinit(gpio_handle_t* handle);
hal_result_t gpio_hal_write_pin(gpio_handle_t* handle, gpio_level_t level);
hal_result_t gpio_hal_read_pin(gpio_handle_t* handle, gpio_level_t* level);
hal_result_t gpio_hal_toggle_pin(gpio_handle_t* handle);
hal_result_t gpio_hal_configure_interrupt(gpio_handle_t* handle, gpio_interrupt_mode_t mode, hal_callback_t callback);
hal_result_t gpio_hal_enable_interrupt(gpio_handle_t* handle);
hal_result_t gpio_hal_disable_interrupt(gpio_handle_t* handle);

/* 高级GPIO操作接口 */
hal_result_t gpio_hal_write_port(uint32_t port, uint32_t pin_mask, gpio_level_t level);
hal_result_t gpio_hal_read_port(uint32_t port, uint32_t* value);
hal_result_t gpio_hal_set_alternate_function(gpio_handle_t* handle, uint32_t alternate);

/* GPIO组管理接口 */
hal_result_t gpio_group_init(gpio_group_t* group);
hal_result_t gpio_group_deinit(gpio_group_t* group);
hal_result_t gpio_group_add_pin(gpio_group_t* group, gpio_pin_id_t pin_id, gpio_handle_t* handle);
hal_result_t gpio_group_remove_pin(gpio_group_t* group, gpio_pin_id_t pin_id);
hal_result_t gpio_group_write_pins(gpio_group_t* group, uint32_t pin_mask, gpio_level_t level);
hal_result_t gpio_group_read_pins(gpio_group_t* group, uint32_t* pin_states);

/* 供墨系统专用接口 */
hal_result_t gpio_hal_system_init(void);
hal_result_t gpio_hal_system_deinit(void);
hal_result_t gpio_hal_set_heater(uint8_t heater_id, bool enable);
hal_result_t gpio_hal_set_valve(uint8_t valve_id, bool enable);
hal_result_t gpio_hal_set_led(uint8_t led_id, bool enable);
hal_result_t gpio_hal_get_level_switch(uint8_t switch_id, bool* state);
hal_result_t gpio_hal_get_emergency_stop(bool* state);
hal_result_t gpio_hal_emergency_shutdown(void);
hal_result_t gpio_hal_system_enable(bool enable);
hal_result_t gpio_hal_feed_watchdog(void);

/* 诊断和测试接口 */
hal_result_t gpio_hal_self_test(void);
hal_result_t gpio_hal_pin_test(gpio_handle_t* handle);
hal_result_t gpio_hal_get_pin_info(gpio_handle_t* handle, gpio_config_t* info);

/* 回调函数类型 */
typedef void (*gpio_interrupt_callback_t)(gpio_pin_id_t pin_id, gpio_level_t level);

/* 全局回调注册 */
hal_result_t gpio_hal_register_interrupt_callback(gpio_interrupt_callback_t callback);
hal_result_t gpio_hal_unregister_interrupt_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_HAL_H */