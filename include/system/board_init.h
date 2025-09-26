/**
 * @file    board_init.h
 * @brief   板级初始化头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __BOARD_INIT_H
#define __BOARD_INIT_H

#include "system_config.h"
#include <stdint.h>
#include <stdbool.h>

// 初始化结果定义
typedef enum {
    BOARD_INIT_SUCCESS = 0,
    BOARD_INIT_ERROR_CLOCK,
    BOARD_INIT_ERROR_GPIO,
    BOARD_INIT_ERROR_PERIPHERAL,
    BOARD_INIT_ERROR_MEMORY
} board_init_result_t;

// 板级初始化函数
board_init_result_t board_init(void);

// 硬件模块初始化函数
void system_clock_config(void);
void systick_config(void);
void watchdog_early_init(void);
void gpio_init_all(void);
void peripheral_init_all(void);

#endif /* __BOARD_INIT_H */