/**
 * @file    task_scheduler.h
 * @brief   任务调度器头文件
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __TASK_SCHEDULER_H
#define __TASK_SCHEDULER_H

#include "system_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>

// 任务调度结果定义
typedef enum {
    SCHEDULER_SUCCESS = 0,
    SCHEDULER_ERROR_TASK_CREATE,
    SCHEDULER_ERROR_MEMORY,
    SCHEDULER_ERROR_INVALID_PARAMETER
} scheduler_result_t;

// 任务创建和管理函数
scheduler_result_t create_system_tasks(void);
void task_monitor_init(void);
void task_performance_monitor(void);

// 任务堆栈大小和优先级定义 (保持v1兼容)
#define SENSOR_TASK_STACK_SIZE          512
#define ACTUATOR_TASK_STACK_SIZE        512
#define CONTROL_TASK_STACK_SIZE         1024
#define COMM_TASK_STACK_SIZE            2048
#define HMI_TASK_STACK_SIZE            1024
#define SAFETY_TASK_STACK_SIZE          512
#define CONFIG_TASK_STACK_SIZE          1024

#define SENSOR_TASK_PRIORITY            5
#define ACTUATOR_TASK_PRIORITY          4
#define CONTROL_TASK_PRIORITY           6
#define COMM_TASK_PRIORITY              3
#define HMI_TASK_PRIORITY              2
#define SAFETY_TASK_PRIORITY            7  // 最高优先级
#define CONFIG_TASK_PRIORITY            1  // 最低优先级

// 任务周期定义 (ms)
#define CONTROL_TASK_PERIOD_MS          10
#define SENSOR_TASK_PERIOD_MS           20
#define ACTUATOR_TASK_PERIOD_MS         50
#define COMM_TASK_PERIOD_MS             100
#define HMI_TASK_PERIOD_MS              200
#define SAFETY_TASK_PERIOD_MS           5
#define CONFIG_TASK_PERIOD_MS           1000

#endif /* __TASK_SCHEDULER_H */