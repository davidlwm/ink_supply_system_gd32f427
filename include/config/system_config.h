/**
 * @file    system_config.h
 * @brief   GD32F427供墨系统配置参数
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

#include "gd32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

// 系统版本信息
#define SYSTEM_VERSION_MAJOR        4
#define SYSTEM_VERSION_MINOR        0
#define SYSTEM_VERSION_PATCH        0
#define SYSTEM_BUILD_DATE           "2025-09-27"

// 系统时钟配置
#define SYSTEM_CLOCK_FREQ           200000000U      // 200MHz
#define AHB_CLOCK_FREQ              200000000U      // AHB总线时钟
#define APB1_CLOCK_FREQ             50000000U       // APB1总线时钟
#define APB2_CLOCK_FREQ             100000000U      // APB2总线时钟

// FreeRTOS任务配置
#define TASK_STACK_SIZE_SMALL       256             // 小任务栈大小
#define TASK_STACK_SIZE_MEDIUM      512             // 中等任务栈大小
#define TASK_STACK_SIZE_LARGE       1024            // 大任务栈大小
#define TASK_STACK_SIZE_XLARGE      2048            // 超大任务栈大小

// 任务优先级定义 (基于v4 RTOS设计)
#define TASK_PRIORITY_IDLE          0               // 空闲任务
#define CONFIG_TASK_PRIORITY        1               // 配置任务
#define HMI_TASK_PRIORITY           2               // HMI任务
#define COMM_TASK_PRIORITY          3               // 通信任务
#define ACTUATOR_TASK_PRIORITY      4               // 执行器任务
#define SENSOR_TASK_PRIORITY        5               // 传感器任务
#define CONTROL_TASK_PRIORITY       6               // 控制任务
#define SAFETY_TASK_PRIORITY        7               // 安全任务 (最高)

// 任务栈大小分配
#define SENSOR_TASK_STACK_SIZE      TASK_STACK_SIZE_MEDIUM
#define ACTUATOR_TASK_STACK_SIZE    TASK_STACK_SIZE_MEDIUM
#define CONTROL_TASK_STACK_SIZE     TASK_STACK_SIZE_LARGE
#define COMM_TASK_STACK_SIZE        TASK_STACK_SIZE_XLARGE
#define HMI_TASK_STACK_SIZE         TASK_STACK_SIZE_LARGE
#define SAFETY_TASK_STACK_SIZE      TASK_STACK_SIZE_MEDIUM
#define CONFIG_TASK_STACK_SIZE      TASK_STACK_SIZE_LARGE

// 传感器配置 (基于v1设计)
#define SENSOR_SAMPLE_PERIOD_MS     10              // 传感器采样周期
#define SENSOR_FILTER_ENABLE        1               // 使能数字滤波
#define LIQUID_LEVEL_SENSOR_COUNT   2               // 液位传感器数量
#define PRESSURE_SENSOR_COUNT       2               // 压力传感器数量
#define TEMPERATURE_SENSOR_COUNT    3               // 温度传感器数量

// 执行器配置 (基于v1设计)
#define ACTUATOR_UPDATE_PERIOD_MS   50              // 执行器更新周期
#define HEATER_COUNT                3               // 加热器数量
#define PUMP_COUNT                  2               // 泵数量
#define VALVE_COUNT                 8               // 电磁阀数量
#define LED_COUNT                   5               // LED数量

// ADC配置
#define ADC_SAMPLE_TIME             ADC_SAMPLETIME_480
#define ADC_RESOLUTION_15BIT        1               // 使能15位精度
#define ADC_DMA_BUFFER_SIZE         10              // ADC DMA缓冲区大小

// PWM配置
#define PWM_HEATER_FREQUENCY        100000U         // 100kHz (加热器PWM)
#define PWM_PUMP_FREQUENCY          10000U          // 10kHz (泵调速PWM)
#define PWM_DUTY_CYCLE_MAX          1000U           // 最大占空比 (0.1%精度)

// 通信配置 (基于v1双协议设计)
#define ETHERCAT_CYCLE_TIME_MS      1               // EtherCAT循环时间
#define TCP_SERVER_PORT             502             // TCP服务器端口
#define TCP_MAX_CONNECTIONS         3               // 最大TCP连接数
#define NETWORK_HEARTBEAT_MS        1000            // 网络心跳间隔

// 安全配置
#define SAFETY_CHECK_PERIOD_MS      100             // 安全检查周期
#define WATCHDOG_TIMEOUT_MS         5000            // 看门狗超时时间
#define EMERGENCY_SHUTDOWN_ENABLE   1               // 使能紧急停机

// 错误代码定义
typedef enum {
    ERROR_NONE = 0,
    ERROR_BOARD_INIT_FAILED,
    ERROR_SYSTEM_INIT_FAILED,
    ERROR_TASK_CREATE_FAILED,
    ERROR_SCHEDULER_FAILED,
    ERROR_SENSOR_FAULT,
    ERROR_ACTUATOR_FAULT,
    ERROR_COMMUNICATION_FAULT,
    ERROR_SAFETY_FAULT
} system_error_t;

// 初始化状态定义
typedef enum {
    BOARD_INIT_SUCCESS = 0,
    BOARD_INIT_FAILED,
    SYSTEM_INIT_SUCCESS,
    SYSTEM_INIT_FAILED
} init_status_t;

// GPIO引脚定义 (基于板级设计)
// 执行器控制引脚
#define GPIO_HEATER_1                   15              // 加热器1控制
#define GPIO_HEATER_2                   16              // 加热器2控制
#define GPIO_HEATER_3                   17              // 加热器3控制

#define GPIO_VALVE_1                    8               // 阀门1控制
#define GPIO_VALVE_2                    9               // 阀门2控制
#define GPIO_VALVE_3                    10              // 阀门3控制
#define GPIO_VALVE_4                    11              // 阀门4控制
#define GPIO_VALVE_5                    12              // 阀门5控制
#define GPIO_VALVE_6                    13              // 阀门6控制
#define GPIO_VALVE_7                    14              // 阀门7控制
#define GPIO_VALVE_8                    15              // 阀门8控制

#define GPIO_PUMP_1_EN                  18              // 泵1使能
#define GPIO_PUMP_1_DIR                 19              // 泵1方向
#define GPIO_PUMP_2_EN                  20              // 泵2使能
#define GPIO_PUMP_2_DIR                 21              // 泵2方向

// LED引脚定义
#define GPIO_LED_POWER                  0               // 电源LED
#define GPIO_LED_RUN                    1               // 运行LED
#define GPIO_LED_ERROR                  2               // 错误LED
#define GPIO_LED_COMM                   3               // 通信LED
#define GPIO_LED_ALARM                  4               // 报警LED

// 系统状态引脚
#define GPIO_EMERGENCY_STOP             8               // 紧急停止
#define GPIO_SYSTEM_RESET               9               // 系统复位
#define GPIO_SYSTEM_ENABLE              0               // 系统使能

// 内存配置
#define HEAP_SIZE                       32768           // 堆大小 (32KB)
#define STACK_SIZE                      4096            // 主栈大小 (4KB)

// 调试配置
#define DEBUG_ENABLE                    1               // 使能调试
#define DEBUG_USART                     USART0          // 调试串口
#define DEBUG_BAUDRATE                  115200          // 调试波特率

// 延时函数宏定义
#define delay_1ms(x)                    vTaskDelay(pdMS_TO_TICKS(x))
#define delay_us(x)                     delay_1ms(1)    // 简化实现

// 工具宏定义
#define ARRAY_SIZE(x)                   (sizeof(x) / sizeof(x[0]))
#define MAX(a, b)                       ((a) > (b) ? (a) : (b))
#define MIN(a, b)                       ((a) < (b) ? (a) : (b))
#define CLAMP(x, min, max)              MAX(min, MIN(x, max))

// 函数声明
void system_clock_config(void);
void systick_config(void);
void watchdog_early_init(void);
void error_handler(system_error_t error);

#endif /* __SYSTEM_CONFIG_H */