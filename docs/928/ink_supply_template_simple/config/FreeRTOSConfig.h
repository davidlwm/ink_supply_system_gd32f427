/*
 * FreeRTOS Kernel V10.6.2
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * FreeRTOS Configuration for GD32F427
 * MCU: GD32F427VGT6
 * Core: ARM Cortex-M4F
 * Clock: 200MHz
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *-----------------------------------------------------------*/

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif

/* ========== 基础配置 ========== */

/* 使能抢占式调度器 */
#define configUSE_PREEMPTION              1

/* 使能时间片轮转调度 */
#define configUSE_TIME_SLICING            1

/* 使能空闲任务钩子函数 */
#define configUSE_IDLE_HOOK               0

/* 使能时钟节拍钩子函数 */
#define configUSE_TICK_HOOK               1

/* 使能堆栈溢出检测 (1=方法1, 2=方法2) */
#define configCHECK_FOR_STACK_OVERFLOW    2

/* 使能malloc失败钩子 */
#define configUSE_MALLOC_FAILED_HOOK      1

/* ========== 时钟配置 ========== */

/* CPU主频 (Hz) */
#define configCPU_CLOCK_HZ                (SystemCoreClock)

/* FreeRTOS时钟节拍频率 (Hz) - 1ms一次节拍 */
#define configTICK_RATE_HZ                ((TickType_t)1000)

/* SysTick中断优先级 (最低优先级) */
#define configKERNEL_INTERRUPT_PRIORITY   255

/* 可被FreeRTOS管理的最高中断优先级 */
/* ARM Cortex-M4只使用高4位，所以实际值为0xA0 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  0xA0

/* 最小堆栈大小 (words) */
#define configMINIMAL_STACK_SIZE          ((uint16_t)128)

/* 堆大小 (bytes) - 分配给FreeRTOS的堆内存 */
#define configTOTAL_HEAP_SIZE             ((size_t)(32 * 1024))

/* 任务名称最大长度 */
#define configMAX_TASK_NAME_LEN           (16)

/* 使用16位时钟节拍计数 */
#define configUSE_16_BIT_TICKS            0

/* 使能任务通知功能 */
#define configUSE_TASK_NOTIFICATIONS      1

/* 任务通知数组大小 */
#define configTASK_NOTIFICATION_ARRAY_ENTRIES  1

/* ========== 任务优先级 ========== */

/* 最大任务优先级 */
#define configMAX_PRIORITIES              (20)

/* 空闲任务优先级 (必须为0) */
#define configIDLE_SHOULD_YIELD           1

/* ========== 内存管理 ========== */

/* 支持静态内存分配 */
#define configSUPPORT_STATIC_ALLOCATION   0

/* 支持动态内存分配 */
#define configSUPPORT_DYNAMIC_ALLOCATION  1

/* 应用程序分配内存对齐 */
#define configAPPLICATION_ALLOCATED_HEAP  0

/* ========== 队列和信号量 ========== */

/* 使能队列集 */
#define configUSE_QUEUE_SETS              1

/* 队列注册表大小 (调试用) */
#define configQUEUE_REGISTRY_SIZE         8

/* 使能互斥量 */
#define configUSE_MUTEXES                 1

/* 使能递归互斥量 */
#define configUSE_RECURSIVE_MUTEXES       1

/* 使能计数信号量 */
#define configUSE_COUNTING_SEMAPHORES     1

/* ========== 软件定时器 ========== */

/* 使能软件定时器 */
#define configUSE_TIMERS                  1

/* 软件定时器任务优先级 */
#define configTIMER_TASK_PRIORITY         (configMAX_PRIORITIES - 2)

/* 软件定时器队列长度 */
#define configTIMER_QUEUE_LENGTH          10

/* 软件定时器任务堆栈大小 */
#define configTIMER_TASK_STACK_DEPTH      (configMINIMAL_STACK_SIZE * 2)

/* ========== 协程 (Co-routines) ========== */

/* 使能协程 (一般不使用) */
#define configUSE_CO_ROUTINES             0

/* 协程最大优先级 */
#define configMAX_CO_ROUTINE_PRIORITIES   (2)

/* ========== 调试和跟踪 ========== */

/* 生成运行时统计信息 */
#define configGENERATE_RUN_TIME_STATS     1

/* 使用跟踪钩子宏 */
#define configUSE_TRACE_FACILITY          1

/* 格式化运行时统计信息 */
#define configUSE_STATS_FORMATTING_FUNCTIONS  1

/* 运行时计数器时钟频率 (使用TIM5, 1MHz) */
extern void vConfigureTimerForRunTimeStats(void);
extern uint32_t ulGetRunTimeCounterValue(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  vConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()          ulGetRunTimeCounterValue()

/* ========== 可选功能 ========== */

/* 使能任务标签 */
#define configUSE_APPLICATION_TASK_TAG    0

/* 使能新库线程安全 */
#define configUSE_NEWLIB_REENTRANT        0

/* ========== FreeRTOS+CLI 配置 ========== */

/* 命令行最大输入长度 */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE 512

/* ========== 内存保护 ========== */

/* MPU支持 (GD32F427不使用MPU) */
#define configENABLE_MPU                  0
#define configENABLE_FPU                  1
#define configENABLE_TRUSTZONE            0

/* ========== API函数使能 ========== */

#define INCLUDE_vTaskPrioritySet          1
#define INCLUDE_uxTaskPriorityGet         1
#define INCLUDE_vTaskDelete               1
#define INCLUDE_vTaskCleanUpResources     0
#define INCLUDE_vTaskSuspend              1
#define INCLUDE_vTaskDelayUntil           1
#define INCLUDE_vTaskDelay                1
#define INCLUDE_xTaskGetSchedulerState    1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark  1
#define INCLUDE_xTaskGetIdleTaskHandle    1
#define INCLUDE_eTaskGetState             1
#define INCLUDE_xEventGroupSetBitFromISR  1
#define INCLUDE_xTimerPendFunctionCall    1
#define INCLUDE_xTaskAbortDelay           1
#define INCLUDE_xTaskGetHandle            1
#define INCLUDE_xTaskResumeFromISR        1

/* ========== Cortex-M4特定配置 ========== */

/* 断言宏定义 */
#define configASSERT(x) if((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

/* 中断优先级设置宏 */
#ifdef __NVIC_PRIO_BITS
  #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
  #define configPRIO_BITS         4
#endif

/* 最低中断优先级 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* 可由FreeRTOS管理的最高中断优先级 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  10

/* ========== 中断服务函数映射 ========== */

/* SVC中断 */
#define vPortSVCHandler    SVC_Handler

/* PendSV中断 */
#define xPortPendSVHandler PendSV_Handler

/* SysTick中断 */
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */