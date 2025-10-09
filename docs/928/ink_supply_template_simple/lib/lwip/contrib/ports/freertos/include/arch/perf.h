/**
 * @file perf.h
 * @brief lwIP performance measurement definitions for ARM Cortex-M4 + FreeRTOS
 *
 * This file is part of the lwIP TCP/IP stack for GD32F427 with FreeRTOS.
 * Performance measurement hooks for profiling and optimization.
 */

#ifndef LWIP_ARCH_PERF_H
#define LWIP_ARCH_PERF_H

/* FreeRTOS includes for timing functions */
#include "FreeRTOS.h"
#include "task.h"

/* Performance measurement support */
#ifdef LWIP_PERF

/* 获取高精度时间戳 */
#define PERF_START    /* null definition */
#define PERF_STOP(x)  /* null definition */

/* 如果需要详细的性能测量，可以使用以下定义 */
#if 0
/* 使用FreeRTOS的tick count进行简单测量 */
extern volatile uint32_t lwip_perf_start;
extern volatile uint32_t lwip_perf_end;

#define PERF_START  do { lwip_perf_start = xTaskGetTickCount(); } while(0)
#define PERF_STOP(x) do { \
    lwip_perf_end = xTaskGetTickCount(); \
    printf("PERF %s: %u ticks\n", x, (unsigned)(lwip_perf_end - lwip_perf_start)); \
} while(0)

/* 或者使用DWT cycle counter进行高精度测量 (如果可用) */
#ifdef ARM_MATH_CM4
#include "core_cm4.h"

/* 启用DWT cycle counter */
static inline void perf_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

static inline uint32_t perf_get_cycles(void)
{
    return DWT->CYCCNT;
}

#undef PERF_START
#undef PERF_STOP

extern volatile uint32_t lwip_perf_cycles_start;
extern volatile uint32_t lwip_perf_cycles_end;

#define PERF_START  do { lwip_perf_cycles_start = perf_get_cycles(); } while(0)
#define PERF_STOP(x) do { \
    lwip_perf_cycles_end = perf_get_cycles(); \
    printf("PERF %s: %u cycles\n", x, (unsigned)(lwip_perf_cycles_end - lwip_perf_cycles_start)); \
} while(0)

#endif /* ARM_MATH_CM4 */
#endif /* detailed perf measurement */

#else /* LWIP_PERF */

/* Performance measurement disabled */
#define PERF_START  /* null definition */
#define PERF_STOP(x)  /* null definition */

#endif /* LWIP_PERF */

/* 内存和CPU使用情况监控宏 */
#ifdef LWIP_STATS_DISPLAY

/* 显示内存使用统计 */
#define PERF_SHOW_MEM_STATS() do { \
    printf("Free heap: %u bytes\n", (unsigned)xPortGetFreeHeapSize()); \
} while(0)

/* 显示任务统计 */
#define PERF_SHOW_TASK_STATS() do { \
    printf("Current task: %s\n", pcTaskGetName(NULL)); \
} while(0)

#else

#define PERF_SHOW_MEM_STATS() /* null definition */
#define PERF_SHOW_TASK_STATS() /* null definition */

#endif /* LWIP_STATS_DISPLAY */

#endif /* LWIP_ARCH_PERF_H */