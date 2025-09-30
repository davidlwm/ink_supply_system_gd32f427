/**
 * @file cc.h
 * @brief lwIP compiler/platform specific definitions for ARM Cortex-M4 + FreeRTOS
 *
 * This file is part of the lwIP TCP/IP stack for GD32F427 with FreeRTOS.
 * Based on lwIP contrib examples for ARM Cortex-M.
 */

#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

/* Include system headers */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>  /* 添加limits.h支持INT_MAX等常量 */

/* FreeRTOS includes for timing */
#include "FreeRTOS.h"
#include "task.h"

/* ARM Cortex-M4 specific */
#define LWIP_NO_STDINT_H 0
#define LWIP_NO_STDDEF_H 0
#define LWIP_NO_INTTYPES_H 1
#define LWIP_NO_LIMITS_H 0  /* 允许使用limits.h */

/* 数据类型定义 */
typedef uint8_t    u8_t;
typedef int8_t     s8_t;
typedef uint16_t   u16_t;
typedef int16_t    s16_t;
typedef uint32_t   u32_t;
typedef int32_t    s32_t;

typedef uintptr_t  mem_ptr_t;

/* 为Keil编译器定义缺失的类型和常量 */
#ifdef __CC_ARM  /* Keil MDK-ARM编译器 */
/* 定义ssize_t类型 - 与lwIP arch.h保持一致 */
#ifndef _SSIZE_T_DEFINED
typedef int ssize_t;  /* 使用int与lwIP默认保持一致 */
#define _SSIZE_T_DEFINED
#endif

/* 定义可能缺失的常量 */
#ifndef SSIZE_MAX
#define SSIZE_MAX INT_MAX  /* 与lwIP arch.h保持一致 */
#endif
#endif

/* 通用的ssize_t定义（如果还没有定义） */
#ifndef _SSIZE_T_DEFINED
typedef int ssize_t;  /* 与lwIP默认保持一致 */
#define _SSIZE_T_DEFINED
#endif

/* 确保SSIZE_MAX被定义 */
#ifndef SSIZE_MAX
#define SSIZE_MAX INT_MAX  /* 与lwIP arch.h保持一致 */
#endif

/* 时间相关结构体定义 (替代sys/time.h) */
#ifndef LWIP_TIMEVAL_PRIVATE
#define LWIP_TIMEVAL_PRIVATE 0
#endif

#if !LWIP_TIMEVAL_PRIVATE
struct timeval {
  long    tv_sec;         /* seconds */
  long    tv_usec;        /* and microseconds */
};
#endif

/* 编译器相关的属性定义 */
#if defined(__CC_ARM) /* Keil MDK-ARM */
#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_BEGIN __packed
#define PACK_STRUCT_END
#elif defined(__GNUC__) && defined(__arm__)
#define PACK_STRUCT_FIELD(x) x __attribute__((packed))
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END
#elif defined(__ICCARM__) /* IAR */
#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_BEGIN __packed
#define PACK_STRUCT_END
#else
#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END
#endif

/* 字节序定义 (ARM Cortex-M4 为小端序) */
#define BYTE_ORDER LITTLE_ENDIAN

/* 编译器优化相关 */
#define LWIP_PLATFORM_DIAG(x) do {printf x;} while(0)
#define LWIP_PLATFORM_ASSERT(x) do {printf("Assertion \"%s\" failed at line %d in %s\n", \
                                            x, __LINE__, __FILE__); while(1);} while(0)

/* 性能优化相关定义 */
#ifdef __GNUC__
#define LWIP_NORETURN __attribute__((noreturn))
#elif defined(__CC_ARM)
#define LWIP_NORETURN __declspec(noreturn)
#else
#define LWIP_NORETURN
#endif

/* 内存对齐 */
#define MEM_ALIGNMENT 4

/* 随机数生成器 (简单实现，生产环境应使用硬件随机数) */
#ifndef LWIP_RAND
#define LWIP_RAND() ((u32_t)rand())
#endif

/* 校验和计算优化 (ARM Cortex-M4支持) */
/* 可以启用硬件校验和计算以提高性能 */
#define LWIP_CHKSUM_ALGORITHM 2

/* 错误处理 */
#define LWIP_PROVIDE_ERRNO 1

/* 定义errno变量 */
#ifndef errno
extern int errno;
#endif

/* 时间相关函数 */
/* sys_now() 在 sys_arch.c 中实现，返回毫秒计数 */

/* 网络接口相关 */
#define LWIP_NETIF_HOSTNAME 1

/* 调试相关 */
#ifdef LWIP_DEBUG
#define U16_F "u"
#define S16_F "d"
#define X16_F "x"
#define U32_F "u"
#define S32_F "d"
#define X32_F "x"
#define SZT_F "u"
#endif

/* ARM Cortex-M4 特定优化 */
#ifdef __arm__
/* 使用ARM Cortex-M4的高效指令 */
#define LWIP_PLATFORM_HTONS(x) __builtin_bswap16(x)
#define LWIP_PLATFORM_HTONL(x) __builtin_bswap32(x)
#endif

/* 编译器特定的警告抑制 */
#ifdef __CC_ARM
#pragma diag_suppress 1295  /* Deprecated declaration */
#endif

#endif /* LWIP_ARCH_CC_H */