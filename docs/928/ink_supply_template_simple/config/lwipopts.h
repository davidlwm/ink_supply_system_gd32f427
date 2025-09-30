/**
 * @file lwipopts.h
 * @brief lwIP配置文件 for GD32F427
 * @version 2.2.0
 *
 * lwIP Configuration for GD32F427 + FreeRTOS
 * MCU: GD32F427VGT6 (1MB Flash, 192KB SRAM)
 * CPU: ARM Cortex-M4F @ 200MHz
 * RTOS: FreeRTOS V10.6.2
 */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ========== 平台配置 ========== */

/* 不使用标准库malloc */
#define NO_SYS                      0

/* 使用lwIP自带的内存管理 */
#define MEM_LIBC_MALLOC             0
#define MEMP_MEM_MALLOC             0

/* lwIP版本 */
#define LWIP_VERSION_MAJOR          2
#define LWIP_VERSION_MINOR          2
#define LWIP_VERSION_REVISION       0

/* ========== 内存配置 ========== */

/* 堆内存大小 (bytes) */
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    (16 * 1024)  // 16KB

/* 内存池数量配置 */
#define MEMP_NUM_PBUF               16
#define MEMP_NUM_RAW_PCB            4
#define MEMP_NUM_UDP_PCB            6
#define MEMP_NUM_TCP_PCB            10
#define MEMP_NUM_TCP_PCB_LISTEN     8
#define MEMP_NUM_TCP_SEG            16
#define MEMP_NUM_REASSDATA          5
#define MEMP_NUM_FRAG_PBUF          15
#define MEMP_NUM_ARP_QUEUE          10
#define MEMP_NUM_IGMP_GROUP         8
#define MEMP_NUM_SYS_TIMEOUT        10
#define MEMP_NUM_NETBUF             4
#define MEMP_NUM_NETCONN            8
#define MEMP_NUM_TCPIP_MSG_API      16
#define MEMP_NUM_TCPIP_MSG_INPKT    16

/* PBUF配置 */
#define PBUF_POOL_SIZE              16
#define PBUF_POOL_BUFSIZE           1536  // 以太网MTU 1500 + 预留

/* ========== ARP配置 ========== */

#define LWIP_ARP                    1
#define ARP_TABLE_SIZE              10
#define ARP_QUEUEING                1
#define ETHARP_SUPPORT_STATIC_ENTRIES  1

/* ========== IP配置 ========== */

#define LWIP_IPV4                   1
#define LWIP_IPV6                   0

/* IP转发 */
#define IP_FORWARD                  0

/* IP分片重组 */
#define IP_REASSEMBLY               1
#define IP_FRAG                     1
#define IP_REASS_MAXAGE             15
#define IP_REASS_MAX_PBUFS          10
#define IP_FRAG_USES_STATIC_BUF     0
#define IP_FRAG_MAX_MTU             1500

/* IP选项 */
#define IP_OPTIONS_ALLOWED          1

/* ========== ICMP配置 ========== */

#define LWIP_ICMP                   1
#define LWIP_BROADCAST_PING         1
#define LWIP_MULTICAST_PING         1

/* ========== DHCP配置 ========== */

#define LWIP_DHCP                   1
#define LWIP_DHCP_CHECK_LINK_UP     1
#define LWIP_DHCP_DOES_ACD_CHECK    1  /* 启用DHCP地址冲突检测 */

/* ========== ACD (Address Conflict Detection) 配置 ========== */

#define LWIP_ACD                    1  /* 启用地址冲突检测模块 */
#define ACD_TMR_INTERVAL            100  /* ACD定时器间隔(ms) */

/* 强制启用ACD相关功能 */
#ifdef LWIP_IPV4
#if LWIP_IPV4
#define LWIP_ACD_ENABLED            1
#endif
#endif

/* ========== UDP配置 ========== */

#define LWIP_UDP                    1
#define LWIP_UDPLITE                0
#define UDP_TTL                     255

/* ========== TCP配置 ========== */

#define LWIP_TCP                    1
#define TCP_TTL                     255
#define TCP_QUEUE_OOSEQ             1
#define TCP_MSS                     1460
#define TCP_SND_BUF                 (4 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define TCP_WND                     (4 * TCP_MSS)
#define TCP_MAXRTX                  12
#define TCP_SYNMAXRTX               6

/* TCP性能优化 */
#define LWIP_TCP_SACK_OUT           0
#define LWIP_TCP_MAX_SACK_NUM       4
#define LWIP_TCP_TIMESTAMPS         0
#define TCP_LISTEN_BACKLOG          1

/* ========== NETIF配置 ========== */

#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_REMOVE_CALLBACK  1
#define LWIP_NETIF_HWADDRHINT       0
#define LWIP_NETIF_TX_SINGLE_PBUF   1

/* ========== Socket API配置 ========== */

#define LWIP_SOCKET                 1
#define LWIP_COMPAT_SOCKETS         1
#define LWIP_POSIX_SOCKETS_IO_NAMES 0
#define LWIP_SOCKET_OFFSET          0
#define LWIP_TCP_KEEPALIVE          1
#define LWIP_SO_SNDTIMEO            1
#define LWIP_SO_RCVTIMEO            1
#define LWIP_SO_RCVBUF              1
#define SO_REUSE                    1
#define LWIP_SO_LINGER              0

/* ========== NETCONN API配置 ========== */

#define LWIP_NETCONN                1
#define LWIP_TCPIP_TIMEOUT          1

/* ========== 统计和调试 ========== */

#define LWIP_STATS                  1
#define LWIP_STATS_DISPLAY          1

#if LWIP_STATS
#define LINK_STATS                  1
#define ETHARP_STATS                1
#define IP_STATS                    1
#define IPFRAG_STATS                1
#define ICMP_STATS                  1
#define UDP_STATS                   1
#define TCP_STATS                   1
#define MEM_STATS                   1
#define MEMP_STATS                  1
#define SYS_STATS                   1
#endif

/* ========== 调试配置 ========== */

#define LWIP_DEBUG                  0

#if LWIP_DEBUG
#define LWIP_DBG_MIN_LEVEL          LWIP_DBG_LEVEL_ALL
#define LWIP_DBG_TYPES_ON           LWIP_DBG_ON

#define ETHARP_DEBUG                LWIP_DBG_OFF
#define NETIF_DEBUG                 LWIP_DBG_OFF
#define PBUF_DEBUG                  LWIP_DBG_OFF
#define API_LIB_DEBUG               LWIP_DBG_OFF
#define API_MSG_DEBUG               LWIP_DBG_OFF
#define SOCKETS_DEBUG               LWIP_DBG_OFF
#define ICMP_DEBUG                  LWIP_DBG_OFF
#define INET_DEBUG                  LWIP_DBG_OFF
#define IP_DEBUG                    LWIP_DBG_OFF
#define IP_REASS_DEBUG              LWIP_DBG_OFF
#define RAW_DEBUG                   LWIP_DBG_OFF
#define MEM_DEBUG                   LWIP_DBG_OFF
#define MEMP_DEBUG                  LWIP_DBG_OFF
#define SYS_DEBUG                   LWIP_DBG_OFF
#define TCP_DEBUG                   LWIP_DBG_OFF
#define TCP_INPUT_DEBUG             LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG            LWIP_DBG_OFF
#define TCP_RTO_DEBUG               LWIP_DBG_OFF
#define TCP_CWND_DEBUG              LWIP_DBG_OFF
#define TCP_WND_DEBUG               LWIP_DBG_OFF
#define TCP_FR_DEBUG                LWIP_DBG_OFF
#define TCP_QLEN_DEBUG              LWIP_DBG_OFF
#define TCP_RST_DEBUG               LWIP_DBG_OFF
#define UDP_DEBUG                   LWIP_DBG_OFF
#define TCPIP_DEBUG                 LWIP_DBG_OFF
#define PPP_DEBUG                   LWIP_DBG_OFF
#define SLIP_DEBUG                  LWIP_DBG_OFF
#define DHCP_DEBUG                  LWIP_DBG_OFF
#endif

/* ========== 校验和配置 ========== */

/* 硬件校验和计算 (如果MAC支持) */
#define CHECKSUM_GEN_IP             0
#define CHECKSUM_GEN_UDP            0
#define CHECKSUM_GEN_TCP            0
#define CHECKSUM_GEN_ICMP           0
#define CHECKSUM_CHECK_IP           0
#define CHECKSUM_CHECK_UDP          0
#define CHECKSUM_CHECK_TCP          0
#define CHECKSUM_CHECK_ICMP         0

/* 嵌入式系统特定配置 */
#define LWIP_NO_UNISTD_H            1  /* 嵌入式系统没有unistd.h */

/* ========== 线程配置 ========== */

/* TCPIP线程优先级 */
#define TCPIP_THREAD_NAME           "tcpip"
#define TCPIP_THREAD_STACKSIZE      1024
#define TCPIP_THREAD_PRIO           (configMAX_PRIORITIES - 2)

/* SLIPIF线程优先级 */
#define SLIPIF_THREAD_NAME          "slipif"
#define SLIPIF_THREAD_STACKSIZE     512
#define SLIPIF_THREAD_PRIO          (configMAX_PRIORITIES - 3)

/* 默认线程优先级 */
#define DEFAULT_THREAD_NAME         "lwip"
#define DEFAULT_THREAD_STACKSIZE    512
#define DEFAULT_THREAD_PRIO         (configMAX_PRIORITIES - 4)

/* ========== 邮箱配置 ========== */

#define TCPIP_MBOX_SIZE             16
#define DEFAULT_RAW_RECVMBOX_SIZE   4
#define DEFAULT_UDP_RECVMBOX_SIZE   8
#define DEFAULT_TCP_RECVMBOX_SIZE   16
#define DEFAULT_ACCEPTMBOX_SIZE     8

/* ========== 协议栈选项 ========== */

/* RAW API */
#define LWIP_RAW                    1

/* DNS */
#define LWIP_DNS                    1
#define DNS_TABLE_SIZE              4
#define DNS_MAX_NAME_LENGTH         256

/* IGMP */
#define LWIP_IGMP                   1

/* SNMP */
#define LWIP_SNMP                   0

/* PPP */
#define PPP_SUPPORT                 0

/* ========== FreeRTOS集成 ========== */

/* 使用FreeRTOS的内存分配 */
#define LWIP_COMPAT_MUTEX           1
#define LWIP_COMPAT_MUTEX_ALLOWED   1

/* 系统时钟 - 由sys_arch.c实现 */
/* sys_now() 函数在 sys_arch.c 中实现 */
#define LWIP_FREERTOS_SYS_NOW_FROM_FREERTOS  1

/* 临界区保护 */
#define SYS_LIGHTWEIGHT_PROT        1

/* ========== 性能优化 ========== */

/* 使用DMA缓冲区对齐 */
#define ETH_RX_BUF_SIZE             1536
#define ETH_TX_BUF_SIZE             1536

/* 接收缓冲区数量 */
#define ETH_RXBUFNB                 4

/* 发送缓冲区数量 */
#define ETH_TXBUFNB                 4

/* ========== 应用层协议 ========== */

/* HTTPD */
#define LWIP_HTTPD                  0

/* SNTP */
#define LWIP_SNTP                   0

/* MQTT */
#define LWIP_MQTT                   0

/* ========== 其他选项 ========== */

/* 允许从中断上下文发送数据 */
#define LWIP_NETIF_TX_SINGLE_PBUF   1

/* 允许在回调中分配内存 */
#define LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT  1

/* 默认网络接口 */
#define LWIP_SINGLE_NETIF           1

#endif /* __LWIPOPTS_H__ */