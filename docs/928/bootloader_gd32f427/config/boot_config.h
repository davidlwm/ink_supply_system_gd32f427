/**
 ******************************************************************************
 * @file    boot_config.h
 * @brief   Bootloader配置数据结构定义
 * @version V1.0
 * @date    2025-09-30
 ******************************************************************************
 */

#ifndef __BOOT_CONFIG_H
#define __BOOT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========== 魔术字和版本 ========== */
#define BOOT_CONFIG_MAGIC           0x42544C4FU  /* "BTLO" */
#define BOOT_CONFIG_VERSION         0x0100U      /* v1.0 */

/* ========== 启动标志枚举 ========== */
typedef enum {
    BOOT_FLAG_NONE = 0,                 /* 未初始化 */
    BOOT_FLAG_NORMAL_RUN,               /* 正常运行 */
    BOOT_FLAG_UPDATE_PENDING,           /* 待升级（Bank B有新固件） */
    BOOT_FLAG_UPDATE_IN_PROGRESS,       /* 升级中 */
    BOOT_FLAG_UPDATE_SUCCESS,           /* 升级成功 */
    BOOT_FLAG_UPDATE_FAILED,            /* 升级失败 */
    BOOT_FLAG_EMERGENCY_MODE,           /* 应急模式 */
    BOOT_FLAG_FACTORY_RESET             /* 恢复出厂设置 */
} boot_flag_t;

/* ========== 固件信息结构 ========== */
typedef struct {
    uint32_t version;                   /* 版本号 (主8位.次8位.修订16位) */
    uint32_t size;                      /* 固件大小（字节） */
    uint32_t crc32;                     /* CRC32校验值 */
    uint32_t timestamp;                 /* 编译时间戳（Unix时间） */
    char description[32];               /* 版本描述字符串 */
} firmware_info_t;

/* ========== 启动配置结构体 ========== */
typedef struct {
    /* 魔术字和版本 */
    uint32_t magic;                     /* 0x42544C4F "BTLO" */
    uint16_t config_version;            /* 配置版本号 */
    uint16_t reserved1;                 /* 保留对齐 */

    /* 启动标志 */
    boot_flag_t boot_flag;              /* 启动状态 */
    uint32_t active_bank;               /* 当前活跃Bank (0=A, 1=B) */

    /* Bank A 固件信息 */
    firmware_info_t bank_a_info;

    /* Bank B 固件信息 */
    firmware_info_t bank_b_info;

    /* 升级统计 */
    uint32_t update_count;              /* 升级次数 */
    uint32_t boot_count;                /* 启动次数 */
    uint32_t last_update_time;          /* 最后升级时间 */

    /* 回退信息 */
    uint32_t rollback_count;            /* 回退次数 */
    uint32_t max_rollback;              /* 最大回退次数 (默认3) */
    uint32_t consecutive_boot_fail;     /* 连续启动失败次数 */

    /* 安全特性 */
    uint32_t secure_boot_enable;        /* 安全启动使能 (0=关闭, 1=开启) */
    uint32_t min_fw_version;            /* 最小固件版本（防回滚） */

    /* 应急模式配置 */
    uint32_t emergency_mode_timeout;    /* 应急模式超时时间（秒） */
    uint32_t emergency_key_gpio;        /* 应急模式按键GPIO */

    /* 保留扩展 */
    uint8_t reserved[64];

    /* 校验码（必须放在最后） */
    uint32_t config_crc32;              /* 配置结构体CRC32 */
} boot_config_t;

/* ========== 版本号操作宏 ========== */
/* 版本号格式: [主版本 8位][次版本 8位][修订版本 16位] */
#define MAKE_VERSION(major, minor, patch)   \
    (((uint32_t)(major) << 24) | ((uint32_t)(minor) << 16) | ((uint32_t)(patch)))

#define GET_VERSION_MAJOR(version)          (((version) >> 24) & 0xFF)
#define GET_VERSION_MINOR(version)          (((version) >> 16) & 0xFF)
#define GET_VERSION_PATCH(version)          ((version) & 0xFFFF)

/* ========== 配置默认值 ========== */
#define DEFAULT_MAX_ROLLBACK            3       /* 默认最大回退次数 */
#define DEFAULT_EMERGENCY_TIMEOUT       300     /* 应急模式超时5分钟 */
#define MAX_BOOT_FAIL_COUNT             3       /* 最大连续启动失败次数 */

/* ========== 函数声明 ========== */
/**
 * @brief  初始化配置为默认值
 * @param  config: 配置结构体指针
 * @retval None
 */
void boot_config_reset(boot_config_t *config);

/**
 * @brief  从Flash加载配置
 * @param  config: 配置结构体指针
 * @retval true: 成功, false: 失败
 */
bool boot_config_load(boot_config_t *config);

/**
 * @brief  保存配置到Flash
 * @param  config: 配置结构体指针
 * @retval true: 成功, false: 失败
 */
bool boot_config_save(const boot_config_t *config);

/**
 * @brief  校验配置完整性
 * @param  config: 配置结构体指针
 * @retval true: 有效, false: 无效
 */
bool boot_config_verify(const boot_config_t *config);

/**
 * @brief  打印配置信息（调试用）
 * @param  config: 配置结构体指针
 * @retval None
 */
void boot_config_print(const boot_config_t *config);

/**
 * @brief  备份配置到备份扇区
 * @param  config: 配置结构体指针
 * @retval true: 成功, false: 失败
 */
bool boot_config_backup(const boot_config_t *config);

/**
 * @brief  从备份扇区恢复配置
 * @param  config: 配置结构体指针
 * @retval true: 成功, false: 失败
 */
bool boot_config_restore(boot_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_CONFIG_H */