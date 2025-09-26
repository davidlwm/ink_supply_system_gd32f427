/**
 * @file app_config.h
 * @brief 应用层配置头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统应用层统一配置文件，包含所有任务和模块的配置参数
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =================================================================== */
/* 系统基础配置 */
/* =================================================================== */
#define APP_SYSTEM_CLOCK_HZ         200000000   /* 系统时钟频率 200MHz */
#define APP_TICK_RATE_HZ            1000        /* FreeRTOS Tick频率 1000Hz */
#define APP_MIN_STACK_SIZE          128         /* 最小栈大小 */
#define APP_HEAP_SIZE               (32 * 1024) /* 堆大小 32KB */

/* =================================================================== */
/* 任务优先级配置 (数值越大优先级越高) */
/* =================================================================== */
#define APP_PRIORITY_IDLE           0   /* 空闲任务 */
#define APP_PRIORITY_LOW            1   /* 低优先级 - 配置管理、统计 */
#define APP_PRIORITY_BACKGROUND     2   /* 后台任务 - HMI、通信 */
#define APP_PRIORITY_NORMAL         3   /* 普通优先级 - 传感器、执行器 */
#define APP_PRIORITY_HIGH           4   /* 高优先级 - 控制算法 */
#define APP_PRIORITY_CRITICAL       5   /* 关键优先级 - 安全监控 */
#define APP_PRIORITY_REALTIME       6   /* 实时任务 - 紧急处理 */

/* =================================================================== */
/* 任务栈大小配置 (单位: 字) */
/* =================================================================== */
#define APP_STACK_SIZE_TINY         256     /* 微小栈 - 简单任务 */
#define APP_STACK_SIZE_SMALL        512     /* 小栈 - 配置管理 */
#define APP_STACK_SIZE_MEDIUM       1024    /* 中等栈 - 传感器、执行器 */
#define APP_STACK_SIZE_LARGE        2048    /* 大栈 - 控制算法 */
#define APP_STACK_SIZE_XLARGE       4096    /* 超大栈 - 复杂算法 */

/* =================================================================== */
/* 任务周期配置 (单位: ms) */
/* =================================================================== */
#define APP_SAFETY_TASK_PERIOD      10      /* 安全监控任务周期 */
#define APP_CONTROL_TASK_PERIOD     20      /* 控制任务周期 */
#define APP_SENSOR_TASK_PERIOD      50      /* 传感器任务周期 */
#define APP_ACTUATOR_TASK_PERIOD    50      /* 执行器任务周期 */
#define APP_HMI_TASK_PERIOD         100     /* HMI任务周期 */
#define APP_COMM_TASK_PERIOD        100     /* 通信任务周期 */
#define APP_CONFIG_TASK_PERIOD      1000    /* 配置管理任务周期 */

/* =================================================================== */
/* 硬件资源配置 */
/* =================================================================== */

/* ADC配置 */
#define APP_ADC_RESOLUTION          12      /* ADC分辨率 12位 */
#define APP_ADC_VREF_MV             3300    /* ADC参考电压 3.3V */
#define APP_ADC_SAMPLE_TIME         15      /* ADC采样时间 15周期 */

/* PWM配置 */
#define APP_PWM_FREQUENCY_HZ        1000    /* PWM频率 1kHz */
#define APP_PWM_RESOLUTION          1000    /* PWM分辨率 1000步 */

/* UART配置 */
#define APP_UART_BAUDRATE           115200  /* UART波特率 */
#define APP_UART_TX_BUFFER_SIZE     512     /* UART发送缓冲区大小 */
#define APP_UART_RX_BUFFER_SIZE     512     /* UART接收缓冲区大小 */

/* SPI配置 */
#define APP_SPI_FREQUENCY_HZ        8000000 /* SPI频率 8MHz */
#define APP_SPI_MODE                0       /* SPI模式0 */

/* =================================================================== */
/* 传感器配置 */
/* =================================================================== */

/* 液位传感器配置 */
#define APP_LEVEL_SENSOR_COUNT      2       /* 液位传感器数量 */
#define APP_LEVEL_TANK_HEIGHT_MM    1000    /* 默认水箱高度 1000mm */
#define APP_LEVEL_FILTER_WINDOW     10      /* 液位滤波窗口 */
#define APP_LEVEL_ALARM_HIGH        900     /* 高位报警 900mm */
#define APP_LEVEL_ALARM_LOW         100     /* 低位报警 100mm */

/* 压力传感器配置 */
#define APP_PRESSURE_SENSOR_COUNT   2       /* 压力传感器数量 */
#define APP_PRESSURE_MAX_KPA        1000    /* 最大压力 1000kPa */
#define APP_PRESSURE_FILTER_WINDOW  8       /* 压力滤波窗口 */
#define APP_PRESSURE_ALARM_HIGH     800     /* 高压报警 800kPa */
#define APP_PRESSURE_ALARM_LOW      50      /* 低压报警 50kPa */

/* 温度传感器配置 */
#define APP_TEMP_SENSOR_COUNT       3       /* 温度传感器数量 */
#define APP_TEMP_MIN_C              -20     /* 最低温度 -20°C */
#define APP_TEMP_MAX_C              150     /* 最高温度 150°C */
#define APP_TEMP_FILTER_WINDOW      5       /* 温度滤波窗口 */
#define APP_TEMP_ALARM_HIGH         120     /* 高温报警 120°C */
#define APP_TEMP_ALARM_LOW          5       /* 低温报警 5°C */

/* =================================================================== */
/* 执行器配置 */
/* =================================================================== */

/* 加热器配置 */
#define APP_HEATER_COUNT            3       /* 加热器数量 */
#define APP_HEATER_MAX_POWER_W      2000    /* 最大功率 2000W */
#define APP_HEATER_RAMP_RATE        50      /* 功率变化率 50W/s */

/* 泵配置 */
#define APP_PUMP_COUNT              2       /* 泵数量 */
#define APP_PUMP_MAX_SPEED_RPM      3000    /* 最大转速 3000RPM */
#define APP_PUMP_MIN_SPEED_RPM      300     /* 最小转速 300RPM */
#define APP_PUMP_RAMP_RATE          100     /* 转速变化率 100RPM/s */

/* 阀门配置 */
#define APP_VALVE_COUNT             4       /* 阀门数量 */
#define APP_VALVE_OPERATE_TIME_MS   2000    /* 阀门动作时间 2秒 */

/* =================================================================== */
/* 控制算法配置 */
/* =================================================================== */

/* PID控制器配置 */
#define APP_PID_OUTPUT_MIN          0.0f    /* PID输出下限 */
#define APP_PID_OUTPUT_MAX          100.0f  /* PID输出上限 */
#define APP_PID_INTEGRAL_LIMIT      50.0f   /* 积分限幅 */
#define APP_PID_DEADBAND            0.5f    /* 死区 */

/* 温度控制PID默认参数 */
#define APP_TEMP_PID_KP             2.0f    /* 比例系数 */
#define APP_TEMP_PID_KI             0.1f    /* 积分系数 */
#define APP_TEMP_PID_KD             0.05f   /* 微分系数 */

/* 压力控制PID默认参数 */
#define APP_PRESSURE_PID_KP         1.5f    /* 比例系数 */
#define APP_PRESSURE_PID_KI         0.08f   /* 积分系数 */
#define APP_PRESSURE_PID_KD         0.02f   /* 微分系数 */

/* =================================================================== */
/* 通信配置 */
/* =================================================================== */

/* Modbus配置 */
#define APP_MODBUS_SLAVE_ADDR       1       /* Modbus从站地址 */
#define APP_MODBUS_BAUDRATE         9600    /* Modbus波特率 */
#define APP_MODBUS_TIMEOUT_MS       1000    /* Modbus超时时间 */

/* 以太网配置 */
#define APP_ETH_DHCP_ENABLE         false   /* DHCP使能 */
#define APP_ETH_IP_ADDR             {192,168,1,100}     /* IP地址 */
#define APP_ETH_NETMASK             {255,255,255,0}     /* 子网掩码 */
#define APP_ETH_GATEWAY             {192,168,1,1}       /* 网关 */
#define APP_ETH_MAC_ADDR            {0x02,0x00,0x00,0x12,0x34,0x56} /* MAC地址 */

/* TCP服务器配置 */
#define APP_TCP_SERVER_PORT         502     /* TCP服务器端口 */
#define APP_TCP_MAX_CONNECTIONS     4       /* 最大连接数 */
#define APP_TCP_TIMEOUT_MS          5000    /* TCP超时时间 */

/* =================================================================== */
/* 安全配置 */
/* =================================================================== */

/* 故障检测配置 */
#define APP_FAULT_CHECK_PERIOD_MS   100     /* 故障检测周期 */
#define APP_FAULT_MAX_COUNT         3       /* 最大故障次数 */
#define APP_FAULT_RESET_TIME_MS     10000   /* 故障复位时间 */

/* 看门狗配置 */
#define APP_WATCHDOG_TIMEOUT_MS     5000    /* 看门狗超时时间 */
#define APP_WATCHDOG_ENABLE         true    /* 看门狗使能 */

/* 急停配置 */
#define APP_EMERGENCY_STOP_TIME_MS  500     /* 急停响应时间 */
#define APP_SAFETY_INTERLOCK_TIME_MS 1000   /* 安全联锁时间 */

/* =================================================================== */
/* 存储配置 */
/* =================================================================== */

/* Flash存储配置 */
#define APP_CONFIG_FLASH_ADDR       0x08040000  /* 配置存储地址 */
#define APP_CONFIG_FLASH_SIZE       0x20000     /* 配置存储大小 128KB */
#define APP_LOG_FLASH_ADDR          0x08060000  /* 日志存储地址 */
#define APP_LOG_FLASH_SIZE          0x20000     /* 日志存储大小 128KB */

/* 配置备份 */
#define APP_CONFIG_BACKUP_COUNT     2       /* 配置备份数量 */
#define APP_CONFIG_CRC_ENABLE       true    /* 配置CRC校验使能 */

/* =================================================================== */
/* 调试配置 */
/* =================================================================== */

/* 调试输出配置 */
#define APP_DEBUG_ENABLE            true    /* 调试使能 */
#define APP_DEBUG_LEVEL             3       /* 调试级别 (0-4) */
#define APP_DEBUG_UART_CHANNEL      0       /* 调试UART通道 */

/* 性能监控配置 */
#define APP_PERFORMANCE_MONITOR     true    /* 性能监控使能 */
#define APP_STACK_MONITOR           true    /* 栈监控使能 */
#define APP_CPU_USAGE_MONITOR       true    /* CPU使用率监控使能 */

/* 统计配置 */
#define APP_STATISTICS_PERIOD_MS    1000    /* 统计周期 */
#define APP_STATISTICS_HISTORY      100     /* 统计历史记录数 */

/* =================================================================== */
/* 版本信息 */
/* =================================================================== */
#define APP_VERSION_MAJOR           4       /* 主版本号 */
#define APP_VERSION_MINOR           0       /* 次版本号 */
#define APP_VERSION_PATCH           0       /* 补丁版本号 */
#define APP_BUILD_NUMBER            1       /* 构建号 */
#define APP_VERSION_STRING          "V4.0.0" /* 版本字符串 */
#define APP_BUILD_DATE              __DATE__ /* 构建日期 */
#define APP_BUILD_TIME              __TIME__ /* 构建时间 */

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */