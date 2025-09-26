/**
 * @file adc_hal.h
 * @brief ADC硬件抽象层头文件 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 供墨系统ADC硬件抽象层，基于GD32F4xx HAL库
 */

#ifndef ADC_HAL_H
#define ADC_HAL_H

#include "hal/hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADC分辨率 */
typedef enum {
    ADC_RESOLUTION_6BIT = 0,    /* 6位分辨率 */
    ADC_RESOLUTION_8BIT = 1,    /* 8位分辨率 */
    ADC_RESOLUTION_10BIT = 2,   /* 10位分辨率 */
    ADC_RESOLUTION_12BIT = 3    /* 12位分辨率 */
} adc_resolution_t;

/* ADC触发源 */
typedef enum {
    ADC_TRIGGER_SOFTWARE = 0,   /* 软件触发 */
    ADC_TRIGGER_TIMER = 1,      /* 定时器触发 */
    ADC_TRIGGER_EXTERNAL = 2    /* 外部触发 */
} adc_trigger_source_t;

/* ADC采样时间 */
typedef enum {
    ADC_SAMPLETIME_3CYCLES = 0,     /* 3个时钟周期 */
    ADC_SAMPLETIME_15CYCLES = 1,    /* 15个时钟周期 */
    ADC_SAMPLETIME_28CYCLES = 2,    /* 28个时钟周期 */
    ADC_SAMPLETIME_56CYCLES = 3,    /* 56个时钟周期 */
    ADC_SAMPLETIME_84CYCLES = 4,    /* 84个时钟周期 */
    ADC_SAMPLETIME_112CYCLES = 5,   /* 112个时钟周期 */
    ADC_SAMPLETIME_144CYCLES = 6,   /* 144个时钟周期 */
    ADC_SAMPLETIME_480CYCLES = 7    /* 480个时钟周期 */
} adc_sample_time_t;

/* ADC转换模式 */
typedef enum {
    ADC_MODE_SINGLE = 0,        /* 单次转换 */
    ADC_MODE_CONTINUOUS = 1,    /* 连续转换 */
    ADC_MODE_SCAN = 2,          /* 扫描模式 */
    ADC_MODE_DISCONTINUOUS = 3  /* 间断模式 */
} adc_conversion_mode_t;

/* ADC数据对齐 */
typedef enum {
    ADC_ALIGN_RIGHT = 0,        /* 右对齐 */
    ADC_ALIGN_LEFT = 1          /* 左对齐 */
} adc_data_align_t;

/* ADC通道配置 */
typedef struct {
    uint8_t channel;            /* 通道号 */
    adc_sample_time_t sample_time; /* 采样时间 */
    uint8_t rank;               /* 转换序列中的位置 */
    bool differential;          /* 差分模式 */
} adc_channel_config_t;

/* ADC配置 */
typedef struct {
    hal_config_base_t base;         /* 基础配置 */
    uint32_t instance;              /* ADC实例 */
    adc_resolution_t resolution;    /* 分辨率 */
    adc_data_align_t data_align;    /* 数据对齐 */
    adc_conversion_mode_t conv_mode; /* 转换模式 */
    adc_trigger_source_t trigger_source; /* 触发源 */
    bool continuous_conv;           /* 连续转换使能 */
    bool scan_mode;                 /* 扫描模式使能 */
    bool dma_enable;                /* DMA使能 */
    uint8_t nb_conversions;         /* 转换数量 */
    adc_channel_config_t* channels; /* 通道配置数组 */
    float reference_voltage;        /* 参考电压 */
    hal_callback_t conversion_complete_callback; /* 转换完成回调 */
    hal_callback_t error_callback;  /* 错误回调 */
} adc_config_t;

/* ADC句柄 */
typedef struct {
    hal_handle_base_t base;     /* 基础句柄 */
    adc_config_t* config;       /* ADC配置 */
    uint16_t* conversion_buffer; /* 转换缓冲区 */
    uint32_t buffer_size;       /* 缓冲区大小 */
    volatile bool conversion_complete; /* 转换完成标志 */
    uint32_t conversion_count;  /* 转换计数 */
} adc_handle_t;

/* 供墨系统ADC通道定义 */
typedef enum {
    /* 液位传感器 */
    ADC_CHANNEL_LIQUID_LEVEL_1 = 0, /* 液位传感器1 (FRD-8061) */
    ADC_CHANNEL_LIQUID_LEVEL_2,     /* 液位传感器2 (FRD-8061) */
    ADC_CHANNEL_LIQUID_LEVEL_3,     /* 液位传感器3 (FRD-8061) */
    ADC_CHANNEL_LIQUID_LEVEL_4,     /* 液位传感器4 (FRD-8061) */

    /* 压力传感器 */
    ADC_CHANNEL_PRESSURE_1,         /* 压力传感器1 (HP10MY) */
    ADC_CHANNEL_PRESSURE_2,         /* 压力传感器2 (HP10MY) */
    ADC_CHANNEL_PRESSURE_3,         /* 压力传感器3 (HP10MY) */
    ADC_CHANNEL_PRESSURE_4,         /* 压力传感器4 (HP10MY) */

    /* 温度传感器 */
    ADC_CHANNEL_TEMP_1_SIGNAL,      /* 温度传感器1信号 (FTT518) */
    ADC_CHANNEL_TEMP_1_REF,         /* 温度传感器1参考 */
    ADC_CHANNEL_TEMP_2_SIGNAL,      /* 温度传感器2信号 */
    ADC_CHANNEL_TEMP_2_REF,         /* 温度传感器2参考 */
    ADC_CHANNEL_TEMP_3_SIGNAL,      /* 温度传感器3信号 */
    ADC_CHANNEL_TEMP_3_REF,         /* 温度传感器3参考 */

    /* 电源监控 */
    ADC_CHANNEL_VOLTAGE_24V,        /* 24V电源监控 */
    ADC_CHANNEL_VOLTAGE_12V,        /* 12V电源监控 */
    ADC_CHANNEL_VOLTAGE_5V,         /* 5V电源监控 */
    ADC_CHANNEL_VOLTAGE_3V3,        /* 3.3V电源监控 */

    /* 电流监控 */
    ADC_CHANNEL_CURRENT_TOTAL,      /* 总电流监控 */
    ADC_CHANNEL_CURRENT_HEATER,     /* 加热器电流监控 */

    /* 备用通道 */
    ADC_CHANNEL_SPARE_1,            /* 备用通道1 */
    ADC_CHANNEL_SPARE_2,            /* 备用通道2 */

    ADC_CHANNEL_COUNT               /* 通道总数 */
} adc_channel_id_t;

/* ADC转换结果 */
typedef struct {
    uint16_t raw_value;         /* 原始ADC值 */
    float voltage;              /* 转换电压值 */
    float physical_value;       /* 物理量值 */
    uint32_t timestamp;         /* 时间戳 */
    bool valid;                 /* 数据有效性 */
} adc_result_t;

/* ADC校准数据 */
typedef struct {
    float offset;               /* 偏移校准 */
    float gain;                 /* 增益校准 */
    bool calibrated;            /* 校准状态 */
} adc_calibration_t;

/* 基础ADC操作接口 */
hal_result_t adc_hal_init(adc_handle_t* handle, const adc_config_t* config);
hal_result_t adc_hal_deinit(adc_handle_t* handle);
hal_result_t adc_hal_start_conversion(adc_handle_t* handle);
hal_result_t adc_hal_stop_conversion(adc_handle_t* handle);
hal_result_t adc_hal_get_conversion_result(adc_handle_t* handle, adc_channel_id_t channel, adc_result_t* result);
hal_result_t adc_hal_start_dma_conversion(adc_handle_t* handle);
hal_result_t adc_hal_stop_dma_conversion(adc_handle_t* handle);

/* 单通道操作接口 */
hal_result_t adc_hal_read_channel(adc_handle_t* handle, adc_channel_id_t channel, uint16_t* value);
hal_result_t adc_hal_read_channel_voltage(adc_handle_t* handle, adc_channel_id_t channel, float* voltage);
hal_result_t adc_hal_configure_channel(adc_handle_t* handle, adc_channel_id_t channel, const adc_channel_config_t* config);

/* 多通道操作接口 */
hal_result_t adc_hal_read_multiple_channels(adc_handle_t* handle, adc_channel_id_t* channels, uint8_t count, adc_result_t* results);
hal_result_t adc_hal_start_scan_conversion(adc_handle_t* handle, adc_channel_id_t* channels, uint8_t count);

/* 校准接口 */
hal_result_t adc_hal_calibrate(adc_handle_t* handle);
hal_result_t adc_hal_set_calibration(adc_handle_t* handle, adc_channel_id_t channel, const adc_calibration_t* calibration);
hal_result_t adc_hal_get_calibration(adc_handle_t* handle, adc_channel_id_t channel, adc_calibration_t* calibration);

/* 供墨系统专用接口 */
hal_result_t adc_hal_system_init(void);
hal_result_t adc_hal_system_deinit(void);
hal_result_t adc_hal_read_liquid_level(uint8_t sensor_id, float* level_mm);
hal_result_t adc_hal_read_pressure(uint8_t sensor_id, float* pressure_kpa);
hal_result_t adc_hal_read_temperature(uint8_t sensor_id, float* temperature_c);
hal_result_t adc_hal_read_voltage_monitor(uint8_t channel_id, float* voltage);
hal_result_t adc_hal_read_current_monitor(uint8_t channel_id, float* current_ma);

/* 过滤和处理接口 */
hal_result_t adc_hal_enable_filter(adc_handle_t* handle, adc_channel_id_t channel, bool enable);
hal_result_t adc_hal_set_filter_coefficient(adc_handle_t* handle, adc_channel_id_t channel, float coefficient);
hal_result_t adc_hal_get_averaged_value(adc_handle_t* handle, adc_channel_id_t channel, uint8_t samples, float* average);

/* 监控和诊断接口 */
hal_result_t adc_hal_self_test(adc_handle_t* handle);
hal_result_t adc_hal_get_conversion_time(adc_handle_t* handle, uint32_t* time_us);
hal_result_t adc_hal_check_reference_voltage(adc_handle_t* handle, float* vref);
hal_result_t adc_hal_get_temperature_sensor(float* temperature);

/* 回调函数类型 */
typedef void (*adc_conversion_complete_callback_t)(adc_handle_t* handle, adc_result_t* results, uint8_t count);
typedef void (*adc_error_callback_t)(adc_handle_t* handle, hal_result_t error);

/* 回调注册接口 */
hal_result_t adc_hal_register_conversion_callback(adc_handle_t* handle, adc_conversion_complete_callback_t callback);
hal_result_t adc_hal_register_error_callback(adc_handle_t* handle, adc_error_callback_t callback);
hal_result_t adc_hal_unregister_callbacks(adc_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif /* ADC_HAL_H */