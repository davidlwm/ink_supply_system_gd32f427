/**
 * @file filter.c
 * @brief 数字滤波器实现 - 8周v4标准
 * @version V4.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description 数字滤波器模块实现，支持多种滤波算法
 */

#include "middleware/filter.h"
#include "middleware/middleware_common.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include <math.h>

/* 私有变量 */
static filter_instance_t g_filters[MW_MAX_FILTERS];
static uint8_t g_filter_count = 0;
static bool g_filter_system_initialized = false;
static SemaphoreHandle_t g_filter_mutex = NULL;

/* 私有函数声明 */
static int find_free_filter_slot(void);
static bool is_valid_filter_id(int filter_id);
static mw_result_t validate_filter_params(filter_type_t type, const void* params);
static void update_filter_statistics(int filter_id, uint32_t process_time_us);

/**
 * @brief 滤波器系统初始化
 * @return mw_result_t 操作结果
 */
mw_result_t filter_system_init(void)
{
    if (g_filter_system_initialized) {
        return MW_ERROR_ALREADY_INITIALIZED;
    }

    /* 创建互斥锁 */
    g_filter_mutex = xSemaphoreCreateMutex();
    if (g_filter_mutex == NULL) {
        return MW_ERROR_OUT_OF_MEMORY;
    }

    /* 初始化滤波器实例数组 */
    memset(g_filters, 0, sizeof(g_filters));
    g_filter_count = 0;

    /* 注册到中间件系统 */
    middleware_register_module(MW_MODULE_FILTER, "Filter", MIDDLEWARE_VERSION_STRING);
    middleware_set_module_state(MW_MODULE_FILTER, MW_STATE_READY);

    g_filter_system_initialized = true;
    return MW_OK;
}

/**
 * @brief 滤波器系统反初始化
 * @return mw_result_t 操作结果
 */
mw_result_t filter_system_deinit(void)
{
    if (!g_filter_system_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    /* 销毁所有滤波器 */
    filter_destroy_all();

    /* 删除互斥锁 */
    if (g_filter_mutex != NULL) {
        vSemaphoreDelete(g_filter_mutex);
        g_filter_mutex = NULL;
    }

    /* 从中间件系统注销 */
    middleware_unregister_module(MW_MODULE_FILTER);

    g_filter_system_initialized = false;
    return MW_OK;
}

/* =================================================================== */
/* 滑动平均滤波器实现 */
/* =================================================================== */

/**
 * @brief 滑动平均滤波器初始化
 * @param filter 滤波器指针
 * @param window_size 窗口大小
 * @return mw_result_t 操作结果
 */
mw_result_t ma_filter_init(moving_average_filter_t* filter, uint8_t window_size)
{
    MW_CHECK_NULL(filter);
    MW_CHECK_RANGE(window_size, 1, MOVING_AVERAGE_MAX_WINDOW);

    memset(filter, 0, sizeof(moving_average_filter_t));
    filter->window_size = window_size;
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0f;
    filter->output = 0.0f;
    filter->state = FILTER_STATE_IDLE;
    filter->initialized = true;
    memset(&filter->stats, 0, sizeof(mw_statistics_t));
    filter->stats.min_process_time_us = UINT32_MAX;

    return MW_OK;
}

/**
 * @brief 滑动平均滤波器重置
 * @param filter 滤波器指针
 * @return mw_result_t 操作结果
 */
mw_result_t ma_filter_reset(moving_average_filter_t* filter)
{
    MW_CHECK_NULL(filter);

    if (!filter->initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    memset(filter->buffer, 0, sizeof(filter->buffer));
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0f;
    filter->output = 0.0f;
    filter->state = FILTER_STATE_IDLE;

    return MW_OK;
}

/**
 * @brief 滑动平均滤波处理
 * @param filter 滤波器指针
 * @param input 输入值
 * @param output 输出值指针
 * @return mw_result_t 操作结果
 */
mw_result_t ma_filter_process(moving_average_filter_t* filter, float input, float* output)
{
    MW_CHECK_NULL(filter);
    MW_CHECK_NULL(output);

    if (!filter->initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    uint32_t start_time = middleware_get_tick_us();

    filter->state = FILTER_STATE_ACTIVE;

    /* 如果缓冲区已满，减去要被替换的值 */
    if (filter->count >= filter->window_size) {
        filter->sum -= filter->buffer[filter->index];
    } else {
        filter->count++;
    }

    /* 添加新值 */
    filter->buffer[filter->index] = input;
    filter->sum += input;

    /* 更新索引 */
    filter->index = (filter->index + 1) % filter->window_size;

    /* 计算平均值 */
    filter->output = filter->sum / filter->count;
    *output = filter->output;

    filter->state = FILTER_STATE_IDLE;

    /* 更新统计信息 */
    uint32_t process_time = middleware_get_tick_us() - start_time;
    middleware_update_statistics(MW_MODULE_FILTER, process_time);

    return MW_OK;
}

/**
 * @brief 设置滑动平均滤波器窗口大小
 * @param filter 滤波器指针
 * @param window_size 窗口大小
 * @return mw_result_t 操作结果
 */
mw_result_t ma_filter_set_window_size(moving_average_filter_t* filter, uint8_t window_size)
{
    MW_CHECK_NULL(filter);
    MW_CHECK_RANGE(window_size, 1, MOVING_AVERAGE_MAX_WINDOW);

    if (!filter->initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    filter->window_size = window_size;

    /* 如果新窗口小于当前数据量，需要重置 */
    if (filter->count > window_size) {
        return ma_filter_reset(filter);
    }

    return MW_OK;
}

/**
 * @brief 获取滑动平均滤波器窗口大小
 * @param filter 滤波器指针
 * @return uint8_t 窗口大小
 */
uint8_t ma_filter_get_window_size(const moving_average_filter_t* filter)
{
    if (filter == NULL || !filter->initialized) {
        return 0;
    }
    return filter->window_size;
}

/**
 * @brief 获取滑动平均滤波器状态
 * @param filter 滤波器指针
 * @return filter_state_t 滤波器状态
 */
filter_state_t ma_filter_get_state(const moving_average_filter_t* filter)
{
    if (filter == NULL || !filter->initialized) {
        return FILTER_STATE_ERROR;
    }
    return filter->state;
}

/* =================================================================== */
/* 卡尔曼滤波器实现 */
/* =================================================================== */

/**
 * @brief 卡尔曼滤波器初始化
 * @param filter 滤波器指针
 * @param Q 过程噪声协方差
 * @param R 测量噪声协方差
 * @param initial_estimate 初始估计值
 * @return mw_result_t 操作结果
 */
mw_result_t kalman_filter_init(kalman_filter_t* filter, float Q, float R, float initial_estimate)
{
    MW_CHECK_NULL(filter);
    MW_CHECK_PARAM(Q > 0.0f && R > 0.0f);

    memset(filter, 0, sizeof(kalman_filter_t));
    filter->Q = Q;
    filter->R = R;
    filter->P = 1.0f;  /* 初始估计误差协方差 */
    filter->K = 0.0f;
    filter->x = initial_estimate;
    filter->x_pred = initial_estimate;
    filter->output = initial_estimate;
    filter->state = FILTER_STATE_IDLE;
    filter->initialized = true;
    memset(&filter->stats, 0, sizeof(mw_statistics_t));
    filter->stats.min_process_time_us = UINT32_MAX;

    return MW_OK;
}

/**
 * @brief 卡尔曼滤波器重置
 * @param filter 滤波器指针
 * @param initial_estimate 初始估计值
 * @return mw_result_t 操作结果
 */
mw_result_t kalman_filter_reset(kalman_filter_t* filter, float initial_estimate)
{
    MW_CHECK_NULL(filter);

    if (!filter->initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    filter->P = 1.0f;
    filter->K = 0.0f;
    filter->x = initial_estimate;
    filter->x_pred = initial_estimate;
    filter->output = initial_estimate;
    filter->state = FILTER_STATE_IDLE;

    return MW_OK;
}

/**
 * @brief 卡尔曼滤波处理
 * @param filter 滤波器指针
 * @param measurement 测量值
 * @param output 输出值指针
 * @return mw_result_t 操作结果
 */
mw_result_t kalman_filter_process(kalman_filter_t* filter, float measurement, float* output)
{
    MW_CHECK_NULL(filter);
    MW_CHECK_NULL(output);

    if (!filter->initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    uint32_t start_time = middleware_get_tick_us();

    filter->state = FILTER_STATE_ACTIVE;

    /* 预测步骤 */
    filter->x_pred = filter->x;  /* 状态预测 (假设状态转移矩阵为1) */
    filter->P = filter->P + filter->Q;  /* 误差协方差预测 */

    /* 更新步骤 */
    filter->K = filter->P / (filter->P + filter->R);  /* 卡尔曼增益 */
    filter->x = filter->x_pred + filter->K * (measurement - filter->x_pred);  /* 状态更新 */
    filter->P = (1 - filter->K) * filter->P;  /* 误差协方差更新 */

    filter->output = filter->x;
    *output = filter->output;

    filter->state = FILTER_STATE_IDLE;

    /* 更新统计信息 */
    uint32_t process_time = middleware_get_tick_us() - start_time;
    middleware_update_statistics(MW_MODULE_FILTER, process_time);

    return MW_OK;
}

/**
 * @brief 设置卡尔曼滤波器噪声参数
 * @param filter 滤波器指针
 * @param process_noise 过程噪声
 * @param measurement_noise 测量噪声
 * @return mw_result_t 操作结果
 */
mw_result_t kalman_filter_set_noise(kalman_filter_t* filter, float process_noise, float measurement_noise)
{
    MW_CHECK_NULL(filter);
    MW_CHECK_PARAM(process_noise > 0.0f && measurement_noise > 0.0f);

    if (!filter->initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    filter->Q = process_noise;
    filter->R = measurement_noise;

    return MW_OK;
}

/* =================================================================== */
/* 低通滤波器实现 */
/* =================================================================== */

/**
 * @brief 低通滤波器初始化
 * @param filter 滤波器指针
 * @param cutoff_freq 截止频率
 * @param sample_freq 采样频率
 * @return mw_result_t 操作结果
 */
mw_result_t lowpass_filter_init(low_pass_filter_t* filter, float cutoff_freq, float sample_freq)
{
    MW_CHECK_NULL(filter);
    MW_CHECK_PARAM(cutoff_freq > 0.0f && sample_freq > 0.0f);
    MW_CHECK_PARAM(cutoff_freq < sample_freq / 2.0f);  /* 奈奎斯特定理 */

    memset(filter, 0, sizeof(low_pass_filter_t));
    filter->cutoff_freq = cutoff_freq;
    filter->sample_freq = sample_freq;

    /* 计算滤波系数 */
    float omega = 2.0f * M_PI * cutoff_freq / sample_freq;
    filter->alpha = omega / (1.0f + omega);

    filter->output = 0.0f;
    filter->prev_output = 0.0f;
    filter->state = FILTER_STATE_IDLE;
    filter->initialized = true;
    memset(&filter->stats, 0, sizeof(mw_statistics_t));
    filter->stats.min_process_time_us = UINT32_MAX;

    return MW_OK;
}

/**
 * @brief 低通滤波器重置
 * @param filter 滤波器指针
 * @return mw_result_t 操作结果
 */
mw_result_t lowpass_filter_reset(low_pass_filter_t* filter)
{
    MW_CHECK_NULL(filter);

    if (!filter->initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    filter->output = 0.0f;
    filter->prev_output = 0.0f;
    filter->state = FILTER_STATE_IDLE;

    return MW_OK;
}

/**
 * @brief 低通滤波处理
 * @param filter 滤波器指针
 * @param input 输入值
 * @param output 输出值指针
 * @return mw_result_t 操作结果
 */
mw_result_t lowpass_filter_process(low_pass_filter_t* filter, float input, float* output)
{
    MW_CHECK_NULL(filter);
    MW_CHECK_NULL(output);

    if (!filter->initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    uint32_t start_time = middleware_get_tick_us();

    filter->state = FILTER_STATE_ACTIVE;

    /* 一阶低通滤波器: y[n] = α * x[n] + (1-α) * y[n-1] */
    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    filter->prev_output = filter->output;

    *output = filter->output;

    filter->state = FILTER_STATE_IDLE;

    /* 更新统计信息 */
    uint32_t process_time = middleware_get_tick_us() - start_time;
    middleware_update_statistics(MW_MODULE_FILTER, process_time);

    return MW_OK;
}

/* =================================================================== */
/* 高级滤波器管理接口实现 */
/* =================================================================== */

/**
 * @brief 创建滑动平均滤波器
 * @param name 滤波器名称
 * @param window_size 窗口大小
 * @return int 滤波器ID，失败返回-1
 */
int filter_create_moving_average(const char* name, uint8_t window_size)
{
    if (!g_filter_system_initialized || name == NULL || window_size == 0) {
        return -1;
    }

    if (xSemaphoreTake(g_filter_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return -1;
    }

    int slot = find_free_filter_slot();
    if (slot < 0) {
        xSemaphoreGive(g_filter_mutex);
        return -1;
    }

    filter_instance_t* instance = &g_filters[slot];
    instance->type = FILTER_TYPE_MOVING_AVERAGE;
    instance->id = slot;
    instance->in_use = true;
    instance->name = name;
    instance->created_time = middleware_get_timestamp();

    mw_result_t result = ma_filter_init(&instance->filter.ma, window_size);
    if (result != MW_OK) {
        instance->in_use = false;
        xSemaphoreGive(g_filter_mutex);
        return -1;
    }

    g_filter_count++;
    xSemaphoreGive(g_filter_mutex);
    return slot;
}

/**
 * @brief 创建卡尔曼滤波器
 * @param name 滤波器名称
 * @param Q 过程噪声协方差
 * @param R 测量噪声协方差
 * @param initial_estimate 初始估计值
 * @return int 滤波器ID，失败返回-1
 */
int filter_create_kalman(const char* name, float Q, float R, float initial_estimate)
{
    if (!g_filter_system_initialized || name == NULL) {
        return -1;
    }

    if (xSemaphoreTake(g_filter_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return -1;
    }

    int slot = find_free_filter_slot();
    if (slot < 0) {
        xSemaphoreGive(g_filter_mutex);
        return -1;
    }

    filter_instance_t* instance = &g_filters[slot];
    instance->type = FILTER_TYPE_KALMAN;
    instance->id = slot;
    instance->in_use = true;
    instance->name = name;
    instance->created_time = middleware_get_timestamp();

    mw_result_t result = kalman_filter_init(&instance->filter.kalman, Q, R, initial_estimate);
    if (result != MW_OK) {
        instance->in_use = false;
        xSemaphoreGive(g_filter_mutex);
        return -1;
    }

    g_filter_count++;
    xSemaphoreGive(g_filter_mutex);
    return slot;
}

/**
 * @brief 通过ID处理滤波器
 * @param filter_id 滤波器ID
 * @param input 输入值
 * @param output 输出值指针
 * @return mw_result_t 操作结果
 */
mw_result_t filter_process_by_id(int filter_id, float input, float* output)
{
    if (!is_valid_filter_id(filter_id) || output == NULL) {
        return MW_ERROR_INVALID_PARAM;
    }

    filter_instance_t* instance = &g_filters[filter_id];
    instance->last_update_time = middleware_get_timestamp();

    switch (instance->type) {
        case FILTER_TYPE_MOVING_AVERAGE:
            return ma_filter_process(&instance->filter.ma, input, output);
        case FILTER_TYPE_KALMAN:
            return kalman_filter_process(&instance->filter.kalman, input, output);
        case FILTER_TYPE_LOW_PASS:
            return lowpass_filter_process(&instance->filter.lowpass, input, output);
        default:
            return MW_ERROR_NOT_SUPPORTED;
    }
}

/**
 * @brief 销毁所有滤波器
 * @return mw_result_t 操作结果
 */
mw_result_t filter_destroy_all(void)
{
    if (!g_filter_system_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    if (xSemaphoreTake(g_filter_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return MW_ERROR_TIMEOUT;
    }

    memset(g_filters, 0, sizeof(g_filters));
    g_filter_count = 0;

    xSemaphoreGive(g_filter_mutex);
    return MW_OK;
}

/**
 * @brief 获取滤波器数量
 * @return uint8_t 滤波器数量
 */
uint8_t filter_get_count(void)
{
    return g_filter_count;
}

/**
 * @brief 滤波器自检
 * @return mw_result_t 操作结果
 */
mw_result_t filter_self_test(void)
{
    if (!g_filter_system_initialized) {
        return MW_ERROR_NOT_INITIALIZED;
    }

    /* 创建测试滤波器 */
    int test_ma = filter_create_moving_average("test_ma", 5);
    if (test_ma < 0) {
        return MW_ERROR;
    }

    /* 测试滤波器处理 */
    float output;
    mw_result_t result = filter_process_by_id(test_ma, 1.0f, &output);

    /* 清理测试滤波器 */
    filter_destroy(test_ma);

    return result;
}

/* 私有函数实现 */

/**
 * @brief 查找空闲滤波器槽位
 * @return int 槽位索引，失败返回-1
 */
static int find_free_filter_slot(void)
{
    for (int i = 0; i < MW_MAX_FILTERS; i++) {
        if (!g_filters[i].in_use) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 检查滤波器ID是否有效
 * @param filter_id 滤波器ID
 * @return bool 有效性
 */
static bool is_valid_filter_id(int filter_id)
{
    return (filter_id >= 0 && filter_id < MW_MAX_FILTERS && g_filters[filter_id].in_use);
}

/**
 * @brief 销毁滤波器
 * @param filter_id 滤波器ID
 * @return mw_result_t 操作结果
 */
mw_result_t filter_destroy(int filter_id)
{
    if (!is_valid_filter_id(filter_id)) {
        return MW_ERROR_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_filter_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return MW_ERROR_TIMEOUT;
    }

    memset(&g_filters[filter_id], 0, sizeof(filter_instance_t));
    g_filter_count--;

    xSemaphoreGive(g_filter_mutex);
    return MW_OK;
}