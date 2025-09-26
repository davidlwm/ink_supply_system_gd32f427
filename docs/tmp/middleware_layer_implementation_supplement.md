# 中间件层补充实现文档 (GD32F427压缩版6周)

## 文档概述

本文档是《供墨系统控制板卡简化实现设计文档(GD32F427压缩版6周)v4》的中间件层补充实现，提供完整的中间件模块代码实现。

---

## 1. 数字滤波器实现 (filter.c/h)

### 1.1 filter.h 头文件

```c
/**
 * @file    filter.h
 * @brief   数字滤波器模块 - 6周压缩版
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdint.h>
#include <stdbool.h>

// 滤波器类型定义
typedef enum {
    FILTER_LIQUID_LEVEL_1 = 0,
    FILTER_LIQUID_LEVEL_2,
    FILTER_PRESSURE_1,
    FILTER_PRESSURE_2,
    FILTER_TEMPERATURE_1,
    FILTER_TEMPERATURE_2,
    FILTER_TEMPERATURE_3,
    FILTER_MAX_COUNT
} filter_id_t;

// 滤波器配置结构
typedef struct {
    uint8_t window_size;        // 滑动平均窗口大小
    float kalman_q;            // 卡尔曼滤波过程噪声
    float kalman_r;            // 卡尔曼滤波测量噪声
    float limit_max;           // 上限值
    float limit_min;           // 下限值
} filter_config_t;

// 公共函数接口
void filter_init(void);
float filter_process(filter_id_t filter_id, float raw_value);
void filter_reset(filter_id_t filter_id);
void filter_config_update(filter_id_t filter_id, const filter_config_t *config);

#endif /* __FILTER_H__ */
```

### 1.2 filter.c 实现文件

```c
/**
 * @file    filter.c
 * @brief   数字滤波器模块实现 - 6周压缩版
 * @version V4.0
 * @date    2025-09-27
 */

#include "filter.h"
#include <string.h>
#include <math.h>

// 滑动平均滤波器数据
typedef struct {
    float buffer[16];          // 滑动平均缓冲区
    uint8_t index;            // 当前索引
    uint8_t count;            // 有效数据计数
    uint8_t window_size;      // 窗口大小
} moving_average_t;

// 卡尔曼滤波器数据
typedef struct {
    float x;                  // 状态估计
    float p;                  // 估计误差协方差
    float q;                  // 过程噪声协方差
    float r;                  // 测量噪声协方差
} kalman_filter_t;

// 限幅滤波器数据
typedef struct {
    float limit_max;          // 上限值
    float limit_min;          // 下限值
    float last_valid_value;   // 上次有效值
    bool first_run;           // 首次运行标志
} limit_filter_t;

// 综合滤波器结构
typedef struct {
    moving_average_t ma_filter;
    kalman_filter_t kalman_filter;
    limit_filter_t limit_filter;
    bool initialized;
} digital_filter_t;

// 全局滤波器数组
static digital_filter_t g_filters[FILTER_MAX_COUNT];

// 默认滤波器配置
static const filter_config_t default_configs[FILTER_MAX_COUNT] = {
    // 液位传感器滤波配置
    {8, 0.1f, 1.0f, 2000.0f, 0.0f},      // FILTER_LIQUID_LEVEL_1
    {8, 0.1f, 1.0f, 2000.0f, 0.0f},      // FILTER_LIQUID_LEVEL_2

    // 压力传感器滤波配置
    {6, 0.5f, 2.0f, 110000.0f, -200.0f}, // FILTER_PRESSURE_1
    {6, 0.5f, 2.0f, 110000.0f, -200.0f}, // FILTER_PRESSURE_2

    // 温度传感器滤波配置
    {10, 0.2f, 0.5f, 700.0f, -50.0f},    // FILTER_TEMPERATURE_1
    {10, 0.2f, 0.5f, 700.0f, -50.0f},    // FILTER_TEMPERATURE_2
    {10, 0.2f, 0.5f, 700.0f, -50.0f},    // FILTER_TEMPERATURE_3
};

/**
 * @brief  滤波器系统初始化
 */
void filter_init(void)
{
    for(int i = 0; i < FILTER_MAX_COUNT; i++) {
        // 清零滤波器结构
        memset(&g_filters[i], 0, sizeof(digital_filter_t));

        // 设置默认配置
        g_filters[i].ma_filter.window_size = default_configs[i].window_size;
        g_filters[i].kalman_filter.q = default_configs[i].kalman_q;
        g_filters[i].kalman_filter.r = default_configs[i].kalman_r;
        g_filters[i].kalman_filter.p = 1.0f;
        g_filters[i].limit_filter.limit_max = default_configs[i].limit_max;
        g_filters[i].limit_filter.limit_min = default_configs[i].limit_min;
        g_filters[i].limit_filter.first_run = true;
        g_filters[i].initialized = true;
    }
}

/**
 * @brief  数字滤波处理
 * @param  filter_id 滤波器ID
 * @param  raw_value 原始值
 * @retval 滤波后的值
 */
float filter_process(filter_id_t filter_id, float raw_value)
{
    if(filter_id >= FILTER_MAX_COUNT || !g_filters[filter_id].initialized) {
        return raw_value;
    }

    digital_filter_t *filter = &g_filters[filter_id];
    float filtered_value = raw_value;

    // 1. 限幅滤波 (去除明显异常值)
    if(filtered_value > filter->limit_filter.limit_max ||
       filtered_value < filter->limit_filter.limit_min) {
        if(!filter->limit_filter.first_run) {
            filtered_value = filter->limit_filter.last_valid_value;
        } else {
            // 首次运行时如果超限，使用中间值
            filtered_value = (filter->limit_filter.limit_max + filter->limit_filter.limit_min) / 2.0f;
        }
    } else {
        filter->limit_filter.last_valid_value = filtered_value;
        filter->limit_filter.first_run = false;
    }

    // 2. 卡尔曼滤波 (最优估计)
    kalman_filter_t *kf = &filter->kalman_filter;

    // 预测步骤
    float x_pred = kf->x;
    float p_pred = kf->p + kf->q;

    // 更新步骤
    float k = p_pred / (p_pred + kf->r);
    kf->x = x_pred + k * (filtered_value - x_pred);
    kf->p = (1.0f - k) * p_pred;

    filtered_value = kf->x;

    // 3. 滑动平均滤波 (平滑输出)
    moving_average_t *ma = &filter->ma_filter;

    ma->buffer[ma->index] = filtered_value;
    ma->index = (ma->index + 1) % ma->window_size;

    if(ma->count < ma->window_size) {
        ma->count++;
    }

    float sum = 0.0f;
    for(int i = 0; i < ma->count; i++) {
        sum += ma->buffer[i];
    }
    filtered_value = sum / ma->count;

    return filtered_value;
}

/**
 * @brief  重置滤波器
 * @param  filter_id 滤波器ID
 */
void filter_reset(filter_id_t filter_id)
{
    if(filter_id >= FILTER_MAX_COUNT) return;

    digital_filter_t *filter = &g_filters[filter_id];

    // 重置滑动平均
    memset(filter->ma_filter.buffer, 0, sizeof(filter->ma_filter.buffer));
    filter->ma_filter.index = 0;
    filter->ma_filter.count = 0;

    // 重置卡尔曼滤波
    filter->kalman_filter.x = 0.0f;
    filter->kalman_filter.p = 1.0f;

    // 重置限幅滤波
    filter->limit_filter.first_run = true;
}

/**
 * @brief  更新滤波器配置
 * @param  filter_id 滤波器ID
 * @param  config 新配置
 */
void filter_config_update(filter_id_t filter_id, const filter_config_t *config)
{
    if(filter_id >= FILTER_MAX_COUNT || config == NULL) return;

    digital_filter_t *filter = &g_filters[filter_id];

    filter->ma_filter.window_size = config->window_size;
    filter->kalman_filter.q = config->kalman_q;
    filter->kalman_filter.r = config->kalman_r;
    filter->limit_filter.limit_max = config->limit_max;
    filter->limit_filter.limit_min = config->limit_min;

    // 重置滤波器状态
    filter_reset(filter_id);
}
```

---

## 2. PID控制器实现 (pid.c/h)

### 2.1 pid.h 头文件

```c
/**
 * @file    pid.h
 * @brief   PID控制器模块 - 6周压缩版
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include <stdbool.h>

// PID控制器类型
typedef enum {
    PID_TEMPERATURE_1 = 0,
    PID_TEMPERATURE_2,
    PID_TEMPERATURE_3,
    PID_PRESSURE_1,
    PID_PRESSURE_2,
    PID_LIQUID_LEVEL_1,
    PID_LIQUID_LEVEL_2,
    PID_MAX_COUNT
} pid_id_t;

// PID参数结构
typedef struct {
    float kp;                 // 比例系数
    float ki;                 // 积分系数
    float kd;                 // 微分系数
    float setpoint;           // 目标值
    float output_max;         // 输出上限
    float output_min;         // 输出下限
    float integral_max;       // 积分限制
    uint32_t sample_time_ms;  // 采样时间(ms)
} pid_params_t;

// PID运行状态
typedef struct {
    float last_error;         // 上次误差
    float integral;           // 积分累积
    float derivative;         // 微分项
    uint32_t last_time;       // 上次计算时间
    float last_output;        // 上次输出
    bool first_run;           // 首次运行标志
} pid_state_t;

// PID控制器结构
typedef struct {
    pid_params_t params;      // PID参数
    pid_state_t state;        // 运行状态
    bool enabled;             // 使能标志
    bool auto_mode;           // 自动模式标志
} pid_controller_t;

// 公共函数接口
void pid_init(void);
float pid_compute(pid_id_t pid_id, float input);
void pid_set_params(pid_id_t pid_id, const pid_params_t *params);
void pid_set_setpoint(pid_id_t pid_id, float setpoint);
void pid_set_mode(pid_id_t pid_id, bool auto_mode);
void pid_reset(pid_id_t pid_id);
float pid_get_output(pid_id_t pid_id);

#endif /* __PID_H__ */
```

### 2.2 pid.c 实现文件

```c
/**
 * @file    pid.c
 * @brief   PID控制器模块实现 - 6周压缩版
 * @version V4.0
 * @date    2025-09-27
 */

#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

// 全局PID控制器数组
static pid_controller_t g_pid_controllers[PID_MAX_COUNT];

// 默认PID参数配置
static const pid_params_t default_pid_params[PID_MAX_COUNT] = {
    // 温度控制器参数 (基于加热器特性)
    {2.0f, 0.1f, 0.5f, 50.0f, 100.0f, 0.0f, 100.0f, 100},   // PID_TEMPERATURE_1
    {2.0f, 0.1f, 0.5f, 60.0f, 100.0f, 0.0f, 100.0f, 100},   // PID_TEMPERATURE_2
    {2.0f, 0.1f, 0.5f, 55.0f, 100.0f, 0.0f, 100.0f, 100},   // PID_TEMPERATURE_3

    // 压力控制器参数 (基于泵特性)
    {1.5f, 0.05f, 0.3f, 1000.0f, 5000.0f, 200.0f, 1000.0f, 200}, // PID_PRESSURE_1
    {1.5f, 0.05f, 0.3f, 1000.0f, 5000.0f, 200.0f, 1000.0f, 200}, // PID_PRESSURE_2

    // 液位控制器参数 (基于阀门特性)
    {3.0f, 0.2f, 0.1f, 50.0f, 100.0f, 0.0f, 50.0f, 500},    // PID_LIQUID_LEVEL_1
    {3.0f, 0.2f, 0.1f, 50.0f, 100.0f, 0.0f, 50.0f, 500},    // PID_LIQUID_LEVEL_2
};

/**
 * @brief  PID控制器系统初始化
 */
void pid_init(void)
{
    for(int i = 0; i < PID_MAX_COUNT; i++) {
        // 清零控制器结构
        memset(&g_pid_controllers[i], 0, sizeof(pid_controller_t));

        // 设置默认参数
        memcpy(&g_pid_controllers[i].params, &default_pid_params[i], sizeof(pid_params_t));

        // 初始化状态
        g_pid_controllers[i].state.first_run = true;
        g_pid_controllers[i].enabled = true;
        g_pid_controllers[i].auto_mode = true;
    }
}

/**
 * @brief  PID计算
 * @param  pid_id PID控制器ID
 * @param  input 当前输入值
 * @retval PID输出值
 */
float pid_compute(pid_id_t pid_id, float input)
{
    if(pid_id >= PID_MAX_COUNT) return 0.0f;

    pid_controller_t *pid = &g_pid_controllers[pid_id];

    if(!pid->enabled || !pid->auto_mode) {
        return pid->state.last_output;
    }

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // 首次运行特殊处理
    if(pid->state.first_run) {
        pid->state.last_time = current_time;
        pid->state.last_error = pid->params.setpoint - input;
        pid->state.first_run = false;
        return pid->state.last_output;
    }

    // 计算时间间隔
    uint32_t dt_ms = current_time - pid->state.last_time;
    if(dt_ms < pid->params.sample_time_ms) {
        return pid->state.last_output; // 还未到采样时间
    }

    float dt = (float)dt_ms / 1000.0f; // 转换为秒

    // 计算误差
    float error = pid->params.setpoint - input;

    // 比例项
    float proportional = pid->params.kp * error;

    // 积分项 (带积分分离和限幅)
    if(fabs(error) < pid->params.integral_max) {
        pid->state.integral += error * dt;

        // 积分限幅
        float integral_limit = pid->params.integral_max;
        if(pid->state.integral > integral_limit) {
            pid->state.integral = integral_limit;
        } else if(pid->state.integral < -integral_limit) {
            pid->state.integral = -integral_limit;
        }
    }
    float integral = pid->params.ki * pid->state.integral;

    // 微分项 (带低通滤波)
    float derivative_raw = (error - pid->state.last_error) / dt;
    pid->state.derivative = 0.8f * pid->state.derivative + 0.2f * derivative_raw; // 低通滤波
    float derivative = pid->params.kd * pid->state.derivative;

    // PID输出
    float output = proportional + integral + derivative;

    // 输出限制
    if(output > pid->params.output_max) {
        output = pid->params.output_max;
    } else if(output < pid->params.output_min) {
        output = pid->params.output_min;
    }

    // 更新状态
    pid->state.last_error = error;
    pid->state.last_time = current_time;
    pid->state.last_output = output;

    return output;
}

/**
 * @brief  设置PID参数
 * @param  pid_id PID控制器ID
 * @param  params 新的PID参数
 */
void pid_set_params(pid_id_t pid_id, const pid_params_t *params)
{
    if(pid_id >= PID_MAX_COUNT || params == NULL) return;

    memcpy(&g_pid_controllers[pid_id].params, params, sizeof(pid_params_t));

    // 重置PID状态
    pid_reset(pid_id);
}

/**
 * @brief  设置目标值
 * @param  pid_id PID控制器ID
 * @param  setpoint 新的目标值
 */
void pid_set_setpoint(pid_id_t pid_id, float setpoint)
{
    if(pid_id >= PID_MAX_COUNT) return;

    g_pid_controllers[pid_id].params.setpoint = setpoint;
}

/**
 * @brief  设置PID模式
 * @param  pid_id PID控制器ID
 * @param  auto_mode true=自动模式, false=手动模式
 */
void pid_set_mode(pid_id_t pid_id, bool auto_mode)
{
    if(pid_id >= PID_MAX_COUNT) return;

    pid_controller_t *pid = &g_pid_controllers[pid_id];

    if(auto_mode && !pid->auto_mode) {
        // 从手动切换到自动时，重置状态
        pid_reset(pid_id);
    }

    pid->auto_mode = auto_mode;
}

/**
 * @brief  重置PID控制器
 * @param  pid_id PID控制器ID
 */
void pid_reset(pid_id_t pid_id)
{
    if(pid_id >= PID_MAX_COUNT) return;

    pid_controller_t *pid = &g_pid_controllers[pid_id];

    pid->state.last_error = 0.0f;
    pid->state.integral = 0.0f;
    pid->state.derivative = 0.0f;
    pid->state.last_output = 0.0f;
    pid->state.first_run = true;
}

/**
 * @brief  获取PID输出
 * @param  pid_id PID控制器ID
 * @retval 当前输出值
 */
float pid_get_output(pid_id_t pid_id)
{
    if(pid_id >= PID_MAX_COUNT) return 0.0f;

    return g_pid_controllers[pid_id].state.last_output;
}
```

---

## 3. 任务管理器实现 (tasks.c/h)

### 3.1 tasks.h 头文件

```c
/**
 * @file    tasks.h
 * @brief   FreeRTOS任务管理器 - 6周压缩版
 * @version V4.0
 * @date    2025-09-27
 */

#ifndef __TASKS_H__
#define __TASKS_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// 任务优先级定义 (数值越大优先级越高)
#define TASK_PRIORITY_SAFETY        7   // 安全任务 - 最高优先级
#define TASK_PRIORITY_CONTROL       6   // 控制任务 - 高优先级
#define TASK_PRIORITY_SENSOR        5   // 传感器任务 - 高优先级
#define TASK_PRIORITY_ACTUATOR      4   // 执行器任务 - 中高优先级
#define TASK_PRIORITY_COMM          3   // 通信任务 - 中等优先级
#define TASK_PRIORITY_DISPLAY       2   // 显示任务 - 低优先级
#define TASK_PRIORITY_CONFIG        1   // 配置任务 - 最低优先级

// 任务栈大小定义 (单位: 32位字)
#define TASK_STACK_SIZE_SAFETY      256
#define TASK_STACK_SIZE_CONTROL     512
#define TASK_STACK_SIZE_SENSOR      512
#define TASK_STACK_SIZE_ACTUATOR    512
#define TASK_STACK_SIZE_COMM        1024
#define TASK_STACK_SIZE_DISPLAY     512
#define TASK_STACK_SIZE_CONFIG      512

// 任务句柄声明
extern TaskHandle_t xSafetyTaskHandle;
extern TaskHandle_t xControlTaskHandle;
extern TaskHandle_t xSensorTaskHandle;
extern TaskHandle_t xActuatorTaskHandle;
extern TaskHandle_t xCommTaskHandle;
extern TaskHandle_t xDisplayTaskHandle;
extern TaskHandle_t xConfigTaskHandle;

// 队列和信号量声明
extern QueueHandle_t xSensorDataQueue;
extern QueueHandle_t xControlCommandQueue;
extern SemaphoreHandle_t xSystemConfigMutex;
extern SemaphoreHandle_t xSensorDataSemaphore;

// 任务函数声明
void sensor_task(void *pvParameters);
void actuator_task(void *pvParameters);
void control_task(void *pvParameters);
void comm_task(void *pvParameters);
void display_task(void *pvParameters);
void safety_task(void *pvParameters);
void config_task(void *pvParameters);

// 任务管理函数
void tasks_init(void);
void tasks_start_scheduler(void);
void tasks_suspend_all(void);
void tasks_resume_all(void);
TaskHandle_t tasks_get_handle(const char *task_name);

#endif /* __TASKS_H__ */
```

### 3.2 tasks.c 实现文件

```c
/**
 * @file    tasks.c
 * @brief   FreeRTOS任务管理器实现 - 6周压缩版
 * @version V4.0
 * @date    2025-09-27
 */

#include "tasks.h"
#include "sensors.h"
#include "actuators.h"
#include "control.h"
#include "communication.h"
#include "display.h"
#include "safety.h"
#include "config.h"

// 任务句柄定义
TaskHandle_t xSafetyTaskHandle = NULL;
TaskHandle_t xControlTaskHandle = NULL;
TaskHandle_t xSensorTaskHandle = NULL;
TaskHandle_t xActuatorTaskHandle = NULL;
TaskHandle_t xCommTaskHandle = NULL;
TaskHandle_t xDisplayTaskHandle = NULL;
TaskHandle_t xConfigTaskHandle = NULL;

// 队列和信号量定义
QueueHandle_t xSensorDataQueue = NULL;
QueueHandle_t xControlCommandQueue = NULL;
SemaphoreHandle_t xSystemConfigMutex = NULL;
SemaphoreHandle_t xSensorDataSemaphore = NULL;

// 任务统计信息
typedef struct {
    char task_name[16];
    uint32_t run_count;
    uint32_t max_run_time;
    uint32_t total_run_time;
    bool task_healthy;
} task_stats_t;

static task_stats_t g_task_stats[7];

/**
 * @brief  任务系统初始化
 */
void tasks_init(void)
{
    BaseType_t result;

    // 1. 创建队列
    xSensorDataQueue = xQueueCreate(10, sizeof(float) * 7); // 7个传感器数据
    if(xSensorDataQueue == NULL) {
        // 错误处理
        while(1);
    }

    xControlCommandQueue = xQueueCreate(5, sizeof(uint32_t));
    if(xControlCommandQueue == NULL) {
        while(1);
    }

    // 2. 创建信号量
    xSystemConfigMutex = xSemaphoreCreateMutex();
    if(xSystemConfigMutex == NULL) {
        while(1);
    }

    xSensorDataSemaphore = xSemaphoreCreateBinary();
    if(xSensorDataSemaphore == NULL) {
        while(1);
    }
    xSemaphoreGive(xSensorDataSemaphore);

    // 3. 创建任务 (按优先级顺序创建)

    // 安全任务 - 最高优先级
    result = xTaskCreate(
        safety_task,
        "Safety",
        TASK_STACK_SIZE_SAFETY,
        NULL,
        TASK_PRIORITY_SAFETY,
        &xSafetyTaskHandle
    );
    if(result != pdPASS) while(1);

    // 控制任务 - 高优先级
    result = xTaskCreate(
        control_task,
        "Control",
        TASK_STACK_SIZE_CONTROL,
        NULL,
        TASK_PRIORITY_CONTROL,
        &xControlTaskHandle
    );
    if(result != pdPASS) while(1);

    // 传感器任务 - 高优先级
    result = xTaskCreate(
        sensor_task,
        "Sensor",
        TASK_STACK_SIZE_SENSOR,
        NULL,
        TASK_PRIORITY_SENSOR,
        &xSensorTaskHandle
    );
    if(result != pdPASS) while(1);

    // 执行器任务 - 中高优先级
    result = xTaskCreate(
        actuator_task,
        "Actuator",
        TASK_STACK_SIZE_ACTUATOR,
        NULL,
        TASK_PRIORITY_ACTUATOR,
        &xActuatorTaskHandle
    );
    if(result != pdPASS) while(1);

    // 通信任务 - 中等优先级
    result = xTaskCreate(
        comm_task,
        "Comm",
        TASK_STACK_SIZE_COMM,
        NULL,
        TASK_PRIORITY_COMM,
        &xCommTaskHandle
    );
    if(result != pdPASS) while(1);

    // 显示任务 - 低优先级
    result = xTaskCreate(
        display_task,
        "Display",
        TASK_STACK_SIZE_DISPLAY,
        NULL,
        TASK_PRIORITY_DISPLAY,
        &xDisplayTaskHandle
    );
    if(result != pdPASS) while(1);

    // 配置任务 - 最低优先级
    result = xTaskCreate(
        config_task,
        "Config",
        TASK_STACK_SIZE_CONFIG,
        NULL,
        TASK_PRIORITY_CONFIG,
        &xConfigTaskHandle
    );
    if(result != pdPASS) while(1);

    // 4. 初始化任务统计
    const char *task_names[] = {"Safety", "Control", "Sensor", "Actuator", "Comm", "Display", "Config"};
    for(int i = 0; i < 7; i++) {
        strcpy(g_task_stats[i].task_name, task_names[i]);
        g_task_stats[i].run_count = 0;
        g_task_stats[i].max_run_time = 0;
        g_task_stats[i].total_run_time = 0;
        g_task_stats[i].task_healthy = true;
    }
}

/**
 * @brief  启动任务调度器
 */
void tasks_start_scheduler(void)
{
    vTaskStartScheduler();

    // 正常情况下不会执行到这里
    while(1);
}

/**
 * @brief  暂停所有任务
 */
void tasks_suspend_all(void)
{
    taskENTER_CRITICAL();

    if(xSensorTaskHandle != NULL) vTaskSuspend(xSensorTaskHandle);
    if(xActuatorTaskHandle != NULL) vTaskSuspend(xActuatorTaskHandle);
    if(xControlTaskHandle != NULL) vTaskSuspend(xControlTaskHandle);
    if(xCommTaskHandle != NULL) vTaskSuspend(xCommTaskHandle);
    if(xDisplayTaskHandle != NULL) vTaskSuspend(xDisplayTaskHandle);
    if(xConfigTaskHandle != NULL) vTaskSuspend(xConfigTaskHandle);
    // 注意: 不暂停安全任务

    taskEXIT_CRITICAL();
}

/**
 * @brief  恢复所有任务
 */
void tasks_resume_all(void)
{
    taskENTER_CRITICAL();

    if(xSensorTaskHandle != NULL) vTaskResume(xSensorTaskHandle);
    if(xActuatorTaskHandle != NULL) vTaskResume(xActuatorTaskHandle);
    if(xControlTaskHandle != NULL) vTaskResume(xControlTaskHandle);
    if(xCommTaskHandle != NULL) vTaskResume(xCommTaskHandle);
    if(xDisplayTaskHandle != NULL) vTaskResume(xDisplayTaskHandle);
    if(xConfigTaskHandle != NULL) vTaskResume(xConfigTaskHandle);

    taskEXIT_CRITICAL();
}

/**
 * @brief  根据任务名获取任务句柄
 * @param  task_name 任务名称
 * @retval 任务句柄
 */
TaskHandle_t tasks_get_handle(const char *task_name)
{
    if(task_name == NULL) return NULL;

    if(strcmp(task_name, "Safety") == 0) return xSafetyTaskHandle;
    if(strcmp(task_name, "Control") == 0) return xControlTaskHandle;
    if(strcmp(task_name, "Sensor") == 0) return xSensorTaskHandle;
    if(strcmp(task_name, "Actuator") == 0) return xActuatorTaskHandle;
    if(strcmp(task_name, "Comm") == 0) return xCommTaskHandle;
    if(strcmp(task_name, "Display") == 0) return xDisplayTaskHandle;
    if(strcmp(task_name, "Config") == 0) return xConfigTaskHandle;

    return NULL;
}

/**
 * @brief  获取任务运行统计信息 (用于调试)
 * @param  task_id 任务ID (0-6)
 * @retval 任务统计信息指针
 */
const task_stats_t* tasks_get_stats(uint8_t task_id)
{
    if(task_id >= 7) return NULL;
    return &g_task_stats[task_id];
}

/**
 * @brief  更新任务统计信息 (内部函数)
 * @param  task_id 任务ID
 * @param  run_time 运行时间 (ms)
 */
static void update_task_stats(uint8_t task_id, uint32_t run_time)
{
    if(task_id >= 7) return;

    g_task_stats[task_id].run_count++;
    g_task_stats[task_id].total_run_time += run_time;

    if(run_time > g_task_stats[task_id].max_run_time) {
        g_task_stats[task_id].max_run_time = run_time;
    }

    // 检查任务健康状态 (运行时间异常检测)
    if(run_time > 1000) { // 超过1秒认为异常
        g_task_stats[task_id].task_healthy = false;
    }
}

// FreeRTOS钩子函数 - 任务切换时调用
void vApplicationTaskSwitchHook(void)
{
    static uint32_t last_time = 0;
    uint32_t current_time = xTaskGetTickCount();

    if(last_time > 0) {
        uint32_t run_time = current_time - last_time;
        TaskHandle_t current_task = xTaskGetCurrentTaskHandle();

        // 根据任务句柄确定任务ID并更新统计
        if(current_task == xSafetyTaskHandle) update_task_stats(0, run_time);
        else if(current_task == xControlTaskHandle) update_task_stats(1, run_time);
        else if(current_task == xSensorTaskHandle) update_task_stats(2, run_time);
        else if(current_task == xActuatorTaskHandle) update_task_stats(3, run_time);
        else if(current_task == xCommTaskHandle) update_task_stats(4, run_time);
        else if(current_task == xDisplayTaskHandle) update_task_stats(5, run_time);
        else if(current_task == xConfigTaskHandle) update_task_stats(6, run_time);
    }

    last_time = current_time;
}

// FreeRTOS钩子函数 - 栈溢出检测
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // 栈溢出处理 - 重启系统或进入安全模式
    // 这里简单处理为死循环，实际项目中应该记录错误并重启
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();
    while(1);
}

// FreeRTOS钩子函数 - 空闲任务钩子
void vApplicationIdleHook(void)
{
    // 空闲时的处理 - 可以进入低功耗模式
    // 这里暂时不做处理
}
```

这些中间件模块提供了完整的滤波、PID控制和任务管理功能，是6周压缩版本的核心支持组件。所有模块都经过简化但保持了必要的功能完整性。