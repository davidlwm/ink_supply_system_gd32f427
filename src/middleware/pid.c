/**
 * @file    pid.c
 * @brief   PID算法实现 - 简化版
 * @version V4.0
 * @date    2025-09-27
 */

#include "middleware/pid.h"
#include <math.h>
#include <string.h>

// PID控制器实例数组
static pid_controller_t pid_controllers[MAX_PID_CONTROLLERS];
static uint8_t pid_controller_count = 0;

/**
 * @brief  PID控制器初始化
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @param  dt: 采样时间 (秒)
 * @retval pid_result_t 初始化结果
 */
pid_result_t pid_init(pid_controller_t* pid, float kp, float ki, float kd, float dt)
{
    if (pid == NULL || dt <= 0) {
        return PID_ERROR_INVALID_PARAMETER;
    }

    // 清零结构体
    memset(pid, 0, sizeof(pid_controller_t));

    // 设置PID参数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;

    // 设置默认限制
    pid->output_min = DEFAULT_OUTPUT_MIN;
    pid->output_max = DEFAULT_OUTPUT_MAX;
    pid->integral_min = DEFAULT_INTEGRAL_MIN;
    pid->integral_max = DEFAULT_INTEGRAL_MAX;

    // 设置默认设定点
    pid->setpoint = 0.0f;

    // 初始化状态
    pid->initialized = true;
    pid->enabled = true;

    return PID_SUCCESS;
}

/**
 * @brief  PID控制器计算
 * @param  pid: PID控制器指针
 * @param  feedback: 反馈值
 * @retval float 控制输出
 */
float pid_compute(pid_controller_t* pid, float feedback)
{
    if (pid == NULL || !pid->initialized || !pid->enabled) {
        return 0.0f;
    }

    // 计算误差
    float error = pid->setpoint - feedback;

    // 比例项
    float proportional = pid->kp * error;

    // 积分项
    pid->integral += error * pid->dt;

    // 积分限幅
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }

    float integral_term = pid->ki * pid->integral;

    // 微分项
    float derivative = (error - pid->prev_error) / pid->dt;
    float derivative_term = pid->kd * derivative;

    // 计算总输出
    float output = proportional + integral_term + derivative_term;

    // 输出限幅
    if (output > pid->output_max) {
        output = pid->output_max;
        // 防止积分饱和
        if (pid->ki != 0) {
            pid->integral = (output - proportional - derivative_term) / pid->ki;
        }
    } else if (output < pid->output_min) {
        output = pid->output_min;
        // 防止积分饱和
        if (pid->ki != 0) {
            pid->integral = (output - proportional - derivative_term) / pid->ki;
        }
    }

    // 更新历史值
    pid->prev_error = error;
    pid->prev_feedback = feedback;
    pid->output = output;

    // 更新统计
    pid->compute_count++;

    return output;
}

/**
 * @brief  设置PID参数
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval pid_result_t 设置结果
 */
pid_result_t pid_set_parameters(pid_controller_t* pid, float kp, float ki, float kd)
{
    if (pid == NULL || !pid->initialized) {
        return PID_ERROR_INVALID_PARAMETER;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    return PID_SUCCESS;
}

/**
 * @brief  设置输出限制
 * @param  pid: PID控制器指针
 * @param  min: 最小输出
 * @param  max: 最大输出
 * @retval pid_result_t 设置结果
 */
pid_result_t pid_set_output_limits(pid_controller_t* pid, float min, float max)
{
    if (pid == NULL || !pid->initialized || min >= max) {
        return PID_ERROR_INVALID_PARAMETER;
    }

    pid->output_min = min;
    pid->output_max = max;

    // 如果当前输出超出新限制，调整积分项
    if (pid->output > max) {
        pid->output = max;
    } else if (pid->output < min) {
        pid->output = min;
    }

    return PID_SUCCESS;
}

/**
 * @brief  设置积分限制
 * @param  pid: PID控制器指针
 * @param  min: 最小积分
 * @param  max: 最大积分
 * @retval pid_result_t 设置结果
 */
pid_result_t pid_set_integral_limits(pid_controller_t* pid, float min, float max)
{
    if (pid == NULL || !pid->initialized || min >= max) {
        return PID_ERROR_INVALID_PARAMETER;
    }

    pid->integral_min = min;
    pid->integral_max = max;

    // 如果当前积分超出新限制，进行限幅
    if (pid->integral > max) {
        pid->integral = max;
    } else if (pid->integral < min) {
        pid->integral = min;
    }

    return PID_SUCCESS;
}

/**
 * @brief  设置设定点
 * @param  pid: PID控制器指针
 * @param  setpoint: 设定点
 * @retval pid_result_t 设置结果
 */
pid_result_t pid_set_setpoint(pid_controller_t* pid, float setpoint)
{
    if (pid == NULL || !pid->initialized) {
        return PID_ERROR_INVALID_PARAMETER;
    }

    pid->setpoint = setpoint;

    return PID_SUCCESS;
}

/**
 * @brief  重置PID控制器
 * @param  pid: PID控制器指针
 * @retval pid_result_t 重置结果
 */
pid_result_t pid_reset(pid_controller_t* pid)
{
    if (pid == NULL || !pid->initialized) {
        return PID_ERROR_INVALID_PARAMETER;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_feedback = 0.0f;
    pid->output = 0.0f;
    pid->compute_count = 0;

    return PID_SUCCESS;
}

/**
 * @brief  启用/禁用PID控制器
 * @param  pid: PID控制器指针
 * @param  enable: 启用标志
 * @retval pid_result_t 设置结果
 */
pid_result_t pid_enable(pid_controller_t* pid, bool enable)
{
    if (pid == NULL || !pid->initialized) {
        return PID_ERROR_INVALID_PARAMETER;
    }

    pid->enabled = enable;

    // 如果禁用，重置输出
    if (!enable) {
        pid->output = 0.0f;
    }

    return PID_SUCCESS;
}

/**
 * @brief  创建PID控制器
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @param  dt: 采样时间
 * @retval int PID控制器ID，失败返回-1
 */
int pid_create(float kp, float ki, float kd, float dt)
{
    if (pid_controller_count >= MAX_PID_CONTROLLERS) {
        return -1;
    }

    int pid_id = pid_controller_count;

    if (pid_init(&pid_controllers[pid_id], kp, ki, kd, dt) == PID_SUCCESS) {
        pid_controller_count++;
        return pid_id;
    }

    return -1;
}

/**
 * @brief  获取PID控制器指针
 * @param  pid_id: PID控制器ID
 * @retval pid_controller_t* PID控制器指针
 */
pid_controller_t* pid_get_controller(int pid_id)
{
    if (pid_id < 0 || pid_id >= pid_controller_count) {
        return NULL;
    }

    return &pid_controllers[pid_id];
}

/**
 * @brief  PID控制器计算 (通过ID)
 * @param  pid_id: PID控制器ID
 * @param  feedback: 反馈值
 * @retval float 控制输出
 */
float pid_compute_by_id(int pid_id, float feedback)
{
    pid_controller_t* pid = pid_get_controller(pid_id);
    if (pid == NULL) {
        return 0.0f;
    }

    return pid_compute(pid, feedback);
}

/**
 * @brief  设置PID设定点 (通过ID)
 * @param  pid_id: PID控制器ID
 * @param  setpoint: 设定点
 * @retval pid_result_t 设置结果
 */
pid_result_t pid_set_setpoint_by_id(int pid_id, float setpoint)
{
    pid_controller_t* pid = pid_get_controller(pid_id);
    if (pid == NULL) {
        return PID_ERROR_INVALID_PARAMETER;
    }

    return pid_set_setpoint(pid, setpoint);
}

/**
 * @brief  获取PID统计信息
 * @param  stats: 统计信息指针
 * @retval None
 */
void pid_get_statistics(pid_statistics_t* stats)
{
    if (stats == NULL) {
        return;
    }

    stats->controller_count = pid_controller_count;
    stats->total_computes = 0;

    for (int i = 0; i < pid_controller_count; i++) {
        stats->total_computes += pid_controllers[i].compute_count;
    }
}

/**
 * @brief  PID自检
 * @param  None
 * @retval bool 自检结果
 */
bool pid_self_test(void)
{
    // 创建测试PID控制器
    int test_pid = pid_create(1.0f, 0.1f, 0.01f, 0.01f);
    if (test_pid < 0) {
        return false;
    }

    pid_controller_t* pid = pid_get_controller(test_pid);
    if (pid == NULL) {
        return false;
    }

    // 设置测试参数
    pid_set_setpoint(pid, 100.0f);
    pid_set_output_limits(pid, 0.0f, 255.0f);

    // 进行几次计算测试
    float feedback = 0.0f;
    for (int i = 0; i < 10; i++) {
        float output = pid_compute(pid, feedback);
        feedback += output * 0.01f; // 简单的系统模拟

        // 检查输出是否在合理范围内
        if (output < 0.0f || output > 255.0f) {
            return false;
        }
    }

    return true;
}

/* 兼容性接口实现 - 为应用层提供简化接口 */

/**
 * @brief  PID控制器初始化 (兼容性接口)
 * @param  pid: PID控制器指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval mw_result_t 初始化结果
 */
mw_result_t pid_controller_init(pid_controller_t* pid, float kp, float ki, float kd)
{
    pid_result_t result = pid_init(pid, kp, ki, kd, 0.01f); // 默认采样时间10ms
    return (result == PID_SUCCESS) ? MW_OK : MW_ERROR;
}

/**
 * @brief  PID控制器更新 (兼容性接口)
 * @param  pid: PID控制器指针
 * @param  error: 误差值
 * @param  dt: 采样时间间隔(ms)
 * @retval float 控制输出
 */
float pid_controller_update(pid_controller_t* pid, float error, float dt)
{
    if (pid == NULL) {
        return 0.0f;
    }

    // 将误差转换为设定点和反馈值
    float setpoint = pid->setpoint;
    float feedback = setpoint - error;

    return pid_compute(pid, feedback);
}

/**
 * @brief  PID系统初始化
 * @param  None
 * @retval mw_result_t 初始化结果
 */
mw_result_t pid_system_init(void)
{
    // 初始化PID控制器数组
    pid_controller_count = 0;
    memset(pid_controllers, 0, sizeof(pid_controllers));

    return MW_OK;
}