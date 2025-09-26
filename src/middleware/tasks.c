/**
 * @file    tasks.c
 * @brief   FreeRTOS任务管理实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "middleware/tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include <string.h>

// 任务信息数组
static task_info_t task_registry[MAX_TASKS];
static uint8_t task_count = 0;
static SemaphoreHandle_t task_mutex = NULL;

// 系统统计信息
static system_stats_t system_statistics = {0};

/**
 * @brief  任务管理器初始化
 * @param  None
 * @retval task_result_t 初始化结果
 */
task_result_t task_manager_init(void)
{
    // 创建互斥锁
    task_mutex = xSemaphoreCreateMutex();
    if (task_mutex == NULL) {
        return TASK_ERROR_MEMORY_ALLOCATION;
    }

    // 初始化任务注册表
    memset(task_registry, 0, sizeof(task_registry));
    task_count = 0;

    // 初始化统计信息
    memset(&system_statistics, 0, sizeof(system_statistics));

    return TASK_SUCCESS;
}

/**
 * @brief  注册任务
 * @param  name: 任务名称
 * @param  handle: 任务句柄
 * @param  priority: 任务优先级
 * @param  stack_size: 栈大小
 * @param  period: 任务周期 (毫秒)
 * @retval task_result_t 注册结果
 */
task_result_t task_register(const char* name, TaskHandle_t handle,
                           uint8_t priority, uint16_t stack_size, uint32_t period)
{
    if (name == NULL || handle == NULL || task_count >= MAX_TASKS) {
        return TASK_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(task_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return TASK_ERROR_TIMEOUT;
    }

    // 填充任务信息
    strncpy(task_registry[task_count].name, name, TASK_NAME_MAX_LEN - 1);
    task_registry[task_count].name[TASK_NAME_MAX_LEN - 1] = '\0';
    task_registry[task_count].handle = handle;
    task_registry[task_count].priority = priority;
    task_registry[task_count].stack_size = stack_size;
    task_registry[task_count].period = period;
    task_registry[task_count].created_time = xTaskGetTickCount();
    task_registry[task_count].state = TASK_STATE_RUNNING;
    task_registry[task_count].run_count = 0;
    task_registry[task_count].cpu_usage = 0.0f;

    task_count++;

    xSemaphoreGive(task_mutex);

    return TASK_SUCCESS;
}

/**
 * @brief  获取任务信息
 * @param  task_id: 任务ID
 * @param  info: 任务信息指针
 * @retval task_result_t 获取结果
 */
task_result_t task_get_info(uint8_t task_id, task_info_t* info)
{
    if (info == NULL || task_id >= task_count) {
        return TASK_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(task_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return TASK_ERROR_TIMEOUT;
    }

    memcpy(info, &task_registry[task_id], sizeof(task_info_t));

    xSemaphoreGive(task_mutex);

    return TASK_SUCCESS;
}

/**
 * @brief  获取所有任务信息
 * @param  info_array: 任务信息数组
 * @param  max_count: 最大数量
 * @param  actual_count: 实际数量指针
 * @retval task_result_t 获取结果
 */
task_result_t task_get_all_info(task_info_t* info_array, uint8_t max_count, uint8_t* actual_count)
{
    if (info_array == NULL || actual_count == NULL) {
        return TASK_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(task_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return TASK_ERROR_TIMEOUT;
    }

    uint8_t copy_count = (task_count < max_count) ? task_count : max_count;
    memcpy(info_array, task_registry, copy_count * sizeof(task_info_t));
    *actual_count = copy_count;

    xSemaphoreGive(task_mutex);

    return TASK_SUCCESS;
}

/**
 * @brief  更新任务统计
 * @param  task_id: 任务ID
 * @retval task_result_t 更新结果
 */
task_result_t task_update_stats(uint8_t task_id)
{
    if (task_id >= task_count) {
        return TASK_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(task_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return TASK_ERROR_TIMEOUT;
    }

    task_registry[task_id].run_count++;
    task_registry[task_id].last_run_time = xTaskGetTickCount();

    // 获取任务状态
    eTaskState state = eTaskGetState(task_registry[task_id].handle);
    switch (state) {
        case eRunning:
            task_registry[task_id].state = TASK_STATE_RUNNING;
            break;
        case eReady:
            task_registry[task_id].state = TASK_STATE_READY;
            break;
        case eBlocked:
            task_registry[task_id].state = TASK_STATE_BLOCKED;
            break;
        case eSuspended:
            task_registry[task_id].state = TASK_STATE_SUSPENDED;
            break;
        case eDeleted:
            task_registry[task_id].state = TASK_STATE_DELETED;
            break;
        default:
            task_registry[task_id].state = TASK_STATE_UNKNOWN;
            break;
    }

    xSemaphoreGive(task_mutex);

    return TASK_SUCCESS;
}

/**
 * @brief  获取系统统计信息
 * @param  stats: 统计信息指针
 * @retval task_result_t 获取结果
 */
task_result_t task_get_system_stats(system_stats_t* stats)
{
    if (stats == NULL) {
        return TASK_ERROR_INVALID_PARAMETER;
    }

    // 更新系统统计
    system_statistics.total_tasks = task_count;
    system_statistics.heap_free = xPortGetFreeHeapSize();
    system_statistics.heap_min_free = xPortGetMinimumEverFreeHeapSize();
    system_statistics.uptime = xTaskGetTickCount();

    // 计算任务状态统计
    system_statistics.running_tasks = 0;
    system_statistics.ready_tasks = 0;
    system_statistics.blocked_tasks = 0;
    system_statistics.suspended_tasks = 0;

    for (uint8_t i = 0; i < task_count; i++) {
        switch (task_registry[i].state) {
            case TASK_STATE_RUNNING:
                system_statistics.running_tasks++;
                break;
            case TASK_STATE_READY:
                system_statistics.ready_tasks++;
                break;
            case TASK_STATE_BLOCKED:
                system_statistics.blocked_tasks++;
                break;
            case TASK_STATE_SUSPENDED:
                system_statistics.suspended_tasks++;
                break;
            default:
                break;
        }
    }

    memcpy(stats, &system_statistics, sizeof(system_stats_t));

    return TASK_SUCCESS;
}

/**
 * @brief  任务看门狗检查
 * @param  None
 * @retval task_result_t 检查结果
 */
task_result_t task_watchdog_check(void)
{
    uint32_t current_time = xTaskGetTickCount();
    bool watchdog_triggered = false;

    if (xSemaphoreTake(task_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return TASK_ERROR_TIMEOUT;
    }

    for (uint8_t i = 0; i < task_count; i++) {
        if (task_registry[i].period > 0) { // 只检查周期性任务
            uint32_t elapsed = current_time - task_registry[i].last_run_time;
            uint32_t timeout = task_registry[i].period * TASK_WATCHDOG_MULTIPLIER;

            if (elapsed > pdMS_TO_TICKS(timeout)) {
                task_registry[i].watchdog_violations++;
                watchdog_triggered = true;

                // 可以在这里添加看门狗处理逻辑
                // 例如：重启任务、记录日志等
            }
        }
    }

    xSemaphoreGive(task_mutex);

    return watchdog_triggered ? TASK_ERROR_WATCHDOG : TASK_SUCCESS;
}

/**
 * @brief  挂起任务
 * @param  task_id: 任务ID
 * @retval task_result_t 挂起结果
 */
task_result_t task_suspend(uint8_t task_id)
{
    if (task_id >= task_count) {
        return TASK_ERROR_INVALID_PARAMETER;
    }

    vTaskSuspend(task_registry[task_id].handle);
    task_registry[task_id].state = TASK_STATE_SUSPENDED;

    return TASK_SUCCESS;
}

/**
 * @brief  恢复任务
 * @param  task_id: 任务ID
 * @retval task_result_t 恢复结果
 */
task_result_t task_resume(uint8_t task_id)
{
    if (task_id >= task_count) {
        return TASK_ERROR_INVALID_PARAMETER;
    }

    vTaskResume(task_registry[task_id].handle);
    task_registry[task_id].state = TASK_STATE_READY;

    return TASK_SUCCESS;
}

/**
 * @brief  获取任务数量
 * @param  None
 * @retval uint8_t 任务数量
 */
uint8_t task_get_count(void)
{
    return task_count;
}

/**
 * @brief  根据名称查找任务ID
 * @param  name: 任务名称
 * @retval int 任务ID，失败返回-1
 */
int task_find_by_name(const char* name)
{
    if (name == NULL) {
        return -1;
    }

    for (uint8_t i = 0; i < task_count; i++) {
        if (strcmp(task_registry[i].name, name) == 0) {
            return i;
        }
    }

    return -1;
}

/**
 * @brief  获取任务句柄
 * @param  task_id: 任务ID
 * @retval TaskHandle_t 任务句柄
 */
TaskHandle_t task_get_handle(uint8_t task_id)
{
    if (task_id >= task_count) {
        return NULL;
    }

    return task_registry[task_id].handle;
}

/**
 * @brief  任务系统自检
 * @param  None
 * @retval bool 自检结果
 */
bool task_system_self_test(void)
{
    // 检查任务管理器是否初始化
    if (task_mutex == NULL) {
        return false;
    }

    // 检查是否有任务注册
    if (task_count == 0) {
        return false;
    }

    // 检查堆内存
    if (xPortGetFreeHeapSize() < MIN_FREE_HEAP_SIZE) {
        return false;
    }

    // 检查看门狗状态
    if (task_watchdog_check() == TASK_ERROR_WATCHDOG) {
        return false;
    }

    return true;
}

/**
 * @brief  重置任务统计
 * @param  task_id: 任务ID
 * @retval task_result_t 重置结果
 */
task_result_t task_reset_stats(uint8_t task_id)
{
    if (task_id >= task_count) {
        return TASK_ERROR_INVALID_PARAMETER;
    }

    if (xSemaphoreTake(task_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return TASK_ERROR_TIMEOUT;
    }

    task_registry[task_id].run_count = 0;
    task_registry[task_id].cpu_usage = 0.0f;
    task_registry[task_id].watchdog_violations = 0;

    xSemaphoreGive(task_mutex);

    return TASK_SUCCESS;
}