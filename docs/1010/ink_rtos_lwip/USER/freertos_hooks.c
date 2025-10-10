/**
 ******************************************************************************
 * @file    freertos_hooks.c
 * @brief   FreeRTOS Hook Functions and Runtime Statistics Implementation
 * @note    This file implements the required hook functions for FreeRTOS
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"
#include "gd32f4xx.h"

/* Private variables ---------------------------------------------------------*/
static uint32_t ulRuntimeCounter = 0;

/**
 * @brief  Configure timer for runtime statistics
 * @note   This function configures TIM5 to provide a high resolution time base
 *         for runtime statistics. TIM5 runs at 1MHz (1us resolution)
 * @retval None
 */
void vConfigureTimerForRunTimeStats(void)
{
    /* This function is called when the scheduler starts.
     * We'll use TIM5 as a free-running counter for runtime statistics.
     * TIM5 is a 32-bit timer, suitable for long runtime measurements.
     */

    /* For now, we'll use a simple counter incremented in SysTick.
     * If you need more precision, configure TIM5 here:
     *
     * 1. Enable TIM5 clock
     * 2. Configure TIM5 to run at 1MHz (1us tick)
     * 3. Start TIM5 in free-running mode
     */

    ulRuntimeCounter = 0;
}

/**
 * @brief  Get runtime counter value
 * @note   Returns the current value of the runtime statistics counter
 * @retval Runtime counter value
 */
uint32_t ulGetRunTimeCounterValue(void)
{
    return ulRuntimeCounter;
}

/**
 * @brief  Increment runtime counter (called from SysTick or high-frequency timer)
 * @note   This should be called from a high-frequency timer interrupt
 *         For better precision, call this from a timer running at 10-100x
 *         the FreeRTOS tick rate
 * @retval None
 */
void vIncrementRuntimeCounter(void)
{
    ulRuntimeCounter++;
}

/**
 * @brief  Application tick hook
 * @note   This function is called on every tick interrupt when
 *         configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h
 * @retval None
 */
void vApplicationTickHook(void)
{
    /* This function will be called every 1ms (tick rate = 1000Hz)
     * Keep this function short and fast!
     *
     * You can use this for:
     * - Incrementing runtime statistics counter (simple approach)
     * - Monitoring system health
     * - Toggling debug pins
     */

    /* Increment runtime counter every 10 ticks to get ~10kHz sampling rate
     * This provides sufficient resolution for runtime statistics
     */
    static uint8_t tickCount = 0;
    tickCount++;
    if (tickCount >= 10) {
        ulRuntimeCounter++;
        tickCount = 0;
    }
}

/**
 * @brief  Application stack overflow hook
 * @note   This function is called when a stack overflow is detected
 *         configCHECK_FOR_STACK_OVERFLOW must be set to 1 or 2
 * @param  xTask: Handle of the task that overflowed its stack
 * @param  pcTaskName: Name of the task that overflowed its stack
 * @retval None
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /* This function is called if a stack overflow is detected.
     *
     * Stack overflow is a serious error that can cause system instability.
     *
     * Actions you can take:
     * 1. Log the task name for debugging
     * 2. Enter an infinite loop (crash gracefully)
     * 3. Trigger a system reset
     * 4. Light an error LED
     */

    (void)xTask;        /* Unused parameter */
    (void)pcTaskName;   /* Unused parameter */

    /* Disable interrupts and halt */
    taskDISABLE_INTERRUPTS();

    /* Infinite loop - Stack overflow detected! */
    /* You can set a breakpoint here for debugging */
    for (;;) {
        /* Optional: Toggle LED to indicate error */
        /* LED0 = !LED0; */
        /* delay_ms(100); */
    }
}

/**
 * @brief  Application malloc failed hook
 * @note   This function is called when pvPortMalloc() fails
 *         configUSE_MALLOC_FAILED_HOOK must be set to 1
 * @retval None
 */
void vApplicationMallocFailedHook(void)
{
    /* This function is called if a call to pvPortMalloc() fails because
     * there is insufficient free memory available in the FreeRTOS heap.
     *
     * pvPortMalloc() is called internally by FreeRTOS API functions that
     * create tasks, queues, software timers, and semaphores.
     *
     * Actions you can take:
     * 1. Increase configTOTAL_HEAP_SIZE in FreeRTOSConfig.h
     * 2. Check for memory leaks
     * 3. Reduce the number of tasks/queues/timers
     * 4. Reduce stack sizes of tasks
     */

    /* Disable interrupts and halt */
    taskDISABLE_INTERRUPTS();

    /* Infinite loop - Memory allocation failed! */
    /* You can set a breakpoint here for debugging */
    for (;;) {
        /* Optional: Toggle LED to indicate error */
        /* LED0 = !LED0; */
        /* delay_ms(200); */
    }
}

/**
 * @brief  Application idle hook (optional)
 * @note   This function is called on each iteration of the idle task
 *         configUSE_IDLE_HOOK must be set to 1
 * @retval None
 */
#if (configUSE_IDLE_HOOK == 1)
void vApplicationIdleHook(void)
{
    /* This function is called on each cycle of the idle task.
     *
     * The idle task runs when no other task is ready to run.
     *
     * You can use this for:
     * 1. Entering low-power mode
     * 2. Background garbage collection
     * 3. Monitoring CPU usage
     *
     * NOTE: This function MUST NOT call any blocking functions!
     */

    /* Example: Enter sleep mode to save power */
    /* __WFI(); */  /* Wait For Interrupt */
}
#endif

/**
 * @brief  Get idle task memory (for static allocation)
 * @note   Only needed if configSUPPORT_STATIC_ALLOCATION == 1
 */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/**
 * @brief  Get timer task memory (for static allocation)
 * @note   Only needed if configSUPPORT_STATIC_ALLOCATION == 1 && configUSE_TIMERS == 1
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif

/************************ (C) COPYRIGHT WKS *****END OF FILE****/
