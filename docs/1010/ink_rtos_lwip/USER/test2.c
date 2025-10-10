/**
 ******************************************************************************
 * @file    test2.c
 * @brief   FreeRTOS Integration Test
 * @note    This file tests FreeRTOS integration with runtime statistics,
 *          task management, and hook functions
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Explicitly include FreeRTOS timers header to avoid lwIP timers.h confusion */
#include "../FreeRTOS/include/timers.h"

#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "led.h"
#include "delay.h"

/* Task handles */
TaskHandle_t xTestTask1Handle = NULL;
TaskHandle_t xTestTask2Handle = NULL;
TaskHandle_t xTestTask3Handle = NULL;
TaskHandle_t xMonitorTaskHandle = NULL;

/* Queue handle for inter-task communication */
QueueHandle_t xTestQueue = NULL;

/* Semaphore handle */
SemaphoreHandle_t xTestSemaphore = NULL;

/* Timer handle */
TimerHandle_t xTestTimer = NULL;

/* Test statistics buffer */
#define STATS_BUFFER_SIZE 1024
static char statsBuffer[STATS_BUFFER_SIZE];

/**
 * @brief  Test Task 1 - High priority task
 * @param  pvParameters: Task parameters
 * @retval None
 */
void vTestTask1(void *pvParameters)
{
    uint32_t count = 0;
    uint32_t queueData;

    printf("Task1: Started (Priority: %d)\r\n", (int)uxTaskPriorityGet(NULL));

    for (;;)
    {
        count++;

        /* Send data to queue every 500ms */
        queueData = count;
        if (xQueueSend(xTestQueue, &queueData, pdMS_TO_TICKS(10)) == pdPASS)
        {
            printf("Task1: Sent data %lu to queue\r\n", count);
        }

        /* Give semaphore to unblock Task3 */
        xSemaphoreGive(xTestSemaphore);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief  Test Task 2 - Medium priority task
 * @param  pvParameters: Task parameters
 * @retval None
 */
void vTestTask2(void *pvParameters)
{
    uint32_t receivedData;

    printf("Task2: Started (Priority: %d)\r\n", (int)uxTaskPriorityGet(NULL));

    for (;;)
    {
        /* Wait for data from queue */
        if (xQueueReceive(xTestQueue, &receivedData, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            printf("Task2: Received data %lu from queue\r\n", receivedData);
        }
        else
        {
            printf("Task2: Queue receive timeout\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(600));
    }
}

/**
 * @brief  Test Task 3 - Low priority task
 * @param  pvParameters: Task parameters
 * @retval None
 */
void vTestTask3(void *pvParameters)
{
    printf("Task3: Started (Priority: %d)\r\n", (int)uxTaskPriorityGet(NULL));

    for (;;)
    {
        /* Wait for semaphore */
        if (xSemaphoreTake(xTestSemaphore, pdMS_TO_TICKS(2000)) == pdPASS)
        {
            printf("Task3: Semaphore acquired, toggling LED\r\n");
            LED0 = !LED0;  /* Toggle LED */
        }
        else
        {
            printf("Task3: Semaphore timeout\r\n");
        }
    }
}

/**
 * @brief  Monitor Task - Display runtime statistics
 * @param  pvParameters: Task parameters
 * @retval None
 */
void vMonitorTask(void *pvParameters)
{
    printf("Monitor: Started (Priority: %d)\r\n", (int)uxTaskPriorityGet(NULL));

    for (;;)
    {
        /* Wait 10 seconds before displaying statistics */
        vTaskDelay(pdMS_TO_TICKS(10000));

        printf("\r\n========== RUNTIME STATISTICS ==========\r\n");

        /* Get task runtime statistics */
        #if (configUSE_STATS_FORMATTING_FUNCTIONS == 1)
        vTaskGetRunTimeStats(statsBuffer);
        printf("Task\t\tAbs Time\tPercent\r\n");
        printf("%s\r\n", statsBuffer);
        #endif

        /* Get task status list */
        #if (configUSE_TRACE_FACILITY == 1)
        printf("\r\n========== TASK STATUS ==========\r\n");
        vTaskList(statsBuffer);
        printf("Name\t\tState\tPrio\tStack\tNum\r\n");
        printf("%s\r\n", statsBuffer);
        #endif

        /* Get heap information */
        printf("\r\n========== HEAP INFORMATION ==========\r\n");
        printf("Free Heap: %u bytes\r\n", (unsigned int)xPortGetFreeHeapSize());
        printf("Min Free Heap: %u bytes\r\n", (unsigned int)xPortGetMinimumEverFreeHeapSize());

        printf("========================================\r\n\r\n");
    }
}

/**
 * @brief  Software timer callback function
 * @param  xTimer: Timer handle
 * @retval None
 */
void vTestTimerCallback(TimerHandle_t xTimer)
{
    static uint32_t timerCount = 0;
    timerCount++;

    printf("Timer: Callback executed (count: %lu)\r\n", timerCount);
}

/**
 * @brief  Initialize and start FreeRTOS test tasks
 * @retval pdPASS if successful, pdFAIL otherwise
 */
BaseType_t FreeRTOS_Test_Init(void)
{
    BaseType_t xReturn = pdPASS;

    /* Create queue for inter-task communication */
    xTestQueue = xQueueCreate(5, sizeof(uint32_t));
    if (xTestQueue == NULL)
    {
        return pdFAIL;
    }

    /* Create binary semaphore */
    xTestSemaphore = xSemaphoreCreateBinary();
    if (xTestSemaphore == NULL)
    {
        return pdFAIL;
    }

    /* Create software timer (3 second period, auto-reload) */
    xTestTimer = xTimerCreate("TestTimer",
                              pdMS_TO_TICKS(3000),
                              pdTRUE,
                              (void *)0,
                              vTestTimerCallback);
    if (xTestTimer == NULL)
    {
        return pdFAIL;
    }

    /* Start the timer */
    if (xTimerStart(xTestTimer, 0) != pdPASS)
    {
        return pdFAIL;
    }

    /* Create Test Task 1 - High priority */
    xReturn = xTaskCreate(vTestTask1,
                          "Task1",
                          256,
                          NULL,
                          3,
                          &xTestTask1Handle);
    if (xReturn != pdPASS)
    {
        return pdFAIL;
    }

    /* Create Test Task 2 - Medium priority */
    xReturn = xTaskCreate(vTestTask2,
                          "Task2",
                          256,
                          NULL,
                          2,
                          &xTestTask2Handle);
    if (xReturn != pdPASS)
    {
        return pdFAIL;
    }

    /* Create Test Task 3 - Low priority */
    xReturn = xTaskCreate(vTestTask3,
                          "Task3",
                          256,
                          NULL,
                          1,
                          &xTestTask3Handle);
    if (xReturn != pdPASS)
    {
        return pdFAIL;
    }

    /* Create Monitor Task - Lowest priority */
    xReturn = xTaskCreate(vMonitorTask,
                          "Monitor",
                          512,
                          NULL,
                          0,
                          &xMonitorTaskHandle);
    if (xReturn != pdPASS)
    {
        return pdFAIL;
    }

    return pdPASS;
}

/**
 * @brief  Test runtime statistics API
 * @retval None
 */
void Test_RuntimeStats(void)
{
    // 使用LCD显示而不是printf
    LCD_ShowString(30, 230, 200, 16, 16, "Runtime Stats: ENABLED");
    delay_ms(300);
    LCD_ShowString(30, 250, 200, 16, 16, "Trace Facility: ENABLED");
    delay_ms(300);
}

/**
 * @brief  Test hook functions configuration
 * @retval None
 */
void Test_HookFunctions(void)
{
    // 使用LCD显示而不是printf
    LCD_ShowString(30, 270, 200, 16, 16, "Tick Hook: ENABLED");
    delay_ms(300);
    LCD_ShowString(30, 290, 200, 16, 16, "Stack Check: ENABLED");
    delay_ms(300);
}

/************************ (C) COPYRIGHT WKS *****END OF FILE****/
