/**
 ******************************************************************************
 * @file    test2.h
 * @brief   FreeRTOS Integration Test Header File
 ******************************************************************************
 */

#ifndef __TEST2_H
#define __TEST2_H

#include "FreeRTOS.h"
#include "task.h"

/* Function prototypes */
BaseType_t FreeRTOS_Test_Init(void);
void Test_RuntimeStats(void);
void Test_HookFunctions(void);

/* Task prototypes */
void vTestTask1(void *pvParameters);
void vTestTask2(void *pvParameters);
void vTestTask3(void *pvParameters);
void vMonitorTask(void *pvParameters);

#endif /* __TEST2_H */

/************************ (C) COPYRIGHT WKS *****END OF FILE****/
