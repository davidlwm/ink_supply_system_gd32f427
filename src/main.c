/**
 * @file    main.c
 * @brief   供墨系统控制板卡主程序入口 - 8周v4标准
 * @version V4.0
 * @date    2024-12-27
 * @note    基于GD32F427VGT6，符合8周v4架构标准
 */

/* 项目配置 */
#include "config/project_config.h"

/* 应用层 */
#include "app/app_main.h"
#include "app/app_types.h"

/* 系统层 */
#include "system/board_init.h"
#include "system/system_manager.h"
#include "system/error_handler.h"

/* HAL层 */
#include "hal/gpio_hal.h"
#include "hal/uart_hal.h"
#include "hal/adc_hal.h"
#include "hal/pwm_hal.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* 系统头文件 */
#include "gd32f4xx.h"
#include <stdio.h>
// 内部函数声明
static void system_early_init(void);
static void hardware_init(void);
static app_result_t init_application_system(void);
static void print_system_info(void);

/**
 * @brief  主函数 - 8周v4标准架构
 * @param  None
 * @retval int 程序退出状态
 */
int main(void)
{
    app_result_t result;
    app_config_t app_config;

    // 1. 系统早期初始化
    system_early_init();

    // 2. 硬件层初始化
    hardware_init();

    // 3. 打印系统信息
    print_system_info();

    // 4. 配置应用参数
    app_config.enable_watchdog = APP_WATCHDOG_ENABLE;
    app_config.enable_stack_monitor = APP_STACK_MONITOR;
    app_config.statistics_period_ms = APP_STATISTICS_PERIOD_MS;
    app_config.heartbeat_period_ms = 1000;

    // 5. 初始化应用系统
    result = init_application_system();
    if (result != APP_OK) {
        printf("Application system init failed: %d\r\n", result);
        while(1); // 系统停机
    }

    // 6. 初始化应用主程序
    result = app_main_init(&app_config);
    if (result != APP_OK) {
        printf("App main init failed: %d\r\n", result);
        while(1); // 系统停机
    }

    // 7. 启动应用系统
    result = app_main_start();
    if (result != APP_OK) {
        printf("App main start failed: %d\r\n", result);
        while(1); // 系统停机
    }

    printf("Ink supply system started successfully!\r\n");

    // 8. 启动FreeRTOS调度器
    vTaskStartScheduler();

    // 正常情况下不会执行到这里
    printf("ERROR: FreeRTOS scheduler failed to start!\r\n");
    while(1);

    return 0;
}

/**
 * @brief  系统早期初始化 - 基础硬件配置
 * @param  None
 * @retval None
 */
static void system_early_init(void)
{
    // GD32F427系统时钟配置为200MHz
    SystemInit();

    // 中断优先级分组配置 (4位抢占优先级，0位子优先级)
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    // 系统滴答定时器配置 (1ms)
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while(1); // 配置失败
    }

    // 使能关键外设时钟
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
}

/**
 * @brief  硬件层初始化 - HAL层初始化
 * @param  None
 * @retval None
 */
static void hardware_init(void)
{
    // 初始化GPIO HAL
    gpio_result_t gpio_result = gpio_hal_init();
    if (gpio_result != GPIO_OK) {
        printf("GPIO HAL init failed: %d\r\n", gpio_result);
        while(1);
    }

    // 初始化UART HAL (用于调试输出)
    uart_result_t uart_result = uart_hal_init();
    if (uart_result != UART_OK) {
        printf("UART HAL init failed: %d\r\n", uart_result);
        while(1);
    }

    // 初始化ADC HAL
    adc_result_t adc_result = adc_hal_init();
    if (adc_result != ADC_OK) {
        printf("ADC HAL init failed: %d\r\n", adc_result);
        while(1);
    }

    // 初始化PWM HAL
    pwm_result_t pwm_result = pwm_hal_init();
    if (pwm_result != PWM_OK) {
        printf("PWM HAL init failed: %d\r\n", pwm_result);
        while(1);
    }

    printf("Hardware initialization completed.\r\n");
}

/**
 * @brief  初始化应用系统 - 各个子系统初始化
 * @param  None
 * @retval app_result_t 初始化结果
 */
static app_result_t init_application_system(void)
{
    printf("Initializing application systems...\r\n");

    // 这里可以添加其他应用子系统的初始化
    // 例如：传感器校准数据加载、配置参数读取等

    printf("Application systems initialized.\r\n");
    return APP_OK;
}

/**
 * @brief  打印系统信息
 * @param  None
 * @retval None
 */
static void print_system_info(void)
{
    printf("\r\n");
    printf("==============================================\r\n");
    printf(" Ink Supply System Control Board\r\n");
    printf(" Version: %s\r\n", APP_VERSION_STRING);
    printf(" Build Date: %s %s\r\n", APP_BUILD_DATE, APP_BUILD_TIME);
    printf(" MCU: GD32F427VGT6 @ %dMHz\r\n", SystemCoreClock/1000000);
    printf(" Architecture: 8-Week V4 Standard\r\n");
    printf("==============================================\r\n");
    printf("\r\n");
}