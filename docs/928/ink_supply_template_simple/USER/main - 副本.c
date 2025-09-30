/**
 * @file    main.c
 * @brief   墨水供应系统 - 基础模板工程 (基于IIC实验)
 * @author  Template based on WKS GD32F427VET6 IIC实验
 * @date    2024
 */

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "usmart.h"
#include "24cxx.h"

/**
 * @brief   系统初始化
 * @param   无
 * @retval  无
 */
void system_init(void)
{
    HAL_Init();                         /* 初始化HAL库 */
    GD32_Clock_Init(336, 25, 2, 7);    /* 设置时钟,168Mhz */
    delay_init(168);                    /* 初始化延时函数 */
    uart_init(115200);                  /* 初始化USART */
    usmart_dev.init(84);                /* 初始化USMART */
    LED_Init();                         /* 初始化LED */
    KEY_Init();                         /* 初始化KEY */
    LCD_Init();                         /* 初始化LCD */
    AT24CXX_Init();                     /* 初始化IIC和24C02 */
}

/**
 * @brief   显示系统信息
 * @param   无
 * @retval  无
 */
void display_system_info(void)
{
    POINT_COLOR = RED;
    LCD_ShowString(30, 50, 200, 16, 16, "Ink Supply System");
    LCD_ShowString(30, 70, 200, 16, 16, "GD32F427 Template");
    LCD_ShowString(30, 90, 200, 16, 16, "WKS SMART");
    LCD_ShowString(30, 110, 200, 16, 16, "KEY_UP:Test  KEY0:Demo");

    /* 检测24C02 */
    while (AT24CXX_Check())
    {
        LCD_ShowString(30, 130, 200, 16, 16, "24C02 Check Failed!");
        delay_ms(500);
        LCD_ShowString(30, 130, 200, 16, 16, "Please Check!      ");
        delay_ms(500);
        LED0 = !LED0;
    }
    LCD_ShowString(30, 130, 200, 16, 16, "24C02 Ready!");
    POINT_COLOR = BLUE;
}

/**
 * @brief   主函数
 * @param   无
 * @retval  无
 */
int main(void)
{
    u8 key;
    u16 led_counter = 0;
    u32 system_counter = 0;

    /* 系统初始化 */
    system_init();

    /* 显示系统信息 */
    display_system_info();

    printf("Ink Supply System Template Ready!\r\n");
    printf("System Clock: 168MHz\r\n");
    printf("Hardware: GD32F427VET6\r\n");
    printf("Template: Based on IIC Experiment\r\n");

    /* 主循环 */
    while (1)
    {
        key = KEY_Scan(0);

        /* 按键处理 */
        if (key == WKUP_PRES)
        {
            LCD_Fill(0, 170, 319, 319, WHITE);
            LCD_ShowString(30, 170, 200, 16, 16, "WKUP Key Pressed!");
            LCD_ShowString(30, 190, 200, 16, 16, "System Test Mode");
            printf("WKUP Key: System test mode activated\r\n");

            /* 这里可以添加系统测试代码 */
            delay_ms(1000);
        }

        if (key == KEY0_PRES)
        {
            LCD_Fill(0, 170, 319, 319, WHITE);
            LCD_ShowString(30, 170, 200, 16, 16, "KEY0 Pressed!");
            LCD_ShowString(30, 190, 200, 16, 16, "Demo Function");
            printf("KEY0 Key: Demo function triggered\r\n");

            /* 这里可以添加演示功能代码 */
            delay_ms(1000);
        }

        /* 系统计数和状态显示 */
        system_counter++;

        /* LED闪烁指示系统运行 */
        led_counter++;
        if (led_counter >= 50)  /* 500ms闪烁一次 */
        {
            LED0 = !LED0;
            led_counter = 0;

            /* 每5秒输出一次系统状态 */
            if (system_counter % 500 == 0)
            {
                printf("System running... Counter: %lu\r\n", system_counter);
            }
        }

        delay_ms(10);  /* 10ms延时 */
    }
}