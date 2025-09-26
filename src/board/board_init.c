/**
 * @file    board_init.c
 * @brief   板级初始化实现
 * @version V4.0
 * @date    2025-09-27
 */

#include "board_init.h"
#include "system_config.h"
#include "error_handler.h"
#include "adc_hal.h"
#include "gpio_hal.h"
#include "pwm_hal.h"

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"

// GD32F427头文件
#include "gd32f4xx.h"

// 板级配置参数
#define SYSTEM_CLOCK_FREQ       200000000U  // 200MHz系统时钟
#define APB1_CLOCK_FREQ         50000000U   // 50MHz APB1时钟
#define APB2_CLOCK_FREQ         100000000U  // 100MHz APB2时钟

// 看门狗配置
#define WATCHDOG_TIMEOUT_MS     5000        // 5秒看门狗超时

// 外部晶振频率
#define HXTAL_FREQ             25000000U    // 25MHz外部晶振

/**
 * @brief  板级初始化
 * @param  None
 * @retval board_init_result_t 初始化结果
 */
board_init_result_t board_init(void)
{
    board_init_result_t result = BOARD_INIT_SUCCESS;

    // 1. 系统时钟配置
    if (system_clock_config() != BOARD_INIT_SUCCESS) {
        error_log(ERROR_LEVEL_CRITICAL, ERROR_BOARD_INIT_FAILED, "System clock config failed");
        return BOARD_INIT_ERROR_CLOCK;
    }

    // 2. GPIO初始化
    gpio_init_all();

    // 3. 外设初始化
    peripheral_init_all();

    // 4. 硬件抽象层初始化
    adc_hal_init();
    gpio_hal_init();
    pwm_hal_init();

    // 5. 系统滴答定时器配置
    systick_config();

    // 6. 看门狗初始化
    watchdog_early_init();

    // 记录初始化完成
    error_log(ERROR_LEVEL_INFO, 0, "Board initialization completed");

    return result;
}

/**
 * @brief  系统时钟配置
 * @param  None
 * @retval board_init_result_t 配置结果
 */
board_init_result_t system_clock_config(void)
{
    uint32_t timeout = 0;
    ErrorStatus status = ERROR;

    // 1. 使能外部高速晶振 (HXTAL)
    rcu_osci_on(RCU_HXTAL);

    // 等待HXTAL稳定
    timeout = 0;
    while ((rcu_flag_get(RCU_FLAG_HXTALSTB) == RESET) && (timeout < 0x1000)) {
        timeout++;
    }

    if (timeout >= 0x1000) {
        error_log(ERROR_LEVEL_ERROR, ERROR_BOARD_INIT_FAILED, "HXTAL startup failed");
        return BOARD_INIT_ERROR_CLOCK;
    }

    // 2. 配置PLL
    // HXTAL = 25MHz, 目标SYSCLK = 200MHz
    // PLL_CK = (HXTAL / PLLPSC) * PLLN / PLLP
    // PLL_CK = (25MHz / 25) * 400 / 2 = 200MHz
    rcu_pll_config(RCU_PLLSRC_HXTAL, 25, 400, 2);

    // 使能PLL
    rcu_osci_on(RCU_PLL_CK);

    // 等待PLL锁定
    timeout = 0;
    while ((rcu_flag_get(RCU_FLAG_PLLSTB) == RESET) && (timeout < 0x1000)) {
        timeout++;
    }

    if (timeout >= 0x1000) {
        error_log(ERROR_LEVEL_ERROR, ERROR_BOARD_INIT_FAILED, "PLL lock failed");
        return BOARD_INIT_ERROR_CLOCK;
    }

    // 3. 配置系统时钟分频
    rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);      // AHB = SYSCLK / 1 = 200MHz
    rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV4);    // APB1 = AHB / 4 = 50MHz
    rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV2);    // APB2 = AHB / 2 = 100MHz

    // 4. 配置Flash等待周期
    fmc_wscnt_set(WS_WSCNT_6);

    // 5. 切换系统时钟源到PLL
    rcu_system_clock_source_config(RCU_CKSYSSRC_PLL);

    // 等待时钟切换完成
    timeout = 0;
    while ((rcu_system_clock_source_get() != RCU_SCSS_PLL) && (timeout < 0x1000)) {
        timeout++;
    }

    if (timeout >= 0x1000) {
        error_log(ERROR_LEVEL_ERROR, ERROR_BOARD_INIT_FAILED, "Clock source switch failed");
        return BOARD_INIT_ERROR_CLOCK;
    }

    // 6. 更新系统时钟变量
    SystemCoreClockUpdate();

    // 验证时钟频率
    if (SystemCoreClock != SYSTEM_CLOCK_FREQ) {
        error_log(ERROR_LEVEL_WARNING, ERROR_BOARD_INIT_FAILED, "Clock frequency mismatch");
    }

    return BOARD_INIT_SUCCESS;
}

/**
 * @brief  系统滴答定时器配置
 * @param  None
 * @retval None
 */
void systick_config(void)
{
    // 配置SysTick为1ms中断
    if (SysTick_Config(SystemCoreClock / 1000U) != 0) {
        error_log(ERROR_LEVEL_ERROR, ERROR_BOARD_INIT_FAILED, "SysTick config failed");
    }

    // 设置SysTick中断优先级 (最高优先级-1，为FreeRTOS预留)
    NVIC_SetPriority(SysTick_IRQn, 0x0F);
}

/**
 * @brief  看门狗早期初始化
 * @param  None
 * @retval None
 */
void watchdog_early_init(void)
{
    // 配置独立看门狗
    // LSI = ~32kHz, 预分频64, 重装载值2500
    // 超时时间 = (64 * 2500) / 32000 ≈ 5秒
    fwdgt_config(WATCHDOG_TIMEOUT_MS * 32 / 64, FWDGT_PSC_DIV64);

    // 启动看门狗
    fwdgt_enable();

    // 第一次喂狗
    fwdgt_counter_reload();

    error_log(ERROR_LEVEL_INFO, 0, "Watchdog initialized");
}

/**
 * @brief  GPIO全局初始化
 * @param  None
 * @retval None
 */
void gpio_init_all(void)
{
    // 使能所有GPIO端口时钟
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOG);
    rcu_periph_clock_enable(RCU_GPIOH);

    // 初始化关键GPIO引脚
    init_critical_gpio();

    // 初始化调试接口
    init_debug_interface();

    error_log(ERROR_LEVEL_INFO, 0, "GPIO initialized");
}

/**
 * @brief  外设全局初始化
 * @param  None
 * @retval None
 */
void peripheral_init_all(void)
{
    // 使能DMA时钟
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMA1);

    // 使能USART时钟 (用于调试和通信)
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_USART1);

    // 使能SPI时钟 (用于扩展接口)
    rcu_periph_clock_enable(RCU_SPI0);
    rcu_periph_clock_enable(RCU_SPI1);

    // 使能I2C时钟 (用于扩展传感器)
    rcu_periph_clock_enable(RCU_I2C0);
    rcu_periph_clock_enable(RCU_I2C1);

    // 使能以太网时钟
    rcu_periph_clock_enable(RCU_ENET);
    rcu_periph_clock_enable(RCU_ENETTX);
    rcu_periph_clock_enable(RCU_ENETRX);

    // 初始化NVIC
    init_nvic_config();

    error_log(ERROR_LEVEL_INFO, 0, "Peripherals initialized");
}

/**
 * @brief  初始化关键GPIO引脚
 * @param  None
 * @retval None
 */
static void init_critical_gpio(void)
{
    // 紧急停止引脚 (输入，上拉)
    gpio_mode_set(GPIOG, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_8);

    // 系统复位引脚 (输入，上拉)
    gpio_mode_set(GPIOG, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_9);

    // 系统使能引脚 (输出，推挽)
    gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_0);
    gpio_bit_set(GPIOF, GPIO_PIN_0); // 默认使能系统

    // 状态LED引脚 (输出，推挽)
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
                           GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

    // 关闭所有LED
    gpio_bit_reset(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
}

/**
 * @brief  初始化调试接口
 * @param  None
 * @retval None
 */
static void init_debug_interface(void)
{
    // 保持JTAG/SWD接口使能用于调试
    // PA13 (SWDIO), PA14 (SWCLK)
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_13 | GPIO_PIN_14);
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_13 | GPIO_PIN_14);

    // 配置USART0用于调试输出 (PA9=TX, PA10=RX)
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9 | GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_9 | GPIO_PIN_10);

    // 配置USART0: 115200-8-N-1
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

/**
 * @brief  初始化NVIC配置
 * @param  None
 * @retval None
 */
static void init_nvic_config(void)
{
    // 设置中断优先级分组 (4位抢占优先级，0位子优先级)
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    // 配置系统关键中断优先级
    nvic_irq_enable(WWDGT_IRQn, 0, 0);           // 窗口看门狗 (最高优先级)
    nvic_irq_enable(USART0_IRQn, 7, 0);         // 调试串口 (低优先级)

    // 为FreeRTOS预留优先级0-4
    // 应用中断使用优先级5-15
}

/**
 * @brief  喂看门狗
 * @param  None
 * @retval None
 */
void board_feed_watchdog(void)
{
    fwdgt_counter_reload();
}

/**
 * @brief  系统软复位
 * @param  None
 * @retval None
 */
void board_system_reset(void)
{
    error_log(ERROR_LEVEL_WARNING, 0, "System reset requested");

    // 等待日志输出完成
    vTaskDelay(pdMS_TO_TICKS(100));

    // 执行系统复位
    NVIC_SystemReset();
}

/**
 * @brief  获取复位原因
 * @param  None
 * @retval uint32_t 复位原因标志
 */
uint32_t board_get_reset_reason(void)
{
    uint32_t reset_flags = 0;

    // 读取复位标志
    if (rcu_flag_get(RCU_FLAG_PORRST) != RESET) {
        reset_flags |= 0x01; // 上电复位
    }
    if (rcu_flag_get(RCU_FLAG_EPRST) != RESET) {
        reset_flags |= 0x02; // 外部引脚复位
    }
    if (rcu_flag_get(RCU_FLAG_SWRST) != RESET) {
        reset_flags |= 0x04; // 软件复位
    }
    if (rcu_flag_get(RCU_FLAG_FWDGTRST) != RESET) {
        reset_flags |= 0x08; // 独立看门狗复位
    }
    if (rcu_flag_get(RCU_FLAG_WWDGTRST) != RESET) {
        reset_flags |= 0x10; // 窗口看门狗复位
    }

    // 清除复位标志
    rcu_all_reset_flag_clear();

    return reset_flags;
}

/**
 * @brief  获取系统时钟频率
 * @param  clock_type 时钟类型
 * @retval uint32_t 时钟频率 (Hz)
 */
uint32_t board_get_clock_freq(uint8_t clock_type)
{
    switch (clock_type) {
        case 0: // SYSCLK
            return rcu_clock_freq_get(CK_SYS);
        case 1: // AHB
            return rcu_clock_freq_get(CK_AHB);
        case 2: // APB1
            return rcu_clock_freq_get(CK_APB1);
        case 3: // APB2
            return rcu_clock_freq_get(CK_APB2);
        default:
            return 0;
    }
}

/**
 * @brief  板级电源管理初始化
 * @param  None
 * @retval None
 */
void board_power_management_init(void)
{
    // 使能电源管理单元
    rcu_periph_clock_enable(RCU_PMU);

    // 配置待机模式
    pmu_backup_write_enable();
    pmu_standby_mode_config(PMU_STANDBY_FLAG_RESET, PMU_STANDBY_FLAG_RESET);

    error_log(ERROR_LEVEL_INFO, 0, "Power management initialized");
}