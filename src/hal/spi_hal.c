/**
 * @file    spi_hal.c
 * @brief   SPI硬件抽象层实现 - LCD显示
 * @version V4.0
 * @date    2025-09-27
 */

#include "spi_hal.h"
#include "gd32f4xx.h"
#include <string.h>

// SPI配置信息
static spi_config_t spi_configs[MAX_SPI_CHANNELS];
static bool spi_initialized[MAX_SPI_CHANNELS] = {false};

// SPI外设映射
static const uint32_t spi_periph_map[MAX_SPI_CHANNELS] = {
    SPI0,       // SPI0 - LCD显示
    SPI1,       // SPI1 - 扩展SPI1
    SPI2        // SPI2 - 扩展SPI2
};

// SPI GPIO配置映射
static const spi_gpio_config_t spi_gpio_configs[MAX_SPI_CHANNELS] = {
    // SPI0 - PA5/PA6/PA7
    {
        .sck_port = GPIOA, .sck_pin = GPIO_PIN_5,
        .miso_port = GPIOA, .miso_pin = GPIO_PIN_6,
        .mosi_port = GPIOA, .mosi_pin = GPIO_PIN_7,
        .cs_port = GPIOA, .cs_pin = GPIO_PIN_4,
        .gpio_af = GPIO_AF_5, .rcu_gpio = RCU_GPIOA, .rcu_spi = RCU_SPI0
    },
    // SPI1 - PB13/PB14/PB15
    {
        .sck_port = GPIOB, .sck_pin = GPIO_PIN_13,
        .miso_port = GPIOB, .miso_pin = GPIO_PIN_14,
        .mosi_port = GPIOB, .mosi_pin = GPIO_PIN_15,
        .cs_port = GPIOB, .cs_pin = GPIO_PIN_12,
        .gpio_af = GPIO_AF_5, .rcu_gpio = RCU_GPIOB, .rcu_spi = RCU_SPI1
    },
    // SPI2 - PC10/PC11/PC12
    {
        .sck_port = GPIOC, .sck_pin = GPIO_PIN_10,
        .miso_port = GPIOC, .miso_pin = GPIO_PIN_11,
        .mosi_port = GPIOC, .mosi_pin = GPIO_PIN_12,
        .cs_port = GPIOC, .cs_pin = GPIO_PIN_9,
        .gpio_af = GPIO_AF_6, .rcu_gpio = RCU_GPIOC, .rcu_spi = RCU_SPI2
    }
};

/**
 * @brief  SPI HAL初始化
 * @param  channel: SPI通道
 * @param  config: 配置参数
 * @retval spi_result_t 初始化结果
 */
spi_result_t spi_hal_init(spi_channel_t channel, const spi_config_t* config)
{
    if (channel >= MAX_SPI_CHANNELS || config == NULL) {
        return SPI_ERROR_INVALID_PARAMETER;
    }

    if (spi_initialized[channel]) {
        return SPI_ERROR_ALREADY_INITIALIZED;
    }

    // 保存配置
    memcpy(&spi_configs[channel], config, sizeof(spi_config_t));

    // 使能时钟
    const spi_gpio_config_t* gpio_cfg = &spi_gpio_configs[channel];
    rcu_periph_clock_enable(gpio_cfg->rcu_gpio);
    rcu_periph_clock_enable(gpio_cfg->rcu_spi);

    // 配置GPIO
    gpio_af_set(gpio_cfg->sck_port, gpio_cfg->gpio_af, gpio_cfg->sck_pin);
    gpio_af_set(gpio_cfg->miso_port, gpio_cfg->gpio_af, gpio_cfg->miso_pin);
    gpio_af_set(gpio_cfg->mosi_port, gpio_cfg->gpio_af, gpio_cfg->mosi_pin);

    // SCK配置
    gpio_mode_set(gpio_cfg->sck_port, GPIO_MODE_AF, GPIO_PUPD_NONE, gpio_cfg->sck_pin);
    gpio_output_options_set(gpio_cfg->sck_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, gpio_cfg->sck_pin);

    // MISO配置
    gpio_mode_set(gpio_cfg->miso_port, GPIO_MODE_AF, GPIO_PUPD_NONE, gpio_cfg->miso_pin);

    // MOSI配置
    gpio_mode_set(gpio_cfg->mosi_port, GPIO_MODE_AF, GPIO_PUPD_NONE, gpio_cfg->mosi_pin);
    gpio_output_options_set(gpio_cfg->mosi_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, gpio_cfg->mosi_pin);

    // CS配置 (如果使用软件CS)
    if (config->cs_control == SPI_CS_SOFTWARE) {
        gpio_mode_set(gpio_cfg->cs_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, gpio_cfg->cs_pin);
        gpio_output_options_set(gpio_cfg->cs_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, gpio_cfg->cs_pin);
        gpio_bit_set(gpio_cfg->cs_port, gpio_cfg->cs_pin); // CS默认高电平
    }

    // 配置SPI参数
    uint32_t spi_periph = spi_periph_map[channel];
    spi_parameter_struct spi_init_struct;

    spi_init_struct.trans_mode = config->mode;
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.frame_size = config->data_size;
    spi_init_struct.clock_polarity_phase = config->clock_polarity | config->clock_phase;
    spi_init_struct.nss = (config->cs_control == SPI_CS_HARDWARE) ? SPI_NSS_HARD : SPI_NSS_SOFT;
    spi_init_struct.prescale = config->prescaler;
    spi_init_struct.endian = config->endian;

    spi_init(spi_periph, &spi_init_struct);

    // 使能SPI
    spi_enable(spi_periph);

    spi_initialized[channel] = true;

    return SPI_SUCCESS;
}

/**
 * @brief  SPI传输数据 (全双工)
 * @param  channel: SPI通道
 * @param  tx_data: 发送数据指针
 * @param  rx_data: 接收数据指针
 * @param  length: 数据长度
 * @param  timeout: 超时时间(毫秒)
 * @retval spi_result_t 传输结果
 */
spi_result_t spi_hal_transfer(spi_channel_t channel, const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, uint32_t timeout)
{
    if (channel >= MAX_SPI_CHANNELS || length == 0) {
        return SPI_ERROR_INVALID_PARAMETER;
    }

    if (!spi_initialized[channel]) {
        return SPI_ERROR_NOT_INITIALIZED;
    }

    uint32_t spi_periph = spi_periph_map[channel];
    uint32_t start_time = get_systick_ms();

    // 软件CS控制
    if (spi_configs[channel].cs_control == SPI_CS_SOFTWARE) {
        spi_hal_cs_low(channel);
    }

    for (uint16_t i = 0; i < length; i++) {
        // 发送数据
        uint8_t tx_byte = (tx_data != NULL) ? tx_data[i] : 0xFF;

        // 等待发送缓冲区空
        while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE)) {
            if (timeout > 0 && (get_systick_ms() - start_time) > timeout) {
                if (spi_configs[channel].cs_control == SPI_CS_SOFTWARE) {
                    spi_hal_cs_high(channel);
                }
                return SPI_ERROR_TIMEOUT;
            }
        }

        spi_i2s_data_transmit(spi_periph, tx_byte);

        // 等待接收缓冲区非空
        while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE)) {
            if (timeout > 0 && (get_systick_ms() - start_time) > timeout) {
                if (spi_configs[channel].cs_control == SPI_CS_SOFTWARE) {
                    spi_hal_cs_high(channel);
                }
                return SPI_ERROR_TIMEOUT;
            }
        }

        uint8_t rx_byte = spi_i2s_data_receive(spi_periph);
        if (rx_data != NULL) {
            rx_data[i] = rx_byte;
        }
    }

    // 等待传输完成
    while (SET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TRANS)) {
        if (timeout > 0 && (get_systick_ms() - start_time) > timeout) {
            if (spi_configs[channel].cs_control == SPI_CS_SOFTWARE) {
                spi_hal_cs_high(channel);
            }
            return SPI_ERROR_TIMEOUT;
        }
    }

    // 软件CS控制
    if (spi_configs[channel].cs_control == SPI_CS_SOFTWARE) {
        spi_hal_cs_high(channel);
    }

    return SPI_SUCCESS;
}

/**
 * @brief  SPI发送数据
 * @param  channel: SPI通道
 * @param  data: 数据指针
 * @param  length: 数据长度
 * @param  timeout: 超时时间(毫秒)
 * @retval spi_result_t 发送结果
 */
spi_result_t spi_hal_send(spi_channel_t channel, const uint8_t* data, uint16_t length, uint32_t timeout)
{
    return spi_hal_transfer(channel, data, NULL, length, timeout);
}

/**
 * @brief  SPI接收数据
 * @param  channel: SPI通道
 * @param  data: 数据指针
 * @param  length: 数据长度
 * @param  timeout: 超时时间(毫秒)
 * @retval spi_result_t 接收结果
 */
spi_result_t spi_hal_receive(spi_channel_t channel, uint8_t* data, uint16_t length, uint32_t timeout)
{
    return spi_hal_transfer(channel, NULL, data, length, timeout);
}

/**
 * @brief  SPI CS拉低
 * @param  channel: SPI通道
 * @retval spi_result_t 操作结果
 */
spi_result_t spi_hal_cs_low(spi_channel_t channel)
{
    if (channel >= MAX_SPI_CHANNELS || !spi_initialized[channel]) {
        return SPI_ERROR_INVALID_PARAMETER;
    }

    if (spi_configs[channel].cs_control != SPI_CS_SOFTWARE) {
        return SPI_ERROR_INVALID_OPERATION;
    }

    const spi_gpio_config_t* gpio_cfg = &spi_gpio_configs[channel];
    gpio_bit_reset(gpio_cfg->cs_port, gpio_cfg->cs_pin);

    return SPI_SUCCESS;
}

/**
 * @brief  SPI CS拉高
 * @param  channel: SPI通道
 * @retval spi_result_t 操作结果
 */
spi_result_t spi_hal_cs_high(spi_channel_t channel)
{
    if (channel >= MAX_SPI_CHANNELS || !spi_initialized[channel]) {
        return SPI_ERROR_INVALID_PARAMETER;
    }

    if (spi_configs[channel].cs_control != SPI_CS_SOFTWARE) {
        return SPI_ERROR_INVALID_OPERATION;
    }

    const spi_gpio_config_t* gpio_cfg = &spi_gpio_configs[channel];
    gpio_bit_set(gpio_cfg->cs_port, gpio_cfg->cs_pin);

    return SPI_SUCCESS;
}

/**
 * @brief  SPI读写单字节
 * @param  channel: SPI通道
 * @param  data: 发送数据
 * @retval uint8_t 接收数据
 */
uint8_t spi_hal_read_write_byte(spi_channel_t channel, uint8_t data)
{
    if (channel >= MAX_SPI_CHANNELS || !spi_initialized[channel]) {
        return 0xFF;
    }

    uint8_t rx_data;
    if (spi_hal_transfer(channel, &data, &rx_data, 1, 1000) == SPI_SUCCESS) {
        return rx_data;
    }

    return 0xFF;
}

/**
 * @brief  SPI反初始化
 * @param  channel: SPI通道
 * @retval spi_result_t 反初始化结果
 */
spi_result_t spi_hal_deinit(spi_channel_t channel)
{
    if (channel >= MAX_SPI_CHANNELS) {
        return SPI_ERROR_INVALID_PARAMETER;
    }

    if (!spi_initialized[channel]) {
        return SPI_ERROR_NOT_INITIALIZED;
    }

    uint32_t spi_periph = spi_periph_map[channel];

    // 禁用SPI
    spi_disable(spi_periph);

    // 反初始化SPI
    spi_i2s_deinit(spi_periph);

    spi_initialized[channel] = false;

    return SPI_SUCCESS;
}