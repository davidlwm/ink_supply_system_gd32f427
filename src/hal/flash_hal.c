/**
 * @file flash_hal.c
 * @brief Flash Hardware Abstraction Layer Implementation
 * @version 1.0
 * @date 2024-12-27
 * @author ink_supply_system
 * @description Flash硬件抽象层实现，基于GD32F4xx HAL库
 */

#include "hal/flash_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

/* 私有宏定义 */
#define FLASH_CONFIG_MAGIC          0x12345678
#define FLASH_CONFIG_VERSION        0x00010001
#define FLASH_UNLOCK_KEY1           0x45670123
#define FLASH_UNLOCK_KEY2           0xCDEF89AB
#define FLASH_CRC32_POLYNOMIAL      0xEDB88320

/* 私有变量 */
static SemaphoreHandle_t g_flash_mutex = NULL;
static flash_state_t g_flash_state = FLASH_STATE_READY;
static uint32_t g_error_count = 0;
static bool g_flash_initialized = false;
static bool g_flash_locked = true;
static flash_erase_callback_t g_erase_callback = NULL;
static flash_program_callback_t g_program_callback = NULL;

/* CRC32查找表 */
static const uint32_t g_crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F,
    0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
    0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
    0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C,
    0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
    0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106,
    0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D,
    0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
    0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7,
    0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA,
    0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
    0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84,
    0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB,
    0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E,
    0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55,
    0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28,
    0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
    0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
    0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69,
    0x616BFFD3, 0x166CCF45, 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC,
    0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693,
    0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

/* 私有函数声明 */
static uint32_t flash_hal_crc32_calc(const uint8_t* data, uint32_t length);
static flash_result_t flash_hal_wait_operation_complete(uint32_t timeout_ms);
static bool flash_hal_is_address_in_range(uint32_t address);

/**
 * @brief Flash HAL初始化
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_init(void)
{
    if (g_flash_initialized) {
        return FLASH_OK;
    }

    /* 创建互斥锁 */
    g_flash_mutex = xSemaphoreCreateMutex();
    if (g_flash_mutex == NULL) {
        return FLASH_ERROR;
    }

    /* 初始化状态 */
    g_flash_state = FLASH_STATE_READY;
    g_error_count = 0;
    g_flash_locked = true;

    /* 使能Flash时钟 */
    rcu_periph_clock_enable(RCU_FMC);

    g_flash_initialized = true;
    return FLASH_OK;
}

/**
 * @brief Flash HAL反初始化
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_deinit(void)
{
    if (!g_flash_initialized) {
        return FLASH_OK;
    }

    /* 锁定Flash */
    flash_hal_lock();

    /* 删除互斥锁 */
    if (g_flash_mutex != NULL) {
        vSemaphoreDelete(g_flash_mutex);
        g_flash_mutex = NULL;
    }

    g_flash_initialized = false;
    return FLASH_OK;
}

/**
 * @brief 解锁Flash
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_unlock(void)
{
    if (!g_flash_initialized) {
        return FLASH_ERROR;
    }

    if (xSemaphoreTake(g_flash_mutex, pdMS_TO_TICKS(FLASH_PROGRAM_TIMEOUT_MS)) != pdTRUE) {
        return FLASH_TIMEOUT;
    }

    if (g_flash_locked) {
        /* 解锁Flash */
        fmc_unlock();
        g_flash_locked = false;
    }

    xSemaphoreGive(g_flash_mutex);
    return FLASH_OK;
}

/**
 * @brief 锁定Flash
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_lock(void)
{
    if (!g_flash_initialized) {
        return FLASH_ERROR;
    }

    if (xSemaphoreTake(g_flash_mutex, pdMS_TO_TICKS(FLASH_PROGRAM_TIMEOUT_MS)) != pdTRUE) {
        return FLASH_TIMEOUT;
    }

    if (!g_flash_locked) {
        /* 锁定Flash */
        fmc_lock();
        g_flash_locked = true;
    }

    xSemaphoreGive(g_flash_mutex);
    return FLASH_OK;
}

/**
 * @brief 擦除扇区
 * @param sector 扇区号
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_erase_sector(uint32_t sector)
{
    if (!g_flash_initialized || g_flash_locked) {
        return FLASH_ERROR;
    }

    if (sector > FLASH_SECTOR_7) {
        return FLASH_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_flash_mutex, pdMS_TO_TICKS(FLASH_ERASE_TIMEOUT_MS)) != pdTRUE) {
        return FLASH_TIMEOUT;
    }

    g_flash_state = FLASH_STATE_BUSY;

    /* 清除错误标志 */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);

    /* 擦除扇区 */
    fmc_state_enum result = fmc_sector_erase(sector);

    flash_result_t flash_result;
    if (result == FMC_READY) {
        /* 等待操作完成 */
        flash_result = flash_hal_wait_operation_complete(FLASH_ERASE_TIMEOUT_MS);
        if (flash_result == FLASH_OK) {
            /* 验证擦除结果 */
            flash_result = flash_hal_blank_check_sector(sector);
        }
    } else {
        flash_result = FLASH_ERASE_FAILED;
        g_error_count++;
    }

    g_flash_state = FLASH_STATE_READY;

    /* 调用回调函数 */
    if (g_erase_callback != NULL) {
        g_erase_callback(sector, flash_result);
    }

    xSemaphoreGive(g_flash_mutex);
    return flash_result;
}

/**
 * @brief 编程字
 * @param address 地址
 * @param data 数据
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_program_word(uint32_t address, uint32_t data)
{
    if (!g_flash_initialized || g_flash_locked) {
        return FLASH_ERROR;
    }

    if (!flash_hal_is_valid_address(address) || !flash_hal_is_aligned_address(address, FLASH_WORD_SIZE)) {
        return FLASH_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_flash_mutex, pdMS_TO_TICKS(FLASH_PROGRAM_TIMEOUT_MS)) != pdTRUE) {
        return FLASH_TIMEOUT;
    }

    g_flash_state = FLASH_STATE_BUSY;

    /* 清除错误标志 */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);

    /* 编程字 */
    fmc_state_enum result = fmc_word_program(address, data);

    flash_result_t flash_result;
    if (result == FMC_READY) {
        /* 等待操作完成 */
        flash_result = flash_hal_wait_operation_complete(FLASH_PROGRAM_TIMEOUT_MS);
        if (flash_result == FLASH_OK) {
            /* 验证编程结果 */
            flash_result = flash_hal_verify_word(address, data);
        }
    } else {
        flash_result = FLASH_PROGRAM_FAILED;
        g_error_count++;
    }

    g_flash_state = FLASH_STATE_READY;

    /* 调用回调函数 */
    if (g_program_callback != NULL) {
        g_program_callback(address, FLASH_WORD_SIZE, flash_result);
    }

    xSemaphoreGive(g_flash_mutex);
    return flash_result;
}

/**
 * @brief 编程缓冲区
 * @param address 起始地址
 * @param data 数据指针
 * @param length 数据长度
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_program_buffer(uint32_t address, const uint8_t* data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return FLASH_INVALID_PARAM;
    }

    if (!flash_hal_is_valid_address(address) || !flash_hal_is_valid_address(address + length - 1)) {
        return FLASH_INVALID_PARAM;
    }

    flash_result_t result = FLASH_OK;
    uint32_t current_addr = address;
    uint32_t remaining = length;

    /* 按字对齐编程 */
    while (remaining >= FLASH_WORD_SIZE && flash_hal_is_aligned_address(current_addr, FLASH_WORD_SIZE)) {
        uint32_t word_data = *(uint32_t*)(data + (current_addr - address));
        result = flash_hal_program_word(current_addr, word_data);
        if (result != FLASH_OK) {
            return result;
        }
        current_addr += FLASH_WORD_SIZE;
        remaining -= FLASH_WORD_SIZE;
    }

    /* 编程剩余字节 */
    while (remaining > 0) {
        uint8_t byte_data = data[current_addr - address];
        result = flash_hal_program_byte(current_addr, byte_data);
        if (result != FLASH_OK) {
            return result;
        }
        current_addr++;
        remaining--;
    }

    return result;
}

/**
 * @brief 编程字节
 * @param address 地址
 * @param data 数据
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_program_byte(uint32_t address, uint8_t data)
{
    if (!g_flash_initialized || g_flash_locked) {
        return FLASH_ERROR;
    }

    if (!flash_hal_is_valid_address(address)) {
        return FLASH_INVALID_PARAM;
    }

    if (xSemaphoreTake(g_flash_mutex, pdMS_TO_TICKS(FLASH_PROGRAM_TIMEOUT_MS)) != pdTRUE) {
        return FLASH_TIMEOUT;
    }

    g_flash_state = FLASH_STATE_BUSY;

    /* 清除错误标志 */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);

    /* 编程字节 */
    fmc_state_enum result = fmc_byte_program(address, data);

    flash_result_t flash_result;
    if (result == FMC_READY) {
        /* 等待操作完成 */
        flash_result = flash_hal_wait_operation_complete(FLASH_PROGRAM_TIMEOUT_MS);
        if (flash_result == FLASH_OK) {
            /* 验证编程结果 */
            flash_result = flash_hal_verify_byte(address, data);
        }
    } else {
        flash_result = FLASH_PROGRAM_FAILED;
        g_error_count++;
    }

    g_flash_state = FLASH_STATE_READY;
    xSemaphoreGive(g_flash_mutex);
    return flash_result;
}

/**
 * @brief 读取字
 * @param address 地址
 * @param data 数据指针
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_read_word(uint32_t address, uint32_t* data)
{
    if (data == NULL) {
        return FLASH_INVALID_PARAM;
    }

    if (!flash_hal_is_valid_address(address) || !flash_hal_is_aligned_address(address, FLASH_WORD_SIZE)) {
        return FLASH_INVALID_PARAM;
    }

    *data = *(volatile uint32_t*)address;
    return FLASH_OK;
}

/**
 * @brief 读取缓冲区
 * @param address 起始地址
 * @param data 数据缓冲区
 * @param length 数据长度
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_read_buffer(uint32_t address, uint8_t* data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return FLASH_INVALID_PARAM;
    }

    if (!flash_hal_is_valid_address(address) || !flash_hal_is_valid_address(address + length - 1)) {
        return FLASH_INVALID_PARAM;
    }

    memcpy(data, (void*)address, length);
    return FLASH_OK;
}

/**
 * @brief 验证字
 * @param address 地址
 * @param data 期望数据
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_verify_word(uint32_t address, uint32_t data)
{
    uint32_t read_data;
    flash_result_t result = flash_hal_read_word(address, &read_data);

    if (result == FLASH_OK && read_data != data) {
        result = FLASH_VERIFY_FAILED;
    }

    return result;
}

/**
 * @brief 验证字节
 * @param address 地址
 * @param data 期望数据
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_verify_byte(uint32_t address, uint8_t data)
{
    if (!flash_hal_is_valid_address(address)) {
        return FLASH_INVALID_PARAM;
    }

    uint8_t read_data = *(volatile uint8_t*)address;

    if (read_data != data) {
        return FLASH_VERIFY_FAILED;
    }

    return FLASH_OK;
}

/**
 * @brief 空白检查扇区
 * @param sector 扇区号
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_blank_check_sector(uint32_t sector)
{
    if (sector > FLASH_SECTOR_7) {
        return FLASH_INVALID_PARAM;
    }

    uint32_t start_addr = flash_hal_sector_to_address(sector);
    uint32_t end_addr = start_addr + FLASH_SECTOR_SIZE;

    return flash_hal_blank_check_range(start_addr, end_addr);
}

/**
 * @brief 空白检查地址范围
 * @param start_addr 起始地址
 * @param end_addr 结束地址
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_blank_check_range(uint32_t start_addr, uint32_t end_addr)
{
    if (!flash_hal_is_valid_address(start_addr) || !flash_hal_is_valid_address(end_addr - 1)) {
        return FLASH_INVALID_PARAM;
    }

    for (uint32_t addr = start_addr; addr < end_addr; addr += FLASH_WORD_SIZE) {
        uint32_t data;
        flash_result_t result = flash_hal_read_word(addr, &data);
        if (result != FLASH_OK) {
            return result;
        }
        if (data != 0xFFFFFFFF) {
            return FLASH_VERIFY_FAILED;
        }
    }

    return FLASH_OK;
}

/**
 * @brief 保存配置
 * @param config 配置数据
 * @param length 数据长度
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_save_config(const void* config, uint32_t length)
{
    if (config == NULL || length == 0 || length > sizeof(((flash_config_t*)0)->data)) {
        return FLASH_INVALID_PARAM;
    }

    flash_config_t flash_config;
    flash_config.magic = FLASH_CONFIG_MAGIC;
    flash_config.version = FLASH_CONFIG_VERSION;
    flash_config.length = length;

    /* 复制配置数据 */
    memcpy(flash_config.data, config, length);

    /* 计算CRC32 */
    flash_config.crc32 = flash_hal_crc32_calc((uint8_t*)&flash_config.version,
                                             sizeof(flash_config) - sizeof(flash_config.magic) - sizeof(flash_config.crc32));

    /* 擦除配置扇区 */
    flash_result_t result = flash_hal_erase_sector(FLASH_CONFIG_SECTOR);
    if (result != FLASH_OK) {
        return result;
    }

    /* 编程配置数据 */
    result = flash_hal_program_buffer(FLASH_CONFIG_ADDR, (uint8_t*)&flash_config, sizeof(flash_config));

    return result;
}

/**
 * @brief 加载配置
 * @param config 配置缓冲区
 * @param max_length 最大长度
 * @param actual_length 实际长度
 * @return flash_result_t 操作结果
 */
flash_result_t flash_hal_load_config(void* config, uint32_t max_length, uint32_t* actual_length)
{
    if (config == NULL) {
        return FLASH_INVALID_PARAM;
    }

    flash_config_t flash_config;

    /* 读取配置数据 */
    flash_result_t result = flash_hal_read_buffer(FLASH_CONFIG_ADDR, (uint8_t*)&flash_config, sizeof(flash_config));
    if (result != FLASH_OK) {
        return result;
    }

    /* 验证魔术字 */
    if (flash_config.magic != FLASH_CONFIG_MAGIC) {
        return FLASH_VERIFY_FAILED;
    }

    /* 验证CRC32 */
    uint32_t calculated_crc = flash_hal_crc32_calc((uint8_t*)&flash_config.version,
                                                  sizeof(flash_config) - sizeof(flash_config.magic) - sizeof(flash_config.crc32));
    if (calculated_crc != flash_config.crc32) {
        return FLASH_VERIFY_FAILED;
    }

    /* 检查长度 */
    if (flash_config.length > max_length) {
        return FLASH_INVALID_PARAM;
    }

    /* 复制配置数据 */
    memcpy(config, flash_config.data, flash_config.length);

    if (actual_length != NULL) {
        *actual_length = flash_config.length;
    }

    return FLASH_OK;
}

/**
 * @brief 计算CRC32
 * @param address 起始地址
 * @param length 数据长度
 * @return uint32_t CRC32值
 */
uint32_t flash_hal_calculate_crc32(uint32_t address, uint32_t length)
{
    if (!flash_hal_is_valid_address(address) || length == 0) {
        return 0;
    }

    return flash_hal_crc32_calc((uint8_t*)address, length);
}

/**
 * @brief 扇区地址转换
 * @param sector 扇区号
 * @return uint32_t 扇区起始地址
 */
uint32_t flash_hal_sector_to_address(uint32_t sector)
{
    uint32_t address = FLASH_BASE_ADDR;

    switch (sector) {
        case FLASH_SECTOR_0: address += 0x00000; break;
        case FLASH_SECTOR_1: address += 0x04000; break;
        case FLASH_SECTOR_2: address += 0x08000; break;
        case FLASH_SECTOR_3: address += 0x0C000; break;
        case FLASH_SECTOR_4: address += 0x10000; break;
        case FLASH_SECTOR_5: address += 0x20000; break;
        case FLASH_SECTOR_6: address += 0x40000; break;
        case FLASH_SECTOR_7: address += 0x60000; break;
        default: address = 0; break;
    }

    return address;
}

/**
 * @brief 获取Flash状态
 * @return flash_state_t Flash状态
 */
flash_state_t flash_hal_get_state(void)
{
    return g_flash_state;
}

/**
 * @brief 检查是否忙碌
 * @return bool 忙碌状态
 */
bool flash_hal_is_busy(void)
{
    return (g_flash_state == FLASH_STATE_BUSY) || (fmc_state_get() == FMC_BUSY);
}

/* 私有函数实现 */

/**
 * @brief 计算CRC32
 * @param data 数据指针
 * @param length 数据长度
 * @return uint32_t CRC32值
 */
static uint32_t flash_hal_crc32_calc(const uint8_t* data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < length; i++) {
        crc = g_crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }

    return ~crc;
}

/**
 * @brief 等待操作完成
 * @param timeout_ms 超时时间
 * @return flash_result_t 操作结果
 */
static flash_result_t flash_hal_wait_operation_complete(uint32_t timeout_ms)
{
    uint32_t start_time = xTaskGetTickCount();

    while (flash_hal_is_busy()) {
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout_ms)) {
            return FLASH_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return FLASH_OK;
}

/**
 * @brief 检查地址是否有效
 * @param address 地址
 * @return bool 地址有效性
 */
static bool flash_hal_is_address_in_range(uint32_t address)
{
    return (address >= FLASH_BASE_ADDR) && (address < (FLASH_BASE_ADDR + 0x80000));
}

/**
 * @brief 检查地址是否有效
 * @param address 地址
 * @return bool 地址有效性
 */
bool flash_hal_is_valid_address(uint32_t address)
{
    return flash_hal_is_address_in_range(address);
}

/**
 * @brief 检查地址对齐
 * @param address 地址
 * @param alignment 对齐要求
 * @return bool 对齐状态
 */
bool flash_hal_is_aligned_address(uint32_t address, uint32_t alignment)
{
    return (address % alignment) == 0;
}