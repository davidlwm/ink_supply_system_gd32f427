/**
 * @file epstruct.h
 * @brief lwIP structure packing end for ARM Cortex-M4 + FreeRTOS
 *
 * This file defines the end of structure packing for the lwIP stack.
 * For ARM Cortex-M4 with RVDS/ARM compiler.
 */

#ifdef __CC_ARM
/* Keil MDK-ARM compiler */
#pragma pack(pop)
#elif defined(__GNUC__)
/* GCC compiler - packing is handled in cc.h via __attribute__((packed)) */
#elif defined(__ICCARM__)
/* IAR compiler */
#pragma pack(pop)
#endif