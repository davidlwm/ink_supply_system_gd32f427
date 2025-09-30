/**
  ******************************************************************************
  * @file    gd32f4xx.h
  * @author  MCD Application Team
  * @version V2.6.1
  * @date    14-February-2017
  * @brief   CMSIS GD32F4xx Device Peripheral Access Layer Header File.
  *            
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The GD32F4xx device used in the target application
  *              - To use or not the peripheral drivers in application code(i.e. 
  *                code will be based on direct access to peripheral registers 
  *                rather than drivers API), this option is controlled by 
  *                "#define USE_HAL_DRIVER"
  *  
  ******************************************************************************
  * @attention
  *
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup gd32f4xx
  * @{
  */
    
#ifndef __GD32F4xx_H
#define __GD32F4xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
   
/** @addtogroup Library_configuration_section
  * @{
  */
  
/**
  * @brief GD32 Family
  */
#if !defined  (STM32F4)
#define STM32F4
#endif /* GD32F4 */

/* Uncomment the line below according to the target GD32 device used in your
   application 
  */
#if !defined (GD32F405xx) && !defined (GD32F415xx) && !defined (GD32F407xx) && !defined (GD32F417xx) && \
    !defined (GD32F427xx) && !defined (GD32F437xx) && !defined (GD32F429xx) && !defined (GD32F439xx) && \
    !defined (GD32F401xC) && !defined (GD32F401xE) && !defined (GD32F410Tx) && !defined (GD32F410Cx) && \
    !defined (GD32F410Rx) && !defined (GD32F411xE) && !defined (GD32F446xx) && !defined (GD32F469xx) && \
    !defined (GD32F479xx) && !defined (GD32F412Cx) && !defined (GD32F412Rx) && !defined (GD32F412Vx) && \
    !defined (GD32F412Zx) && !defined (GD32F413xx) && !defined (GD32F423xx)
  /* #define GD32F405xx */   /*!< GD32F405RG, GD32F405VG and GD32F405ZG Devices */
  /* #define GD32F415xx */   /*!< GD32F415RG, GD32F415VG and GD32F415ZG Devices */
  /* #define GD32F407xx */   /*!< GD32F407VG, GD32F407VE, GD32F407ZG, GD32F407ZE, GD32F407IG  and GD32F407IE Devices */
  /* #define GD32F417xx */   /*!< GD32F417VG, GD32F417VE, GD32F417ZG, GD32F417ZE, GD32F417IG and GD32F417IE Devices */
  /* #define GD32F427xx */   /*!< GD32F427VG, GD32F427VI, GD32F427ZG, GD32F427ZI, GD32F427IG and GD32F427II Devices */
  /* #define GD32F437xx */   /*!< GD32F437VG, GD32F437VI, GD32F437ZG, GD32F437ZI, GD32F437IG and GD32F437II Devices */
  /* #define GD32F429xx */   /*!< GD32F429VG, GD32F429VI, GD32F429ZG, GD32F429ZI, GD32F429BG, GD32F429BI, GD32F429NG, 
                                   GD32F439NI, GD32F429IG  and GD32F429II Devices */
  /* #define GD32F439xx */   /*!< GD32F439VG, GD32F439VI, GD32F439ZG, GD32F439ZI, GD32F439BG, GD32F439BI, GD32F439NG, 
                                   GD32F439NI, GD32F439IG and GD32F439II Devices */
  /* #define GD32F401xC */   /*!< GD32F401CB, GD32F401CC, GD32F401RB, GD32F401RC, GD32F401VB and GD32F401VC Devices */
  /* #define GD32F401xE */   /*!< GD32F401CD, GD32F401RD, GD32F401VD, GD32F401CE, GD32F401RE and GD32F401VE Devices */
  /* #define GD32F410Tx */   /*!< GD32F410T8 and GD32F410TB Devices */
  /* #define GD32F410Cx */   /*!< GD32F410C8 and GD32F410CB Devices */
  /* #define GD32F410Rx */   /*!< GD32F410R8 and GD32F410RB Devices */
  /* #define GD32F411xE */   /*!< GD32F411CC, GD32F411RC, GD32F411VC, GD32F411CE, GD32F411RE and GD32F411VE Devices */
  /* #define GD32F446xx */   /*!< GD32F446MC, GD32F446ME, GD32F446RC, GD32F446RE, GD32F446VC, GD32F446VE, GD32F446ZC, 
                                   and GD32F446ZE Devices */
  /* #define GD32F469xx */   /*!< GD32F469AI, GD32F469II, GD32F469BI, GD32F469NI, GD32F469AG, GD32F469IG, GD32F469BG, 
                                   GD32F469NG, GD32F469AE, GD32F469IE, GD32F469BE and GD32F469NE Devices */
  /* #define GD32F479xx */   /*!< GD32F479AI, GD32F479II, GD32F479BI, GD32F479NI, GD32F479AG, GD32F479IG, GD32F479BG 
                                   and GD32F479NG Devices */
  /* #define GD32F412Cx */   /*!< GD32F412CEU and GD32F412CGU Devices */
  /* #define GD32F412Zx */   /*!< GD32F412ZET, GD32F412ZGT, GD32F412ZEJ and GD32F412ZGJ Devices */
  /* #define GD32F412Vx */   /*!< GD32F412VET, GD32F412VGT, GD32F412VEH and GD32F412VGH Devices */
  /* #define GD32F412Rx */   /*!< GD32F412RET, GD32F412RGT, GD32F412REY and GD32F412RGY Devices */
  /* #define GD32F413xx */   /*!< GD32F413CH, GD32F413MH, GD32F413RH, GD32F413VH, GD32F413ZH, GD32F413CG, GD32F413MG,
                                   GD32F413RG, GD32F413VG and GD32F413ZG Devices */
  /* #define GD32F423xx */   /*!< GD32F423CH, GD32F423RH, GD32F423VH and GD32F423ZH Devices */
#endif
   
/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
#if !defined  (USE_HAL_DRIVER)
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will 
   be based on direct access to peripherals registers 
   */
  /*#define USE_HAL_DRIVER */
#endif /* USE_HAL_DRIVER */

/**
  * @brief CMSIS version number V2.6.1
  */
#define __GD32F4xx_CMSIS_VERSION_MAIN   (0x02U) /*!< [31:24] main version */
#define __GD32F4xx_CMSIS_VERSION_SUB1   (0x06U) /*!< [23:16] sub1 version */
#define __GD32F4xx_CMSIS_VERSION_SUB2   (0x01U) /*!< [15:8]  sub2 version */
#define __GD32F4xx_CMSIS_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __GD32F4xx_CMSIS_VERSION        ((__GD32F4xx_CMSIS_VERSION_MAIN << 24)\
                                         |(__GD32F4xx_CMSIS_VERSION_SUB1 << 16)\
                                         |(__GD32F4xx_CMSIS_VERSION_SUB2 << 8 )\
                                         |(__GD32F4xx_CMSIS_VERSION))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */

#if defined(STM32F405xx)
  #include "gd32f405xx.h"
#elif defined(STM32F415xx)
  #include "gd32f415xx.h"
#elif defined(STM32F407xx)
  #include "gd32f407xx.h"
#elif defined(STM32F417xx)
  #include "gd32f417xx.h"
#elif defined(STM32F427xx)
  #include "gd32f427xx.h"
#elif defined(STM32F437xx)
  #include "gd32f437xx.h"
#elif defined(STM32F429xx)
  #include "gd32f429xx.h"
#elif defined(STM32F439xx)
  #include "gd32f439xx.h"
#elif defined(STM32F401xC)
  #include "gd32f401xc.h"
#elif defined(STM32F401xE)
  #include "gd32f401xe.h"
#elif defined(STM32F410Tx)
  #include "gd32f410tx.h"
#elif defined(STM32F410Cx)
  #include "gd32f410cx.h"
#elif defined(STM32F410Rx)
  #include "gd32f410rx.h"
#elif defined(STM32F411xE)
  #include "gd32f411xe.h"
#elif defined(STM32F446xx)
  #include "gd32f446xx.h"
#elif defined(STM32F469xx)
  #include "gd32f469xx.h"
#elif defined(STM32F479xx)
  #include "gd32f479xx.h"
#elif defined(STM32F412Cx)
  #include "gd32f412cx.h"
#elif defined(STM32F412Zx)
  #include "gd32f412zx.h"
#elif defined(STM32F412Rx)
  #include "gd32f412rx.h"
#elif defined(STM32F412Vx)
  #include "gd32f412vx.h"
#elif defined(STM32F413xx)
  #include "gd32f413xx.h"
#elif defined(STM32F423xx)
  #include "gd32f423xx.h"
#else
 #error "Please select first the target GD32F4xx device used in your application (in gd32f4xx.h file)"
#endif

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */ 
typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0U, 
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum 
{
  ERROR = 0U, 
  SUCCESS = !ERROR
} ErrorStatus;

/**
  * @}
  */


/** @addtogroup Exported_macro
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL))) 


/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "stm32f4xx_hal.h"
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __GD32F4xx_H */
/**
  * @}
  */

/**
  * @}
  */
  



