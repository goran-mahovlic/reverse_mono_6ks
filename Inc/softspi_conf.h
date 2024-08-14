/**
 * @file    softspi_conf.h
 * @author  Myth
 * @version 0.2
 * @date    2021.10.12
 * @brief   STM32 SoftSPI Library Config File
 */

#ifndef __SOFTSPI_CONF_H
#define __SOFTSPI_CONF_H

// Set your HAL Library here.

#include "stm32f4xx_hal.h"

#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)
 
#define Delay_us(us) \
    do { \
         uint32_t start = SysTick->VAL; \
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
         while((start - SysTick->VAL) < ticks); \
    } while (0)

// Set your owm Delay_us function here.

//#include "systick.h"

#define SoftSPI_Delay_us(__time__) Delay_us(__time__)

#endif
