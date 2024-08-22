/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AccelStepper.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
volatile uint8_t touchPressed;
//uint16_t x , y;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_ENABLE_Pin GPIO_PIN_3
#define MOTOR_ENABLE_GPIO_Port GPIOE
#define MOTOR_DIR_Pin GPIO_PIN_4
#define MOTOR_DIR_GPIO_Port GPIOE
#define MOTOR_STEP_Pin GPIO_PIN_5
#define MOTOR_STEP_GPIO_Port GPIOE
#define D1_Pin GPIO_PIN_0
#define D1_GPIO_Port GPIOF
#define HOME_SW_Pin GPIO_PIN_3
#define HOME_SW_GPIO_Port GPIOC
#define FAN_Pin GPIO_PIN_3
#define FAN_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_6
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define UV_LED_Pin GPIO_PIN_0
#define UV_LED_GPIO_Port GPIOG
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOG
#define LCD_PB10_Pin GPIO_PIN_10
#define LCD_PB10_GPIO_Port GPIOB
#define LCD_PB11_Pin GPIO_PIN_11
#define LCD_PB11_GPIO_Port GPIOB
#define TS_CS_Pin GPIO_PIN_12
#define TS_CS_GPIO_Port GPIOD
#define TS_CLK_Pin GPIO_PIN_13
#define TS_CLK_GPIO_Port GPIOD
#define LCD_RSTG2_Pin GPIO_PIN_2
#define LCD_RSTG2_GPIO_Port GPIOG
#define TS_DOUT_Pin GPIO_PIN_3
#define TS_DOUT_GPIO_Port GPIOG
#define TS_DIN_Pin GPIO_PIN_4
#define TS_DIN_GPIO_Port GPIOG
#define TS_IRQ_Pin GPIO_PIN_6
#define TS_IRQ_GPIO_Port GPIOG
#define TS_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define FMC_A1_REAL_Pin GPIO_PIN_7
#define FMC_A1_REAL_GPIO_Port GPIOG
#define LCD_BL_Pin GPIO_PIN_8
#define LCD_BL_GPIO_Port GPIOG
#define MOTOR_M0_Pin GPIO_PIN_10
#define MOTOR_M0_GPIO_Port GPIOG
#define MOTOR_DEC1_Pin GPIO_PIN_11
#define MOTOR_DEC1_GPIO_Port GPIOG
#define MOTOR_DEC0_Pin GPIO_PIN_12
#define MOTOR_DEC0_GPIO_Port GPIOG
#define MOTOR_M1_Pin GPIO_PIN_13
#define MOTOR_M1_GPIO_Port GPIOG
#define MOTOR_nSLEEP_Pin GPIO_PIN_14
#define MOTOR_nSLEEP_GPIO_Port GPIOG
#define SPI3_NSS_Pin GPIO_PIN_15
#define SPI3_NSS_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
