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
#include "stm32f4xx_spi_master_emul.h"

#define SPI_EMUL_MASTER_SIDE

#ifdef SPI_EMUL_MASTER_SIDE
/* Definition for TIMx clock resources */
#define TIMx                             TIM4
#define TIMx_CLK_ENABLE()                __HAL_RCC_TIM4_CLK_ENABLE()


/* Definition for TIMx Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIO_PORT                      GPIOD
#define GPIO_PIN_CHANNEL               GPIO_PIN_13
#define GPIO_AF_TIMx                   GPIO_AF2_TIM4

/* Initialize GPIO and pin number for SPI Emulation */
#define SPI_EMUL_Clk_PIN                GPIO_PIN_13
#define SPI_EMUL_Clk_PORT               GPIOD
#define SPI_EMUL_TX_PIN                 TS_DOUT_Pin
#define SPI_EMUL_TX_PORT                TS_DOUT_GPIO_Port
#define SPI_EMUL_RX_PIN                 TS_DIN_Pin
#define SPI_EMUL_RX_PORT                TS_DIN_GPIO_Port


/*  Enable the clock for port SPI Emulation */
#define SPI_EMUL_Clk_GPIO_CLK_ENABLE()        __GPIOD_CLK_ENABLE();
#define SPI_EMUL_TX_GPIO_CLK_ENABLE()         __GPIOG_CLK_ENABLE();
#define SPI_EMUL_RX_GPIO_CLK_ENABLE()         __GPIOG_CLK_ENABLE();

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                     (COUNTOF(aTxBuffer) - 1)

/* Size of Receive buffer */
#define BUFFERSIZE                     TXBUFFERSIZE


/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

volatile static uint8_t Touch_IRQ;

#endif /* SPI_EMUL_MASTER_SIDE */


#ifdef SPI_HW_SLAVE_SIDE

/* Definition for SPIx clock resources */
#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI2_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI2_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_13
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_SCK_AF                      GPIO_AF5_SPI2
#define SPIx_MISO_PIN                    GPIO_PIN_14
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MISO_AF                     GPIO_AF5_SPI2
#define SPIx_MOSI_PIN                    GPIO_PIN_15
#define SPIx_MOSI_GPIO_PORT              GPIOB
#define SPIx_MOSI_AF                     GPIO_AF5_SPI2

/* Definition for SPIx's DMA */
#define SPIx_TX_DMA_CHANNEL              DMA_CHANNEL_0
#define SPIx_TX_DMA_STREAM               DMA1_Stream4
#define SPIx_RX_DMA_CHANNEL              DMA_CHANNEL_0
#define SPIx_RX_DMA_STREAM               DMA1_Stream3

/* Definition for SPIx's NVIC */
#define SPIx_DMA_TX_IRQn                 DMA1_Stream4_IRQn
#define SPIx_DMA_RX_IRQn                 DMA1_Stream3_IRQn
#define SPIx_DMA_TX_IRQHandler           DMA1_Stream4_IRQHandler
#define SPIx_DMA_RX_IRQHandler           DMA1_Stream3_IRQHandler

/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */


#endif /* SPI_HW_SLAVE_SIDE */
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_PIN1_Pin GPIO_PIN_3
#define MOTOR_PIN1_GPIO_Port GPIOE
#define MOTOR_PIN2_Pin GPIO_PIN_4
#define MOTOR_PIN2_GPIO_Port GPIOE
#define MOTOR_PIN3_Pin GPIO_PIN_5
#define MOTOR_PIN3_GPIO_Port GPIOE
#define D1_Pin GPIO_PIN_0
#define D1_GPIO_Port GPIOF
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
#define TS_CS_Pin GPIO_PIN_12
#define TS_CS_GPIO_Port GPIOD
#define TS_CLK_Pin GPIO_PIN_13
#define TS_CLK_GPIO_Port GPIOD
#define LCD_RST_Pin GPIO_PIN_2
#define LCD_RST_GPIO_Port GPIOG
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
