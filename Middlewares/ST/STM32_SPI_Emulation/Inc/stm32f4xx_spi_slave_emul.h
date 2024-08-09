/**
  ******************************************************************************
  * @file    stm32f4xx_hal_spi_slave_emul.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-July-2015
  * @brief   Header file of SPI Emulation HAL module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_SPI_SLAVE_EMUL_H
#define __STM32F4xx_HAL_SPI_SLAVE_EMUL_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_def.h" 
	 #include "stm32f4xx_nucleo.h"
	 #include "main.h" 
	 
/** @addtogroup STM32F4xx_HAL_SPI_SALVE_EMUL_Driver
  * @{
  */

/** @addtogroup SPI_EMUL_HAL_Driver
  * @{
  */ 
	 
/* Exported types ------------------------------------------------------------*/
  
/** 
  * @brief  SPI Emulation Configuration Structure definition  
  */
typedef struct
{
  uint32_t Mode;               /*!< Specifies the SPI operating mode.
                                    This parameter can be a value of @ref SPI_Emul_mode */

  uint32_t Direction;          /*!< Specifies the SPI Directional mode state.
                                    This parameter can be a value of @ref SPI_Emul_Direction_mode */

  uint32_t DataSize;           /*!< Specifies the SPI data size.
                                    This parameter can be a value of @ref SPI_Emul_data_size */

  uint32_t CLKPolarity;        /*!< Specifies the serial clock steady state.
                                    This parameter can be a value of @ref SPI_Emul_Clock_Polarity */

  uint32_t CLKPhase;           /*!< Specifies the clock active edge for the bit capture.
                                    This parameter can be a value of @ref SPI_Emul_Clock_Phase */

  uint32_t SPI_Clk;            /*!< Specifies the SPI Clock Frequency which will be
                                    used in transmission and reception.
                                    This parameter must be a number between Min_Data = 500000 and Max_Data = 10000000
                                    @note The communication clock is derived from the master
                                    clock. The slave clock does not need to be set */

  uint32_t FirstBit;           /*!< Specifies whether data transfers start from MSB or LSB bit.
                                    This parameter can be a value of @ref SPI_Emul_MSB_LSB_transmission */
																		
  uint32_t RxPinNumber;        /*!< Specifies the number of Receiver Pin. 
	                                  This parameter can be a value of @ref GPIO_pins_define */
  
  uint32_t TxPinNumber;        /*!< Specifies the number of Transmitter Pin. 
	                                  This parameter can be a value of @ref GPIO_pins_define */
																		
	uint32_t ClkPinNumber;        /*!< Specifies the number of Clock Pin. 
	                                  This parameter can be a value of @ref GPIO_pins_define */
																		


}SPI_Emul_InitTypeDef;

/**
  * @brief  HAL SPI Emulation State structure definition
  */
typedef enum
{
  HAL_SPI_EMUL_STATE_RESET      = 0x00,  /*!< SPI not yet initialized or disabled */
  HAL_SPI_EMUL_STATE_READY      = 0x01,  /*!< SPI initialized and ready for use */
  HAL_SPI_EMUL_STATE_BUSY       = 0x02,  /*!< SPI process is ongoing */
  HAL_SPI_EMUL_STATE_BUSY_TX    = 0x12,  /*!< Data Transmission process is ongoing */
  HAL_SPI_EMUL_STATE_BUSY_RX    = 0x22,  /*!< Data Reception process is ongoing */
  HAL_SPI_EMUL_STATE_BUSY_TX_RX = 0x32,  /*!< Data Transmission and Reception process is ongoing */
  HAL_SPI_EMUL_STATE_ERROR      = 0x03   /*!< SPI error state */
    
}HAL_SPI_Emul_StateTypeDef;

/** 
  * @brief  HAL SPI Emulation Error Code structure definition  
  */ 
typedef enum
{
  HAL_SPI_EMUL_ERROR_NONE      = 0x00,    /*!< No error            */
  HAL_SPI_EMUL_ERROR_FE        = 0x01,    /*!< frame error         */
	HAL_SPI_EMUL_ERROR_RE        = 0x02,    /*!< receiver error      */
	HAL_SPI_EMUL_ERROR_PE        = 0x04     /*!< transfer error      */
}HAL_SPI_Emul_ErrorTypeDef;

/** 
  * @brief Serial Peripheral Interface
  */
     
typedef struct
{
  __IO uint8_t SR;         /*!< SPI Emulation Status register software */
  
} SPI_Emul_TypeDef;

/** 
  * @brief  SPI Emulation handle Structure definition
  */
typedef struct __SPI_Emul_HandleTypeDef
{
  SPI_TypeDef                     Instance;    /* Instance for SPI Emulation register */

  SPI_Emul_InitTypeDef            Init;         /* SPI Emulation communication parameters */

  uint8_t                         *pTxBuffPtr;  /* Pointer to SPI Emulation Tx transfer Buffer */

  uint16_t                        TxXferSize;   /* SPI Emulation Tx transfer size */
  
  uint16_t                        TxXferCount;  /* SPI Emulation Tx Transfer Counter */

  uint8_t                         *pRxBuffPtr;  /* Pointer to SPI Emulation Rx transfer Buffer */

  uint16_t                        RxXferSize;   /* SPI Emulation Rx transfer size */

  uint16_t                        RxXferCount;  /* SPI Emulation Rx Transfer Counter */

  GPIO_TypeDef                    *RxPortName;  /* SPI Emulation Rx port name */	
	
	GPIO_TypeDef                    *TxPortName;  /* SPI Emulation Tx port name */
	
	GPIO_TypeDef                    *ClkPortName;  /* SPI Emulation Clock port name */	
	
  GPIO_TypeDef                    *NSSPortName;  /* SPI Emulation Clock port name */

  void                            (*RxISR)(struct __SPI_Emul_HandleTypeDef * hspi); /* function pointer on Rx ISR */

  void                            (*TxISR)(struct __SPI_Emul_HandleTypeDef * hspi); /* function pointer on Tx ISR */

	
	__IO HAL_SPI_Emul_StateTypeDef  State;        /* SPI Emulation communication state */
  
  __IO HAL_SPI_Emul_ErrorTypeDef  ErrorCode;    /* SPI Emulation Error code */


}SPI_Emul_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_Emulation_Exported_Constants
  * @{
  */

/** @defgroup SPI_Emul_mode
  * @{
  */
#define SPI_EMUL_MODE_SLAVE                 ((uint8_t)0x01)
#define SPI_EMUL_MODE_MASTER                ((uint8_t)0x02)
#define IS_SPI_EMUL_MODE(MODE)              (((MODE) == SPI_EMUL_MODE_SLAVE) || \
																					  ((MODE) == SPI_EMUL_MODE_MASTER))
/**
  * @}
  */

/** @defgroup SPI_Emul_Direction_mode
  * @{
  */

#define SPI_EMUL_DIRECTION_RX                      ((uint32_t)0x00000000)
#define SPI_EMUL_DIRECTION_TX                      ((uint32_t)0x00000001)
#define SPI_EMUL_DIRECTION_TX_RX                   ((uint32_t)0x00000002)
#define IS_SPI_EMUL_DIRECTION(DIRECTION)           (((DIRECTION) == SPI_EMUL_DIRECTION_RX) || \
																					         ((DIRECTION) == SPI_EMUL_DIRECTION_TX ) || \
																					         ((DIRECTION) == SPI_EMUL_DIRECTION_TX_RX ))


/**
  * @}
  */

/** @defgroup SPI_Emul_data_size
  * @{
  */
#define SPI_EMUL_DATASIZE_8BIT               ((uint32_t)0x00000008)
#define SPI_EMUL_DATASIZE_16BIT              ((uint32_t)0x00000010)
#define SPI_EMUL_DATASIZE_32BIT              ((uint32_t)0x00000020)
#define IS_SPI_EMUL_DATASIZE(DATASIZE)       (((DATASIZE) == SPI_EMUL_DATASIZE_8BIT) || \
                                             ((DATASIZE) == SPI_EMUL_DATASIZE_16BIT) || \
                                             ((DATASIZE) == SPI_EMUL_DATASIZE_32BIT))
/**
  * @}
  */ 

/** @defgroup SPI_Emul_Clock_Polarity
  * @{
  */
#define SPI_EMUL_POLARITY_LOW                ((uint32_t)0x00000001)
#define SPI_EMUL_POLARITY_HIGH               ((uint32_t)0x00000002)
#define IS_SPI_EMUL_POLARITY(POLARITY)       (((POLARITY) == SPI_EMUL_POLARITY_LOW) || \
																					   ((POLARITY) == SPI_EMUL_POLARITY_HIGH))
/**
  * @}
  */

/** @defgroup SPI_Emul_Clock_Phase
  * @{
  */
#define SPI_EMUL_PHASE_1EDGE                 ((uint32_t)0x00000001)
#define SPI_EMUL_PHASE_2EDGE                 ((uint32_t)0x00000002)
#define IS_SPI_EMUL_PHASE(PHASE)             (((PHASE) == SPI_EMUL_PHASE_1EDGE) || \
																					   ((PHASE) == SPI_EMUL_PHASE_1EDGE))
/**
  * @}
  */
	
/** @defgroup SPI_Emul_BaudRate
  * @{
  */

#define IS_SPI_CLK(SPI_CLK) (((SPI_CLK) >= 0) && ((SPI_CLK) <= 10000000))


  
/** @defgroup SPI_Emul_MSB_LSB_transmission
  * @{
  */
#define SPI_EMUL_FIRSTBIT_MSB                ((uint32_t)0x00000000)
#define SPI_EMUL_FIRSTBIT_LSB                ((uint32_t)0x00000001)
#define IS_SPI_EMUL_FIRSTBIT(FIRSTBIT)       (((FIRSTBIT) == SPI_EMUL_FIRSTBIT_MSB) || \
																					   ((FIRSTBIT) == SPI_EMUL_FIRSTBIT_LSB))
/**
  * @}
  */
	
/** @defgroup SPI_Emul_CRC_Calculation
  * @{
  */
#define SPI_EMUL_CRCCALCULATION_DISABLE                  ((uint32_t)0x00000000)
#define SPI_EMUL_CRCCALCULATION_ENABLE                   ((uint32_t)0x00000001)
#define IS_SPI_EMUL_CRCCALCULATION(CRCCALCULATION)       (((CRCCALCULATION) == SPI_EMUL_CRCCALCULATION_DISABLE) || \
																					               ((CRCCALCULATION) == SPI_EMUL_CRCCALCULATION_ENABLE))
/**
  * @}
  */
	
#define IS_SPI_EMUL_CRC_POLYNOMIAL(POLYNOMIAL) (((POLYNOMIAL) >= 0x1) && ((POLYNOMIAL) <= 0xFFFF))


/** @brief  Enable the clock for SPI Emulation.
  *            clock in the peripherique used in this driver Timer and DMA   
  * @param  None  
  * @retval None
  */																										
#define __SPI_EMUL_CLK_ENABLE()                   __TIM1_CLK_ENABLE();\
                                                  __DMA2_CLK_ENABLE();
																													
#define __SPI_EMUL_CLK_DISABLE()                 __TIM1_CLK_DISABLE();\
                                                 __DMA2_CLK_DISABLE();																												
																													
/** @defgroup SPI Emulation constant 
  * @{
  */

#define ONE_FRAME          ((uint8_t)0x01)
#define TWO_FRAMES         ((uint8_t)0x02)
#define BitMask_LSB        ((uint8_t)0x01)
#define BitMask_MSB_8bit   ((uint8_t)0x80)
#define BitMask_MSB_16bit  ((uint16_t)0x8000)
#define BitMask_MSB_32bit  ((uint32_t)0x80000000)
#define RX_BUFFER_SIZE     ((uint8_t)0x50)
#define TX_BUFFER_SIZE     ((uint8_t)0x50)
#define ONE_BYTE              ((uint8_t)0x08)
#define NUMBER_OF_DATA_ITEMS  ((uint32_t)320)


/* Definition Handler for SPI Emulation transmit mode */ 
#define SPI_EMUL_TX_DMA_IRQHandler           DMA2_Stream1_IRQHandler
#define SPI_EMUL_RX_DMA_IRQHandler           DMA2_Stream2_IRQHandler

/** @brief  Determinate the size for the frame .
  * @param  __HANDLE__: specifies the SPI Emulation Handle.    
  * @param  None  
  * @retval None
  */	  
#define __HAL_SPI_EMUL_FRAME_SIZE(__HANDLE__)          (uint32_t)((__HANDLE__)->Init.DataSize)

/** @brief  Set the specified SPI Emulation  flag.
  * @param  __HANDLE__: specifies the SPI Emulation Handle.
  *    
  * @param  __FLAG__: specifies the flag to check.
  *          This parameter can be any combination of the following values:
  *            @arg SPI_EMUL_FLAG_RC :  Receiver Complete.
  *            @arg SPI_EMUL_FLAG_TC :  Transmitter Complete.
  *            @arg SPI_EMUL_FLAG_FE :  Frame Error.
  *   
  * @retval None
  */
#define __HAL_SPI_EMUL_SET_FLAG(__HANDLE__, __FLAG__)  ((__HANDLE__)->Instance.SR |= (__FLAG__))

/** @brief  Checks whether the specified SPI Emulation flag is set or not.
  * @param  __HANDLE__: specifies the SPI Emulation Handle.
  * @param  __FLAG__: specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg SPI_EMUL_FLAG_RC:  Receiver Complete flag
  *            @arg SPI_EMUL_FLAG_TC:  Transmission Complete flag
  *            @arg SPI_EMUL_FLAG_FE:  Framing Error flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_SPI_EMUL_GET_FLAG(__HANDLE__, __FLAG__)  (((__HANDLE__)->Instance.SR & (__FLAG__)) == (__FLAG__))   

/** @brief  Clears the specified SPI Emulation  flag.
  * @param  __HANDLE__: specifies the SPI Emulation Handle.
  *    
  * @param  __FLAG__: specifies the flag to check.
  *          This parameter can be any combination of the following values:
  *            @arg SPI_EMUL_FLAG_RC :  Receiver Complet.
  *            @arg SPI_EMUL_FLAG_TC :  Transmitter Complet.
  *            @arg SPI_EMUL_FLAG_FE :  Frame Error.
  *    
  *   
  * @retval None
  */
#define __HAL_SPI_EMUL_CLEAR_FLAG(__HANDLE__, __FLAG__)  ((__HANDLE__)->Instance.SR &= ~(__FLAG__))

/** @brief  Set the specified SPI Emulation  flag.
  * @param  __HANDLE__: specifies the SPI Emulation Handle.
  *    
  * @param  __FLAG__: specifies the flag to check.
  *          This parameter can be any combination of the following values:
  *            @arg SPI_EMUL_FLAG_RC :  Receiver Complete.
  *            @arg SPI_EMUL_FLAG_TC :  Transmitter Complete.
  *            @arg SPI_EMUL_FLAG_FE :  Frame Error.
  *   
  * @retval None
  */


/** @defgroup SPI_Emul_Flags 
  *           Elements values convention: 0xXX
  *           - 0xXX  : Flag mask in the SR register software
  * @{
  */
#define SPI_EMUL_FLAG_RC                        ((uint8_t)0x01)
#define SPI_EMUL_FLAG_TC                        ((uint8_t)0x02)
#define SPI_EMUL_FLAG_FE                        ((uint8_t)0x04)

/* Functions prototypes */
void HAL_SPI_Emul_MspInit(SPI_Emul_HandleTypeDef *hspi);
void HAL_SPI_Emul_MspDeInit(SPI_Emul_HandleTypeDef *hspi);
void HAL_SPI_Emul_TxCpltCallback(SPI_Emul_HandleTypeDef *hspi);
void HAL_SPI_Emul_TxHalfCpltCallback(SPI_Emul_HandleTypeDef *hspi);
void HAL_SPI_Emul_ErrorCallback(SPI_Emul_HandleTypeDef *hspi);
void HAL_SPI_Emul_RxCpltCallback(SPI_Emul_HandleTypeDef *hspi);
void HAL_SPI_Emul_IRQHandler(SPI_Emul_HandleTypeDef *hspi);
void HAL_SPI_Emul_RxHalfCpltCallback(SPI_Emul_HandleTypeDef *hspi);

HAL_StatusTypeDef HAL_SPI_Emul_Init(SPI_Emul_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Emul_Transmit_DMA(SPI_Emul_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Emul_Receive_DMA(SPI_Emul_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Emul_TransmitReceive_DMA(SPI_Emul_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

/**
  * @}
  */ 

/**
  * @}
  */ 
  
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_SPI_SLAVE_EMUL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
