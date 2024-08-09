/**
  ******************************************************************************
  * @file    stm32f4xx_hal_spi_master_emul.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-July-2015
  * @brief   SPI Emulation HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Serial Peripheral Interface (SPI Emulation):
  *           + Initialization and de-initialization functions
  *           + IO operation functions
#include "stm32f4xx.h"                  // Device header
  *           + Peripheral State and Errors functions
  *
  @verbatim
   ==============================================================================
                        ##### How to use this driver #####
   ==============================================================================
 [..]
    The SPI Emulation HAL driver can be used as follows:

  (#) Declare a SPI_Emul_HandleTypeDef handle structure.

  (#)Initialize the SPI Emulation low level resources by implementing the HAL_SPI_MspInit ()API:
      (##) Enable the SPI_EMUL  clock.
        (##) SPI Emulation port declaration
            (+++) SPI pins configuration: TxPinNumber, RxPinNumber and ClkPinNumber
            (+++) Enable the clock for the GPIOs

  (#) Program the Mode, Direction , Data size, clock speed, Clock polarity and phase,
       FirstBit and CRC configuration in the hspi Emul Init structure.

  (#) Initialize the SPI Emulation registers software by calling the HAL_SPI_Emul_Init() API.

    -@- The specific SPI Emulaion Handle (Transmission complete, Reception complete
            and Transfer Error ) will be managed using the macros
            HAL_SPI_Emul_TxCpltCallback(), HAL_SPI_Emul_RxCpltCallback() and __HAL_SPI_Emul_TranferError() inside the transmit
            and receive process.

        -@- These API's(HAL_SPI_Emul_Init() configures also the
            low level Hardware GPIO, CLOCK, CORTEX...etc) by calling the customed
            HAL_SPI_Emul_MspInit() API.

    (#) Three mode of operations are available within this driver :

     *** SPI Emulation mode IO operation ***
     ===================================
     [..]
       (+) Send an amount of data in non blocking mode (DMA) using HAL_SPI_Emul_Transmit_DMA()
       (+) At transmission end of transfer HAL_SPI_Emul_TxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_SPI_Emul_TxCpltCallback
       (+) Receive an amount of data in non blocking mode (DMA) using HAL_SPI_Emul_Receive_DMA()

       (+) At reception end of transfer HAL_SPI_Emul_RxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_SPI_Emul_RxCpltCallback
       (+) In case of transfer Error, HAL_SPI_ErrorCallback() function is executed and user can
            add his own code by customization of function pointer HAL_SPI_Emul_ErrorCallback

     *** SPI Emulation HAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in SPI Emulation HAL driver.

      (+) __HAL_SPI_EMUL_GET_FLAG : Checks whether the specified SPI Emulation flag is set or not
      (+) __HAL_SPI_EMUL_CLEAR_FLAG : Clears the specified SPI Emulation pending flag
      (+) __HAL_SPI_EMUL_SET_FLAG :  Set the specified SPI Emulation  flag

     [..]
       (@) You can refer to the SPI Emulation HAL driver header file for more useful macros

  @endverbatim
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_spi_master_emul.h"

/** @addtogroup STM32F4xx_HAL_SPI_MASTER_EMUL_Driver
  * @{
  */

/** @defgroup SPI_EMUL_HAL_Driver
  * @brief HAL SPI Emulation module driver
  * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Timer handler declaration */
static TIM_HandleTypeDef  TimHandle;

/* DMA Handle declaration Tx */
static DMA_HandleTypeDef  hdma_tx;

/* DMA Handle declaration Rx*/
static DMA_HandleTypeDef  hdma_rx;

/* Timer Output Compare Configuration Structure declaration */
static TIM_OC_InitTypeDef       sConfig;
static TIM_OC_InitTypeDef       sConfig1;

/* SPI Emulation Handle */
static SPI_Emul_HandleTypeDef      *hspi_emul;

/* Buffer used for transmission */
static uint32_t *pBuffer_Tx[4*(TX_BUFFER_SIZE)];

/* Buffer used for reception */
static uint32_t *pBuffer_Rx[4*(RX_BUFFER_SIZE)];

/* Private Generic function prototypes ------------------------------------------*/
static void SPI_Emul_SetConfig (SPI_Emul_HandleTypeDef *spi);
static void SPI_Emul_DMAError(DMA_HandleTypeDef *hdma);
static void SPI_Emul_EnableRessources(SPI_Emul_HandleTypeDef *hspi);

/* Private Tx function prototypes -----------------------------------------------*/
static void SPI_Emul_SetConfig_DMATx(void);
static void SPI_Emul_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_Emul_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_Emul_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_Emul_TransmitFormatFrame(SPI_Emul_HandleTypeDef *spi , uint16_t pData, uint32_t *pBuffer_tmp_Tx);
static void SPI_Emul_Transmission_Process_Complete(void);

/* Private Rx function prototypes -----------------------------------------------*/
static void SPI_Emul_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_Emul_SetConfig_DMARx(void);
static void SPI_Emul_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static uint32_t SPI_Emul_ReceiveFormatFrame(SPI_Emul_HandleTypeDef *hspi, uint32_t *pBuffer);
static void SPI_Emul_Reception_Process_Complete(void);

uint16_t TmpBuffer[0x10];
volatile uint32_t IrqTx = 0x00;
volatile uint32_t IrqRx = 0x00;
extern volatile uint32_t RestTx;
volatile uint32_t RestRx = 0x00;
volatile uint32_t LastTx = 0x00;

/** @defgroup SPI_EMUL_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the SPI Emulator:

(+) Call the function HAL_SPI_Emul_Init() to configure these parameters:
        (++) Mode
        (++) Direction
        (++) Data Size
        (++) Clock Polarity and Phase
        (++) NSS Management
        (++) Clock Speed
        (++) FirstBit
        (++) TIMode
        (++) CRC Calculation
        (++) CRC Polynomial if CRC enabled

        (+) Call the function HAL_SPI_DeInit() to restore the default configuration
          of the SPI Emulator.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the SPI Emulation according to the specified parameters
  *         in the SPI_Emul_InitTypeDef and create the associated handle.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *         the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Emul_Init(SPI_Emul_HandleTypeDef *hspi)
{
  /* Check the SPI handle allocation */
  if (hspi == NULL)
  {
    return HAL_ERROR;
  }
  if (hspi->State == HAL_SPI_EMUL_STATE_RESET)
  {
    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    HAL_SPI_Emul_MspInit(hspi);
  }
  /* Get Structure for spi emul Handle */
  hspi_emul = hspi;

  /* Set the TIM state */
  hspi->State = HAL_SPI_EMUL_STATE_BUSY;

  /* Set the SPI Emulation Communication parameters */
  SPI_Emul_SetConfig(hspi);

  /* Initialize the SPI Emulation state */
  hspi->ErrorCode = HAL_SPI_EMUL_ERROR_NONE;
  hspi->State = HAL_SPI_EMUL_STATE_READY;

  return HAL_OK;
}

/**
  * @brief  DeInitializes the SPI Emulation .
  * @param  hSPI: SPI Emulation handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Emul_DeInit(SPI_Emul_HandleTypeDef *hspi)
{
  /* Check the SPI handle allocation */
  if (hspi == NULL)
  {
    return HAL_ERROR;
  }

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
  HAL_SPI_Emul_MspDeInit(hspi);

  hspi->ErrorCode = HAL_SPI_EMUL_ERROR_NONE;
  hspi->State = HAL_SPI_EMUL_STATE_RESET;

  return HAL_OK;
}

/**
  * @brief SPI Emulation MSP Init.
  * @param  hspi: SPI Emulation Handle.
  * @retval None
  */
__weak void HAL_SPI_Emul_MspInit(SPI_Emul_HandleTypeDef *hspi)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
           the HAL_SPI_Emul_MspInit could be implemented in the user file
  */
}

/**
  * @brief SPI Emulation MSP DeInit
  * @param  hspi: SPI Emulation handle
  * @retval None
  */
__weak void HAL_SPI_Emul_MspDeInit(SPI_Emul_HandleTypeDef *hspi)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPI_Emul_MspDeInit could be implemented in the user file
   */
}

/**
  * @}
  */
/**
  * @brief  Configures the SPI Emulation peripheral.
             + Enable clock for ALL peripheral Timer, GPIO
             + DMA 2 Configuration channel, Stream, Mode? Priority, ...
             + Configuration SPI Emulation CR registre software
  * @param  hspi: SPI Emulation handle
  * @retval None
  */
static void SPI_Emul_SetConfig (SPI_Emul_HandleTypeDef *hspi)
{
  uint32_t bit_time = 0;

  /* Check the parameters */
  assert_param(IS_SPI_EMUL_MODE(hspi->Init.Mode));
  assert_param(IS_SPI_EMUL_DIRECTION(hspi->Init.Direction));
  assert_param(IS_SPI_EMUL_DATASIZE(hspi->Init.DataSize));
  assert_param(IS_SPI_EMUL_POLARITY(hspi->Init.CLKPolarity));
  assert_param(IS_SPI_EMUL_PHASE(hspi->Init.CLKPhase));
  assert_param(IS_SPI_CLK(hspi->Init.SPI_Clk));
  assert_param(IS_SPI_EMUL_FIRSTBIT(hspi->Init.FirstBit));

  /* Init Bit Time */
  if ((HAL_RCC_GetSysClockFreq() / HAL_RCC_GetPCLK2Freq() == 1))
  {
    bit_time = ((uint32_t) ((HAL_RCC_GetSysClockFreq() / hspi->Init.SPI_Clk) - 1));
  }
  else
  {
    bit_time = ((uint32_t) (((HAL_RCC_GetPCLK2Freq() * 2) / hspi->Init.SPI_Clk) - 1));
  }

  /*##-1- Configure  the Timer peripheral (TIM1) in Bit Delay ##############*/
  /* Initialize TIM1 peripheral as follow:
  + Period = TimerPeriod 
  + Prescaler = 0
  + ClockDivision = 0
  + Counter direction = Up
  */
  TimHandle.Instance            = TIM1;
  TimHandle.Init.Prescaler      = 0;
  TimHandle.Init.ClockDivision  = 0;
  TimHandle.Init.CounterMode    = TIM_COUNTERMODE_UP;
  HAL_TIM_PWM_Init(&TimHandle);

  /*##-2- Configure the PWM channel 1 #########################################*/
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_LOW;
  sConfig.OCFastMode   = TIM_OCFAST_ENABLE;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);

  /*##-2- Configure the PWM channel 2 #########################################*/
  sConfig1.OCMode       = TIM_OCMODE_PWM1;
  if (hspi->Init.CLKPolarity == SPI_EMUL_POLARITY_LOW)
  {
    sConfig1.OCPolarity   = TIM_OCPOLARITY_HIGH;
  }
  else if (hspi->Init.CLKPolarity == SPI_EMUL_POLARITY_HIGH)
  {
    sConfig1.OCPolarity   = TIM_OCPOLARITY_LOW;

    if (hspi->Init.ClkPinNumber == GPIO_PIN_9)
    {
      (hspi_emul->ClkPortName)->PUPDR |= 0x00040000;
    }
    else /* hspi_emul->ClkPinName == GPIO_PIN_8 */
    {
      GPIOA->PUPDR |= 0x00010000;
    }
  }
  sConfig1.OCFastMode   = TIM_OCFAST_ENABLE;
  sConfig1.OCIdleState  = TIM_OCIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig1, TIM_CHANNEL_2);

  TIMx->ARR |= bit_time;
  TIMx->CCR2 |= bit_time / 2;

  if (hspi->Init.CLKPhase == SPI_EMUL_PHASE_2EDGE)
  {
    TIMx->CCR1 |= bit_time;
  }
  else if (hspi->Init.CLKPhase == SPI_EMUL_PHASE_1EDGE)
  {
    TIMx->CCR1 |= bit_time / 2;
  }

  if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_TX) && (hspi->Init.Mode == SPI_EMUL_MODE_MASTER))
  {
    /* Configure SPI Emulation in Transmission mode */
    SPI_Emul_SetConfig_DMATx();
  }
  else if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_RX) && (hspi->Init.Mode == SPI_EMUL_MODE_MASTER))
  {
    /* Configure SPI Emulation in Reception mode */
    SPI_Emul_SetConfig_DMARx();
  }
}

/**
  * @brief  Configures the DMA for SPI Emulation transmission.
             + DMA 2 Configuration channel, Stream, Mode? Priority, ...
  * @param  None
  * @retval None
  */
static void SPI_Emul_SetConfig_DMATx(void)
{

  /*##-1- Configure  DMA For SPI Emulation TX #############################*/
  /* Set the parameters to be configured */
  hdma_tx.Init.Channel              = DMA_CHANNEL_6;                /* DMA_CHANNEL_6                        */
  hdma_tx.Init.Direction            = DMA_MEMORY_TO_PERIPH;         /* Transfer mode                        */
  hdma_tx.Init.PeriphInc            = DMA_PINC_DISABLE;             /* Peripheral increment mode Disable    */
  hdma_tx.Init.MemInc               = DMA_MINC_ENABLE;              /* Memory increment mode Enable         */
  hdma_tx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_WORD ;         /* Peripheral data alignment : Word     */
  hdma_tx.Init.MemDataAlignment     = DMA_MDATAALIGN_WORD ;         /* memory data alignment :  Word        */
  hdma_tx.Init.Mode                 = DMA_CIRCULAR;                 /* Circular DMA mode                    */
  hdma_tx.Init.Priority             = DMA_PRIORITY_HIGH;            /* priority level : high                */
  hdma_tx.Init.FIFOMode             = DMA_FIFOMODE_DISABLE;         /* FIFO mode disabled                   */
  hdma_tx.Init.FIFOThreshold        = DMA_FIFO_THRESHOLD_FULL;      /* FIFO threshold full configuration    */
  hdma_tx.Init.MemBurst             = DMA_MBURST_SINGLE;            /* Memory burst                         */
  hdma_tx.Init.PeriphBurst          = DMA_PBURST_SINGLE;            /* Peripheral burst                     */

  /* Set hdma_tim instance */
  hdma_tx.Instance = DMA2_Stream1;
  hdma_tx.Parent = TimHandle.hdma[1];

  /* Link hdma_tim to hdma[ ] ( channel Tx or Rx) */
  __HAL_LINKDMA(&TimHandle, hdma[1] , hdma_tx);

  /* Initialize TIMx DMA handle */
  HAL_DMA_Init(TimHandle.hdma[1]);

  /*##-2- NVIC configuration for DMA transfer complete interrupt ###########*/
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  /* Configure DMA Stream source and destination address */
  hdma_tx.Instance->PAR =  (uint32_t) & ((hspi_emul->TxPortName)->BSRR);
  hdma_tx.Instance->M0AR = (uint32_t)pBuffer_Tx;

  /* Configure DMA Stream data length */
  if ((hspi_emul->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi_emul->Init.DataSize == SPI_EMUL_DATASIZE_16BIT))
  {
    hdma_tx.Instance->NDTR = NUMBER_OF_DATA_ITEMS;
  }
  else
  {
    if ((hspi_emul->TxXferSize) < 20)
    {
      hdma_tx.Instance->NDTR = 2 * (hspi_emul->TxXferSize) * (hspi_emul->Init.DataSize);
    }
    else
    {
      hdma_tx.Instance->NDTR = NUMBER_OF_DATA_ITEMS / 2;
    }
  }
}

/**
 * @brief  Sends an amount of data
 * @param  hspi: SPI Emulation handle
 * @param  pData: Pointer to data buffer
 * @param  Size: Amount of data to be sent
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_SPI_Emul_Transmit_DMA(SPI_Emul_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  uint32_t counter_format = 0;
  uint32_t counter_shift = 0;
  uint16_t pData_16bit = 0;
  uint32_t test_hal_spi_state_ready = HAL_SPI_STATE_READY;

  if ((hspi->State == test_hal_spi_state_ready) || (hspi->State == HAL_SPI_EMUL_STATE_BUSY_RX))
  {
    if ((pData == NULL) || (Size == 0))
    {
      return  HAL_ERROR;
    }
    /* Configure communication */
    hspi->ErrorCode   = HAL_SPI_EMUL_ERROR_NONE;

    hspi->TxXferSize  = Size;
    hspi->pTxBuffPtr  = pData;
    hspi->TxXferCount = 0;

    /* Check if a receive process is ongoing or not */
    if (hspi->State == HAL_SPI_EMUL_STATE_BUSY_RX)
    {
      hspi->State = HAL_SPI_EMUL_STATE_BUSY_TX_RX;
    }
    else
    {
      hspi->State = HAL_SPI_EMUL_STATE_BUSY_TX;
    }
    /* Set the SPI TxDMA transfer complete callback */
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferCpltCallback = SPI_Emul_DMATransmitCplt;

    /* Set the SPI TxDMA half transfer callback */
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = SPI_Emul_DMAHalfTransmitCplt;

    /* Set the DMA error callback */
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferErrorCallback = SPI_Emul_DMAError;

    if ((hspi_emul->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi_emul->Init.DataSize == SPI_EMUL_DATASIZE_16BIT))
    {
      for (counter_format = 0; counter_format < 20; counter_format++)
      {
        pData_16bit = (pData[counter_shift+1] << ONE_BYTE) | pData[counter_shift];
        SPI_Emul_TransmitFormatFrame(hspi, pData_16bit, (uint32_t*)(pBuffer_Tx + counter_format*16));
        hspi_emul->TxXferCount++;
        counter_shift += 2;
      }
    }
    else
    {
      for (counter_format = 0; counter_format < 20; counter_format++)
      {
        SPI_Emul_TransmitFormatFrame(hspi, *(pData + counter_format), (uint32_t*)(pBuffer_Tx + counter_format*8));
        hspi_emul->TxXferCount++;
      }
    }

    SPI_Emul_EnableRessources(hspi);
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  This function is executed to enable transmission ressources
  * @param  hspi: SPI Emulation handle
  * @retval None
  */
static void SPI_Emul_EnableRessources(SPI_Emul_HandleTypeDef *hspi)
{
  if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) || (hspi->Init.Direction == SPI_EMUL_DIRECTION_RX))
  {
    /* Enable the transfer complete interrupt (Reception) */
    __HAL_DMA_ENABLE_IT(&hdma_rx, DMA_IT_TC);

    /* Enable the transfer complete interrupt (Reception) */
    __HAL_DMA_ENABLE_IT(&hdma_rx, DMA_IT_HT);

    /* Enable the transfer Error interrupt (Reception) */
    __HAL_DMA_ENABLE_IT(&hdma_rx, DMA_IT_TE);

    /* Enable the TIM Update DMA request (Reception) */
    __HAL_TIM_ENABLE_DMA(&TimHandle, TIM_DMA_CC2);
  }
  if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) || (hspi->Init.Direction == SPI_EMUL_DIRECTION_TX))
  {
    /* Enable the transfer complete interrupt (Transmission) */
    __HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_TC);

    /* Enable the half transfer complete interrupt (Transmission) */
    __HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_HT);

    /* Enable the transfer Error interrupt (Transmission) */
    __HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_TE);

    /* Enable the TIM Update DMA request */
    __HAL_TIM_ENABLE_DMA(&TimHandle, TIM_DMA_CC1);
  }
  if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) || (hspi->Init.Direction == SPI_EMUL_DIRECTION_TX))
  {
    /* Enable the Peripheral */
    __HAL_DMA_ENABLE(&hdma_tx);
  }
  if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) || (hspi->Init.Direction == SPI_EMUL_DIRECTION_RX))
  {
    __HAL_DMA_ENABLE(&hdma_rx);
  }
  if ((hspi->Init.CLKPhase == SPI_EMUL_PHASE_1EDGE) && ((hspi->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) || (hspi->Init.Direction == SPI_EMUL_DIRECTION_RX)))
  {
    TIMx->EGR |= 0x004;
  }
  if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) || (hspi->Init.Direction == SPI_EMUL_DIRECTION_TX))
  {
    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_ENABLE);
    TIMx->EGR |= 0x002;
  }
  TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE);

  __HAL_TIM_MOE_ENABLE(&TimHandle);

  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(&TimHandle);

}

/**
  * @brief  This function is executed in case of Transfer Complete.
  * @param  hdma: DMA handle
  * @retval None
  */
static void SPI_Emul_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  uint8_t pData_8bit = 0;
  uint16_t pData_16bit = 0;
  __IO uint32_t counter_format = 0;
  uint32_t transfer_tx_size = hspi_emul->TxXferSize;
  uint32_t rest_transfer_tx = RestTx;

  if ((hspi_emul->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi_emul->Init.DataSize == SPI_EMUL_DATASIZE_16BIT))
  {
    if (hspi_emul->TxXferCount <= ((transfer_tx_size) - 10))
    {
      for (counter_format = 10; counter_format < 20; counter_format++)
      {
        pData_8bit = *(hspi_emul->pTxBuffPtr + (hspi_emul->TxXferCount++));
        pData_16bit = ((pData_8bit + 1) << ONE_BYTE) | pData_8bit;
        SPI_Emul_TransmitFormatFrame(hspi_emul, pData_16bit, (uint32_t*)&pBuffer_Tx[counter_format*(hspi_emul->Init.DataSize)]);
        hspi_emul->TxXferCount++;
      }
    }
    else
    {
      SPI_Emul_Transmission_Process_Complete();
      /* Handle for SPI Emulation Transfer Complete */
      HAL_SPI_Emul_TxCpltCallback(hspi_emul);
    }
  }
  else
  {
    if ((hspi_emul->TxXferSize) < 20)
    {
      SPI_Emul_Transmission_Process_Complete();
      HAL_SPI_Emul_TxCpltCallback(hspi_emul);
    }
    else
    {
      if (((hspi_emul->TxXferCount) + 10) <= (transfer_tx_size))
      {

        for (counter_format = 10; counter_format < 20; counter_format++)
        {
          pData_8bit = *(hspi_emul->pTxBuffPtr + (hspi_emul->TxXferCount++));
          SPI_Emul_TransmitFormatFrame(hspi_emul, pData_8bit , (uint32_t*)&pBuffer_Tx[counter_format*8]);
        }
      }
      else
      {
        if ((hspi_emul->TxXferCount) <= (transfer_tx_size))
        {
          RestTx = (transfer_tx_size) - (hspi_emul->TxXferCount);
          for (counter_format = 10; counter_format < rest_transfer_tx + 10; counter_format++)
          {
            pData_8bit = *(hspi_emul->pTxBuffPtr + (hspi_emul->TxXferCount++));
            SPI_Emul_TransmitFormatFrame(hspi_emul, pData_8bit, (uint32_t*)&pBuffer_Tx[counter_format*8]);
          }
          LastTx = 1;
        }
        else
        {

          if (LastTx == 1)
          {
            SPI_Emul_Transmission_Process_Complete();
            HAL_SPI_Emul_TxCpltCallback(hspi_emul);
            LastTx = 0;
          }
          else
          {
            LastTx = 1;
          }
        }
      }
    }
  }
}



/**
  * @brief  This function is executed in case of Half Transfer Complete.
  * @param  hdma: DMA handle
  * @retval None
  */
static void SPI_Emul_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma)
{

  uint8_t pData_8bit = 0;
  uint16_t pData_16bit = 0;
  __IO uint32_t counter_format = 0;
  uint32_t transfer_tx_size = hspi_emul->TxXferSize;
  uint32_t rest_transfer_tx = RestTx;

  if ((hspi_emul->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi_emul->Init.DataSize == SPI_EMUL_DATASIZE_16BIT))
  {
    if (hspi_emul->TxXferCount <= transfer_tx_size)
    {
      for (counter_format = 0; counter_format < 10; counter_format++)
      {
        pData_8bit = *(hspi_emul->pTxBuffPtr + (hspi_emul->TxXferCount++));
        pData_16bit = ((pData_8bit + 1) << ONE_BYTE) | pData_8bit;
        SPI_Emul_TransmitFormatFrame(hspi_emul, pData_16bit , (uint32_t*)&pBuffer_Tx[counter_format*16]);
        hspi_emul->TxXferCount++;
      }
    }
    else
    {
      SPI_Emul_Transmission_Process_Complete();
      HAL_SPI_Emul_TxHalfCpltCallback(hspi_emul);
    }
  }
  else
  {
    if (((hspi_emul->TxXferCount) + 10) <= (transfer_tx_size))
    {

      for (counter_format = 0; counter_format < 10; counter_format++)
      {
        pData_8bit = *(hspi_emul->pTxBuffPtr + (hspi_emul->TxXferCount++));
        SPI_Emul_TransmitFormatFrame(hspi_emul, pData_8bit, (uint32_t*)&pBuffer_Tx[counter_format*8]);
      }
    }
    else
    {
      if (((hspi_emul->TxXferCount)) <= (transfer_tx_size))
      {
        RestTx = ((transfer_tx_size) - (hspi_emul->TxXferCount));
        for (counter_format = 0; counter_format < rest_transfer_tx; counter_format++)
        {
          pData_8bit = *(hspi_emul->pTxBuffPtr + (hspi_emul->TxXferCount++));
          SPI_Emul_TransmitFormatFrame(hspi_emul, pData_8bit, (uint32_t*)&pBuffer_Tx[counter_format*8]);
        }
        LastTx = 1;
      }
      else
      {
        if (LastTx == 1)
        {
          SPI_Emul_Transmission_Process_Complete();
          HAL_SPI_Emul_TxCpltCallback(hspi_emul);
          LastTx = 0;
        }
        else
        {
          LastTx = 1;
        }
      }
    }
  }
}


/**
  * @brief  This function disables ressources
  * @param  None
  * @retval None
  */
static void SPI_Emul_Transmission_Process_Complete(void)
{
  HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2);

  __HAL_DMA_DISABLE_IT(TimHandle.hdma[TIM_DMA_ID_CC1], DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(TimHandle.hdma[TIM_DMA_ID_CC1], DMA_IT_TC);

  if (hspi_emul->Init.Direction == SPI_EMUL_DIRECTION_TX)
  {
    /* Set TC flag in the SR registre software */
    __HAL_SPI_EMUL_SET_FLAG(hspi_emul, SPI_EMUL_FLAG_TC);
  }
  /* De_Initialize counter frame for Tx */
  hspi_emul->TxXferCount = 0;


  /* Disable the Capture compare channel */
  TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);

  if ((hspi_emul->Init.Direction == SPI_EMUL_DIRECTION_TX) && (hspi_emul->Init.Mode == SPI_EMUL_MODE_MASTER))
  {
    TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_DISABLE);
    __HAL_TIM_MOE_DISABLE(&TimHandle);
    __HAL_TIM_DISABLE(&TimHandle);
  }
  __HAL_DMA_DISABLE(TimHandle.hdma[TIM_DMA_ID_CC1]);

  /* Initialize the SPI Emulation state */
  hspi_emul->ErrorCode = HAL_SPI_EMUL_ERROR_NONE;

  /* Check if a receive process is ongoing or not */
  if (hspi_emul->State == HAL_SPI_EMUL_STATE_BUSY_TX_RX)
  {
    hspi_emul->State = HAL_SPI_EMUL_STATE_BUSY_RX;
  }
  else
  {
    hspi_emul->State = HAL_SPI_EMUL_STATE_READY;
  }
}

/**
  * @brief  This function formatted one Frame
  * @param  SPI Emulation Handle
  * @param  pdata pointer in data
  * @retval None
  */
static void SPI_Emul_TransmitFormatFrame(SPI_Emul_HandleTypeDef *hspi, uint16_t Data, uint32_t *pBuffer_tmp_Tx)
{
  uint32_t counter = 0;
  uint32_t bitmask = 0;

  /* Get the Pin Number */
  bitmask = (uint32_t) hspi->Init.TxPinNumber;

  if (hspi->Init.FirstBit == SPI_EMUL_FIRSTBIT_LSB)
  {
    for (counter = 0; counter < 0x08 ; counter++)
    {
      if (((Data >> counter)&BitMask_LSB) != 0)
      {
        pBuffer_tmp_Tx[counter] = bitmask;
      }
      else
      {
        pBuffer_tmp_Tx[counter] = (bitmask << 16);
      }
    }
  }
  else if ((hspi->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi->Init.DataSize == SPI_EMUL_DATASIZE_8BIT))
  {
    for (counter = 0; counter < 0x08; counter++)
    {
      if (((Data << counter)&BitMask_MSB_8bit) != 0)
      {
        pBuffer_tmp_Tx[counter] = bitmask;
      }
      else
      {
        pBuffer_tmp_Tx[counter] = (bitmask << 16);
      }
    }
  }
  else if ((hspi->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi->Init.DataSize == SPI_EMUL_DATASIZE_16BIT))
  {
    for (counter = 0; counter < 0x10; counter++)
    {
      if (((Data << counter)&BitMask_MSB_16bit) != 0)
      {
        pBuffer_tmp_Tx[counter] = bitmask;
      }
      else
      {
        pBuffer_tmp_Tx[counter] = (bitmask << 16);
      }
    }
  }
}

/**
* @brief  This function handles DMA interrupt request for TC.
* @param  None
* @retval None
*/
void SPI_EMUL_TX_DMA_IRQHandler(void)
{
  if ((IrqTx == 0) && (hspi_emul->TxXferSize > 20))
  {
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback(TimHandle.hdma[TIM_DMA_ID_CC1]);
    __HAL_DMA_CLEAR_FLAG(TimHandle.hdma[TIM_DMA_ID_CC1], __HAL_DMA_GET_HT_FLAG_INDEX(TimHandle.hdma[TIM_DMA_ID_CC1]));
    IrqTx = 1;
    if (RestTx != 0x00)
    {
      __HAL_DMA_DISABLE(&hdma_tx);
      hdma_tx.Instance->NDTR = RestTx * (hspi_emul->Init.DataSize);
      __HAL_DMA_ENABLE(&hdma_tx);
    }
  }
  /* Transfer complete callback */
  else
  {
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferCpltCallback(TimHandle.hdma[TIM_DMA_ID_CC1]);
    __HAL_DMA_CLEAR_FLAG(TimHandle.hdma[TIM_DMA_ID_CC1], __HAL_DMA_GET_TC_FLAG_INDEX(TimHandle.hdma[TIM_DMA_ID_CC1]));
    IrqTx = 0;
  }
}
/**
  * @brief  Returns the SPI Emulation state.
  * @param  hspi: SPI Emulation handle
  * @retval HAL state
  */
HAL_SPI_Emul_StateTypeDef HAL_SPI_Emul_GetState(SPI_Emul_HandleTypeDef *hspi)
{
  return hspi->State;
}

/**
  * @brief  This function is executed in case of error of Transfer occurrence.
  * @param  hdma : DMA Handle
  * @retval None
  */
static void SPI_Emul_DMAError(DMA_HandleTypeDef *hdma)
{
  /* SPI Emulation frame error occurred */
  __HAL_SPI_EMUL_SET_FLAG(hspi_emul, SPI_EMUL_FLAG_FE);

  hspi_emul->ErrorCode |= HAL_SPI_EMUL_ERROR_FE;

  HAL_SPI_Emul_ErrorCallback(hspi_emul);
}

/**
  * @brief  This function handles SPI Emulation request.
  * @param  hspi: SPI Emulation handle
  * @retval None
  */
void HAL_SPI_Emul_IRQHandler(SPI_Emul_HandleTypeDef *hspi)
{
  uint32_t flag = 0;
  flag = __HAL_SPI_EMUL_GET_FLAG(hspi, SPI_EMUL_FLAG_FE);

  /* SPI Emulation frame error occurred */
  if (flag != RESET)
  {
    __HAL_SPI_EMUL_CLEAR_FLAG(hspi, SPI_EMUL_FLAG_FE);

    hspi->ErrorCode |= HAL_SPI_EMUL_ERROR_FE;
  }

  flag = __HAL_SPI_EMUL_GET_FLAG(hspi, SPI_EMUL_FLAG_TC);

  /* SPI Emulation in mode Transmitter */
  if ((flag != RESET))
  {
    __HAL_SPI_EMUL_CLEAR_FLAG(hspi, SPI_EMUL_FLAG_TC);
  }

  flag = __HAL_SPI_EMUL_GET_FLAG(hspi, SPI_EMUL_FLAG_RC);

  /* SPI Emulation in mode Transmitter */
  if ((flag != RESET))
  {
    __HAL_SPI_EMUL_CLEAR_FLAG(hspi, SPI_EMUL_FLAG_RC);
  }

  if (hspi->ErrorCode != HAL_SPI_EMUL_ERROR_NONE)
  {
    /* Set the SPI Emulation state ready to be able to start again the process */
    hspi->State = HAL_SPI_EMUL_STATE_READY;

    HAL_SPI_Emul_ErrorCallback(hspi);
  }
}

/**
  * @}
  */

/**
  * @brief  Initializes the SPI Emulation Transfer Complete.
  * @param  hspi: SPI Emulation Handle
  * @retval None
  */
__weak void HAL_SPI_Emul_TxCpltCallback(SPI_Emul_HandleTypeDef *hspi)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPI_Emul_TransferComplete could be implemented in the user file
   */
}

/**
  * @brief  Initializes the SPI Emulation Half Transfer Complete.
  * @param  hspi: SPI Emulation Handle
  * @retval None
  */
__weak void HAL_SPI_Emul_TxHalfCpltCallback(SPI_Emul_HandleTypeDef *hspi)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPI_Emul_TransferComplete could be implemented in the user file
   */
}

/**
  * @brief  SPI Emulation error callbacks.
  * @param  hspi: SPI Emulation handle
  * @retval None
  */
__weak void HAL_SPI_Emul_ErrorCallback(SPI_Emul_HandleTypeDef *hspi)
{
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_SPI_ErrorCallback could be implemented in the user file
   */
}

/*********************************************************************/
/*********************************************************************/
/***************************Reception Process*************************/
/*********************************************************************/
/*********************************************************************/

/**
  * @brief  Configures the DMA for SPI Emulation transmission.
             + DMA 2 Configuration channel, Stream, Mode? Priority, ...
  * @param  None
  * @retval None
  */
static void SPI_Emul_SetConfig_DMARx(void)
{
  /*##-1- Configure  DMA For SPI Emulation RX #############################*/
  /* Set the parameters to be configured */
  hdma_rx.Init.Channel              = DMA_CHANNEL_6;                /* DMA_CHANNEL_6                        */
  hdma_rx.Init.Direction            = DMA_PERIPH_TO_MEMORY;         /* Transfer mode                        */
  hdma_rx.Init.PeriphInc            = DMA_PINC_DISABLE;             /* Peripheral increment mode Disable    */
  hdma_rx.Init.MemInc               = DMA_MINC_ENABLE;              /* Memory increment mode Enable         */
  hdma_rx.Init.PeriphDataAlignment  = DMA_PDATAALIGN_WORD ;         /* Peripheral data alignment : Word     */
  hdma_rx.Init.MemDataAlignment     = DMA_MDATAALIGN_WORD ;         /* memory data alignment :  Word        */
  hdma_rx.Init.Mode                 = DMA_CIRCULAR;                 /* Circular DMA mode                    */
  hdma_rx.Init.Priority             = DMA_PRIORITY_VERY_HIGH;       /* priority level : very high           */
  hdma_rx.Init.FIFOMode             = DMA_FIFOMODE_DISABLE;         /* FIFO mode disabled                   */
  hdma_rx.Init.FIFOThreshold        = DMA_FIFO_THRESHOLD_FULL;      /* FIFO threshold full configuration    */
  hdma_rx.Init.MemBurst             = DMA_MBURST_SINGLE;            /* Memory burst                         */
  hdma_rx.Init.PeriphBurst          = DMA_PBURST_SINGLE;            /* Peripheral burst                     */

  /* Set hdma_tim instance */
  hdma_rx.Instance = DMA2_Stream2;
  hdma_rx.Parent = TimHandle.hdma[2];

  /* Link hdma_tim to hdma[ ] ( channel Tx or Rx) */
  __HAL_LINKDMA(&TimHandle, hdma[2] , hdma_rx);

  /* Initialize TIMx DMA handle */
  HAL_DMA_Init(TimHandle.hdma[2]);

  /*##-2- NVIC configuration for DMA transfer complete interrupt ###########*/
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);


  /* Configure DMA Stream source and destination address */
  hdma_rx.Instance->PAR = (uint32_t) & ((hspi_emul->RxPortName)->IDR);
  hdma_rx.Instance->M0AR  = (uint32_t)pBuffer_Rx;

  /* Configure DMA Stream data length */
  if ((hspi_emul->RxXferSize) < 20)
  {
    hdma_rx.Instance->NDTR = 2 * (hspi_emul->RxXferSize) * (hspi_emul->Init.DataSize);
  }
  else
  {
    hdma_rx.Instance->NDTR = 2 * 10 * (hspi_emul->Init.DataSize);
  }
}

/**
 * @brief  Receives an amount of data
 * @param  hspi: SPI Emulation handle
 * @param  pData: Pointer to data buffer
 * @param  Size: Amount of data to be sent
 * @retval HAL status
*/
HAL_StatusTypeDef HAL_SPI_Emul_Receive_DMA(SPI_Emul_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  uint32_t test_hal_spi_state_ready = HAL_SPI_STATE_READY;

  if ((hspi->State == test_hal_spi_state_ready) || (hspi->State == HAL_SPI_EMUL_STATE_BUSY_TX))
  {
    if ((pData == NULL) || (Size == 0))
    {
      return  HAL_ERROR;
    }

    /* Configure communication */
    hspi->ErrorCode   = HAL_SPI_EMUL_ERROR_NONE;

    hspi->RxXferSize  = Size;
    hspi->pRxBuffPtr  = pData;
    hspi->RxXferCount = 0;

    /* Check if a receive process is ongoing or not */
    if (hspi->State == HAL_SPI_EMUL_STATE_BUSY_TX)
    {
      hspi->State = HAL_SPI_EMUL_STATE_BUSY_TX_RX;
    }
    else
    {
      hspi->State = HAL_SPI_EMUL_STATE_BUSY_RX;
    }

    /* Set the SPI TxDMA transfer complete callback */
    TimHandle.hdma[TIM_DMA_ID_CC2]->XferCpltCallback = SPI_Emul_DMAReceiveCplt;

    /* Set the SPI TxDMA transfer half callback */
    TimHandle.hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback = SPI_Emul_DMAHalfReceiveCplt;

    /* Set the DMA error callback */
    TimHandle.hdma[TIM_DMA_ID_CC2]->XferErrorCallback = SPI_Emul_DMAError;

    SPI_Emul_EnableRessources(hspi);
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief  This function is executed in case of Receive Complete for one Frame.
  * @param  hdma : DMA Handle
  * @retval None
  */
static void SPI_Emul_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  uint32_t counter_format = 0;
  uint32_t transfer_rx_size = hspi_emul->RxXferSize;

  if ((hspi_emul->RxXferSize) < 20)
  {
    SPI_Emul_Reception_Process_Complete();
    for (counter_format = 0; counter_format < (hspi_emul->RxXferSize); counter_format++)
    {
      hspi_emul->pRxBuffPtr[hspi_emul->RxXferCount] = SPI_Emul_ReceiveFormatFrame(hspi_emul, (uint32_t*) & pBuffer_Rx[(hspi_emul->Init.DataSize)*counter_format]);
      hspi_emul->RxXferCount ++;
    }

    HAL_SPI_Emul_RxHalfCpltCallback(hspi_emul);
  }

  else
  {
    if ((hspi_emul->RxXferCount + 10) <= (transfer_rx_size))
    {
      for (counter_format = (10); counter_format < (20); counter_format++)
      {
        hspi_emul->pRxBuffPtr[hspi_emul->RxXferCount] = SPI_Emul_ReceiveFormatFrame(hspi_emul, (uint32_t*) & pBuffer_Rx[(hspi_emul->Init.DataSize)*counter_format]);
        hspi_emul->RxXferCount ++;
      }
    }
    else if ((hspi_emul->RxXferCount ) < (transfer_rx_size))
    {
      RestRx = ((transfer_rx_size) - (hspi_emul->RxXferCount));
      for (counter_format = (10); counter_format < (RestRx + 10); counter_format++)
      {
        hspi_emul->pRxBuffPtr[hspi_emul->RxXferCount] = SPI_Emul_ReceiveFormatFrame(hspi_emul, (uint32_t*) & pBuffer_Rx[(hspi_emul->Init.DataSize)*counter_format]);
        hspi_emul->RxXferCount ++;
      }
    }
    else
    {
      SPI_Emul_Reception_Process_Complete();

      /* Handle for SPI Emulation Transfer Complete */
      HAL_SPI_Emul_RxCpltCallback(hspi_emul);
    }
  }
}
/**
  * @brief  This function is executed in case of Transfer Half for one Frame.
  * @param  hdma : DMA Handle
  * @retval None
  */
static void SPI_Emul_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma)
{
  uint8_t counter_format = 0;
  uint32_t transfer_rx_size = hspi_emul->RxXferSize;

  if ((hspi_emul->RxXferCount + 10) <= (transfer_rx_size))
  {
    for (counter_format = (0); counter_format < (10); counter_format++)
    {
      hspi_emul->pRxBuffPtr[hspi_emul->RxXferCount] = SPI_Emul_ReceiveFormatFrame(hspi_emul, (uint32_t*) & pBuffer_Rx[(hspi_emul->Init.DataSize)*counter_format]);
      hspi_emul->RxXferCount ++;
    }
  }
  else if ((hspi_emul->RxXferCount) < (transfer_rx_size))
  {
    RestRx = ((transfer_rx_size) - (hspi_emul->RxXferCount));
    for (counter_format = (0); counter_format < (RestRx); counter_format++)
    {
      hspi_emul->pRxBuffPtr[hspi_emul->RxXferCount] = SPI_Emul_ReceiveFormatFrame(hspi_emul, (uint32_t*) & pBuffer_Rx[(hspi_emul->Init.DataSize)*counter_format]);
      hspi_emul->RxXferCount ++;
    }
  }
  else
  {
    SPI_Emul_Reception_Process_Complete();

    HAL_SPI_Emul_RxHalfCpltCallback(hspi_emul);
  }
}

/**
  * @brief  This function disable ressources
  * @param  None
  * @retval None
  */
static void SPI_Emul_Reception_Process_Complete(void)
{
  HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2);
  __HAL_DMA_DISABLE_IT(TimHandle.hdma[TIM_DMA_ID_CC2], DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(TimHandle.hdma[TIM_DMA_ID_CC2], DMA_IT_TC);

  /* Set TC flag in the SR registre software */
  __HAL_SPI_EMUL_SET_FLAG(hspi_emul, SPI_EMUL_FLAG_RC);
  __HAL_SPI_EMUL_SET_FLAG(hspi_emul, SPI_EMUL_FLAG_TC);/////HALIM


  __HAL_DMA_DISABLE(TimHandle.hdma[TIM_DMA_ID_CC2]);

  /* Disable the Capture compare channel */
  TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_DISABLE);
  __HAL_TIM_MOE_DISABLE(&TimHandle);
  __HAL_TIM_DISABLE(&TimHandle);
  __HAL_DMA_DISABLE(TimHandle.hdma[TIM_DMA_ID_CC2]);

  /* Initialize the SPI Emulation state */
  hspi_emul->ErrorCode = HAL_SPI_EMUL_ERROR_NONE;

  /* Check if a receive process is ongoing or not */
  if (hspi_emul->State == HAL_SPI_EMUL_STATE_BUSY_TX_RX)
  {
    hspi_emul->State = HAL_SPI_EMUL_STATE_BUSY_TX;
  }
  else
  {
    hspi_emul->State = HAL_SPI_EMUL_STATE_READY;
  }
}

/**
* @brief  Format Frame in Receiver mode.
* @param  hspi: SPI Emulation handle
* @param  pBuffer: pointer of receiver Buffer
* @param  pFrame: pointer of Frame
* @retval None
*/
static uint32_t SPI_Emul_ReceiveFormatFrame(SPI_Emul_HandleTypeDef *hspi, uint32_t *pBuffer_tmp_Rx)
{
  uint32_t counter = 0;
  uint32_t length = 0;
  uint32_t received_data = 0;

  /* Get Length of frame */
  length = hspi->Init.DataSize;

  if (hspi->Init.FirstBit == SPI_EMUL_FIRSTBIT_LSB)
  {
    for (counter = 0; counter < length ; counter++)
    {
      if ((pBuffer_tmp_Rx[counter]&(hspi->Init.RxPinNumber)) == (hspi->Init.RxPinNumber))
      {
        received_data = (BitMask_LSB << counter) | received_data;
      }
    }
  }
  else if (hspi->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB)
  {
    for (counter = 0; counter < length; counter++)
    {
      if ((pBuffer_tmp_Rx[counter]&(hspi->Init.RxPinNumber)) == (hspi->Init.RxPinNumber))
      {
        if (hspi->Init.DataSize == SPI_EMUL_DATASIZE_8BIT)
        {
          received_data = (BitMask_MSB_8bit >> counter) | received_data;
        }
        else if (hspi->Init.DataSize == SPI_EMUL_DATASIZE_16BIT)
        {
          received_data = (BitMask_MSB_16bit >> counter) | received_data;
        }
      }
    }
  }
  return received_data;
}

/**
* @brief  This function handles DMA interrupt request for TC.
* @param  None
* @retval None
*/
void SPI_EMUL_RX_DMA_IRQHandler(void)
{
  if ((IrqRx == 0) && (hspi_emul->RxXferSize > 20))
  {
    TimHandle.hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback(TimHandle.hdma[TIM_DMA_ID_CC2]);
    __HAL_DMA_CLEAR_FLAG(TimHandle.hdma[TIM_DMA_ID_CC2], __HAL_DMA_GET_HT_FLAG_INDEX(TimHandle.hdma[TIM_DMA_ID_CC2]));
    IrqRx = 1;
    if (RestRx != 0x00)
    {
      __HAL_DMA_DISABLE(&hdma_rx);
      hdma_rx.Instance->NDTR = RestRx * (hspi_emul->Init.DataSize);
      __HAL_DMA_ENABLE(&hdma_rx);
    }
  }
  /* Transfer complete callback */
  else
  {
    TimHandle.hdma[TIM_DMA_ID_CC2]->XferCpltCallback(TimHandle.hdma[TIM_DMA_ID_CC2]);
    __HAL_DMA_CLEAR_FLAG(TimHandle.hdma[TIM_DMA_ID_CC2], __HAL_DMA_GET_TC_FLAG_INDEX(TimHandle.hdma[TIM_DMA_ID_CC2]));
    IrqRx = 0;
    if (RestRx != 0x00)
    {
      __HAL_DMA_DISABLE(&hdma_rx);
      hdma_rx.Instance->NDTR = RestRx * (hspi_emul->Init.DataSize);
      __HAL_DMA_ENABLE(&hdma_rx);
    }
  }
}

/**
  * @}
  */

/**
  * @brief  Initializes the SPI Emulation Receive Complete.
  * @param  hspi: SPI Emulation Handle
  * @retval None
  */
__weak void HAL_SPI_Emul_RxCpltCallback(SPI_Emul_HandleTypeDef *hspi)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPI_Emul_TransferComplet could be implemented in the user file
   */
}

/**
  * @brief  Initializes the SPI Emulation Half Receive Complete.
  * @param  hspi: SPI Emulation Handle
  * @retval None
  */
__weak void HAL_SPI_Emul_RxHalfCpltCallback(SPI_Emul_HandleTypeDef *hspi)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPI_Emul_TransferComplete could be implemented in the user file
   */
}

/**
  * @brief  Transmit and Receive an amount of data in no-blocking mode with DMA
  * @param  hspi: pointer to a SPI_Emul_HandleTypeDef structure that contains
  *                the configuration information for SPI module.
  * @param  pTxData: pointer to transmission data buffer
  * @param  pRxData: pointer to reception data buffer
  * @param  Size: amount of data to be sent
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_SPI_Emul_TransmitReceive_DMA(SPI_Emul_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
  uint32_t counter_format = 0;
  uint32_t counter_shift = 0;
  uint16_t pData_16bit = 0;
  uint32_t tmpstate = 0;
  tmpstate = hspi->State;

  if ((tmpstate == HAL_SPI_STATE_READY) || ((hspi->Init.Mode == SPI_MODE_MASTER) && \
      (hspi->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) && (tmpstate == HAL_SPI_STATE_BUSY_RX)))
  {
    if ((pTxData == NULL ) || (pRxData == NULL ) || (Size == 0))
    {
      return  HAL_ERROR;
    }

    /* Check the parameters */
    assert_param(SPI_EMUL_DIRECTION_TX_RX(hspi->Init.Direction));

    /* Configure communication */
    hspi->ErrorCode   = HAL_SPI_EMUL_ERROR_NONE;

    hspi->TxXferSize  = Size;
    hspi->RxXferSize  = Size;
    hspi->pTxBuffPtr  = pTxData;
    hspi->pRxBuffPtr  = pRxData;
    hspi->TxXferCount = 0;

    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if (hspi->State != HAL_SPI_EMUL_STATE_BUSY_RX)
    {
      hspi->State = HAL_SPI_EMUL_STATE_BUSY_TX_RX;
    }
    SPI_Emul_SetConfig_DMATx();
    SPI_Emul_SetConfig_DMARx();


    /* Set the SPI TxDMA transfer complete callback */
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferCpltCallback = SPI_Emul_DMATransmitCplt;

    /* Set the SPI TxDMA transfer half callback */
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = SPI_Emul_DMAHalfTransmitCplt;

    /* Set the DMA error callback */
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferErrorCallback = SPI_Emul_DMAError;

    /* Set the SPI RxDMA transfer complete callback */
    TimHandle.hdma[TIM_DMA_ID_CC2]->XferCpltCallback = SPI_Emul_DMAReceiveCplt;

    /* Set the SPI RxDMA transfer half callback */
    TimHandle.hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback = SPI_Emul_DMAHalfReceiveCplt;

    /* Set the DMA error callback */
    TimHandle.hdma[TIM_DMA_ID_CC2]->XferErrorCallback = SPI_Emul_DMAError;

    if ((hspi_emul->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi_emul->Init.DataSize == SPI_EMUL_DATASIZE_16BIT))
    {
      for (counter_format = 0; counter_format < 20; counter_format++)
      {
        pData_16bit = (pTxData[counter_shift+1] << ONE_BYTE) | pTxData[counter_shift];
        SPI_Emul_TransmitFormatFrame(hspi, pData_16bit, (uint32_t*)(pBuffer_Tx + counter_format*16));
        hspi_emul->TxXferCount++;
        counter_shift += 2;
      }
    }
    else
    {
      if (hspi->TxXferSize < 20)
      {
        for (counter_format = 0; counter_format < (hspi->TxXferSize); counter_format++)
        {
          SPI_Emul_TransmitFormatFrame(hspi, *(pTxData + counter_format), (uint32_t*)(pBuffer_Tx + counter_format*8));
          hspi_emul->TxXferCount++;
        }
      }
      else
      {
        for (counter_format = 0; counter_format < 20; counter_format++)
        {
          SPI_Emul_TransmitFormatFrame(hspi, *(pTxData + counter_format), (uint32_t*)(pBuffer_Tx + counter_format*8));
          hspi_emul->TxXferCount++;
        }

      }
    }
    SPI_Emul_EnableRessources(hspi);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
