/**
  ******************************************************************************
  * @file    stm32f4xx_hal_spi_slave_emul.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-July-2015
  * @brief   SPI Emulation HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Serial Peripheral Interface (SPI Emulation):
  *           + Initialization and de-initialization functions
  *           + IO operation functions
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
#include "stm32f4xx_spi_slave_emul.h"

/** @addtogroup STM32F4xx_HAL_SPI_SLAVE_EMUL_Driver
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

/* Timer Input Capture Configuration Structure declaration */
static TIM_IC_InitTypeDef       sConfig;
static TIM_IC_InitTypeDef       sConfig1;

/* SPI Emulation Handle */
static SPI_Emul_HandleTypeDef      *hspi_emul;

/* Buffer used for transmission */
static uint32_t *pBuffer_Tx[4*(TX_BUFFER_SIZE)];

/* Buffer used for reception */
static uint32_t *pBuffer_Rx[4*(RX_BUFFER_SIZE)];

/* Private Generic function prototypes -----------------------------------------------*/
static void SPI_Emul_SetConfig (SPI_Emul_HandleTypeDef *spi);
static void SPI_Emul_DMAError(DMA_HandleTypeDef *hdma);
static void SPI_Emul_EnableRessources(void);

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

volatile uint32_t IrqTx = 0x00;
volatile uint32_t IrqRx = 0x00;

/* Private functions ---------------------------------------------------------*/

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
  * @param  hspi: SPI Emulation handle
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
  /* Check the parameters */
  assert_param(IS_SPI_EMUL_MODE(hspi->Init.Mode));
  assert_param(IS_SPI_EMUL_DIRECTION(hspi->Init.Direction));
  assert_param(IS_SPI_EMUL_DATASIZE(hspi->Init.DataSize));
  assert_param(IS_SPI_EMUL_POLARITY(hspi->Init.CLKPolarity));
  assert_param(IS_SPI_EMUL_PHASE(hspi->Init.CLKPhase));
  assert_param(IS_SPI_CLK(hspi->Init.SPI_Clk));
  assert_param(IS_SPI_EMUL_FIRSTBIT(hspi->Init.FirstBit));

  /* Initialize TIM1 peripheral as follow:
  + Period = TimerPeriod 
  + Prescaler = 0
  + ClockDivision = 0
  + Counter direction = Up
  */

  TimHandle.Instance            = TIM1;
  TimHandle.Init.Period         = 0xFFFF;
  TimHandle.Init.Prescaler      = 0;
  TimHandle.Init.ClockDivision  = 0;
  TimHandle.Init.CounterMode    = TIM_COUNTERMODE_UP;
  HAL_TIM_IC_Init(&TimHandle);

  sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sConfig.ICFilter    = 0;
  HAL_TIM_IC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2);

  if (hspi->Init.CLKPhase == SPI_EMUL_PHASE_1EDGE)
  {
    sConfig1.ICPolarity  = TIM_ICPOLARITY_FALLING;
    sConfig.ICPolarity  = TIM_ICPOLARITY_FALLING;
  }
  else if (hspi->Init.CLKPhase == SPI_EMUL_PHASE_2EDGE)
  {
    sConfig1.ICPolarity  = TIM_ICPOLARITY_RISING;
    sConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  }

  sConfig1.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfig1.ICPrescaler = TIM_ICPSC_DIV1;
  sConfig1.ICFilter    = 0;
  HAL_TIM_IC_ConfigChannel(&TimHandle, &sConfig1, TIM_CHANNEL_1);

  if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_TX) && (hspi->Init.Mode == SPI_EMUL_MODE_SLAVE))
  {
    /* Configure SPI Emulation in Transmission mode */
    SPI_Emul_SetConfig_DMATx();
  }
  else if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_RX) && (hspi->Init.Mode == SPI_EMUL_MODE_SLAVE))
  {
    /* Configure SPI Emulation in Reception mode */
    SPI_Emul_SetConfig_DMARx();
  }
  else if ((hspi->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) && (hspi->Init.Mode == SPI_EMUL_MODE_SLAVE))
  {
    /* Configure SPI Emulation in full-duplex mode */
    SPI_Emul_SetConfig_DMATx();
    SPI_Emul_SetConfig_DMARx();
  }
}

/*********************************************************************/
/*********************************************************************/
/***************************Transmission Process**********************/
/*********************************************************************/
/*********************************************************************/

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

  /* Configure DMA Stream source and destination address */
  hdma_tx.Instance->PAR =  (uint32_t) & ((hspi_emul->RxPortName)->BSRR);
  hdma_tx.Instance->M0AR = (uint32_t) & pBuffer_Tx[0];

  /*##-2- NVIC configuration for DMA transfer complete interrupt ###########*/
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  /* Configure DMA Stream data length */
  if ((hspi_emul->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi_emul->Init.DataSize == SPI_EMUL_DATASIZE_16BIT))
  {
    hdma_tx.Instance->NDTR = NUMBER_OF_DATA_ITEMS;
  }
  else
  {
    hdma_tx.Instance->NDTR = NUMBER_OF_DATA_ITEMS / 2;
  }
}

/**
 * @brief  Receives an amount of data
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

    SPI_Emul_SetConfig_DMATx();

    /* Set the SPI TxDMA transfer complete callback */
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferCpltCallback = SPI_Emul_DMATransmitCplt;

    /* Set the SPI TxDMA transfer half callback */
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
    SPI_Emul_EnableRessources();

    if (hspi->Init.CLKPhase == SPI_EMUL_PHASE_1EDGE)
    {
      TIMx->EGR |= 0x02;
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  This function is executed to enable reception ressources
  * @param  None
  * @retval None
  */
static void SPI_Emul_EnableRessources()
{
  if ((hspi_emul->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) || (hspi_emul->Init.Direction == SPI_EMUL_DIRECTION_RX))
  {
    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE);

    /* Enable the Peripheral */
    __HAL_TIM_ENABLE(&TimHandle);

    /* Enable the transfer complete interrupt */
    __HAL_DMA_ENABLE_IT(&hdma_rx, DMA_IT_TC);

    /* Enable the half transfer interrupt */
    __HAL_DMA_ENABLE_IT(&hdma_rx, DMA_IT_HT);

    /* Enable the transfer Error interrupt */
    __HAL_DMA_ENABLE_IT(&hdma_rx, DMA_IT_TE);

    /* Enable the TIM Update DMA request */
    __HAL_TIM_ENABLE_DMA(&TimHandle, TIM_DMA_CC2);

    __HAL_DMA_ENABLE(&hdma_rx);
  }
  if ((hspi_emul->Init.Direction == SPI_EMUL_DIRECTION_TX_RX) || (hspi_emul->Init.Direction == SPI_EMUL_DIRECTION_TX))
  {
    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_ENABLE);

    /* Enable the Peripheral */
    __HAL_TIM_ENABLE(&TimHandle);

    /* Enable the transfer complete interrupt */
    __HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_TC);

    /* Enable the half transfer interrupt */
    __HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_HT);

    /* Enable the transfer Error interrupt */
    __HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_TE);

    /* Enable the TIM Update DMA request */
    __HAL_TIM_ENABLE_DMA(&TimHandle, TIM_DMA_CC1);

    __HAL_DMA_ENABLE(&hdma_tx);
  }
}

/**
  * @brief  This function is executed in case of Receive Complete for one Frame.
  * @param  None
  * @retval None
  */
static void SPI_Emul_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  uint8_t pData_8bit = 0;
  uint16_t pData_16bit = 0;
  uint32_t counter_format = 0;

  if ((hspi_emul->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi_emul->Init.DataSize == SPI_EMUL_DATASIZE_16BIT))
  {
    if (hspi_emul->TxXferCount <= hspi_emul->TxXferSize - 10)
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
    if (hspi_emul->TxXferCount <= hspi_emul->TxXferSize)
    {
      for (counter_format = 10; counter_format < 20; counter_format++)
      {
        pData_8bit = *(hspi_emul->pTxBuffPtr + (hspi_emul->TxXferCount++));
        SPI_Emul_TransmitFormatFrame(hspi_emul, pData_8bit , (uint32_t*)&pBuffer_Tx[counter_format*8]);
      }
    }
    else
    {
      SPI_Emul_Transmission_Process_Complete();
      /* Handle for SPI Emulation Transfer Complete */
      HAL_SPI_Emul_TxCpltCallback(hspi_emul);
    }
  }
}

/**
  * @brief  This function is executed in case of Half Transfer Complete.
  * @param  None
  * @retval None
  */
static void SPI_Emul_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma)
{
  uint8_t pData_8bit = 0;
  uint16_t pData_16bit = 0;
  uint32_t counter_format = 0;

  if ((hspi_emul->Init.FirstBit == SPI_EMUL_FIRSTBIT_MSB) && (hspi_emul->Init.DataSize == SPI_EMUL_DATASIZE_16BIT))
  {
    if (hspi_emul->TxXferCount <= hspi_emul->TxXferSize - 10)
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
    if (hspi_emul->TxXferCount <= hspi_emul->TxXferSize)
    {
      for (counter_format = 0; counter_format < 10; counter_format++)
      {
        pData_8bit = *(hspi_emul->pTxBuffPtr + (hspi_emul->TxXferCount++));
        SPI_Emul_TransmitFormatFrame(hspi_emul, pData_8bit, (uint32_t*)&pBuffer_Tx[counter_format*8]);
      }
    }
    else
    {
      SPI_Emul_Transmission_Process_Complete();
      HAL_SPI_Emul_TxCpltCallback(hspi_emul);
    }
  }
}

/**
  * @brief  This function disable ressources
  * @param  None
  * @retval None
  */
static void SPI_Emul_Transmission_Process_Complete(void)
{
  /* Disable the Input Capture channel */
  TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_DISABLE);

  /* Disable the Peripheral */
  __HAL_TIM_DISABLE(&TimHandle);

  hspi_emul->TxXferCount = 0;

  /* Set TC flag in the SR registre software */
  __HAL_SPI_EMUL_SET_FLAG(hspi_emul, SPI_EMUL_FLAG_TC);


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
 * @brief  Format Frame in Transmit mode.
 * @param  hspi: SPI Emulation handle
 * @param  pBuffer: pointer of receiver Buffer
 * @param  pFrame: pointer of Frame
 * @retval None
*/
static void SPI_Emul_TransmitFormatFrame(SPI_Emul_HandleTypeDef *hspi , uint16_t Data, uint32_t *pBuffer_tmp_Tx)
{
  uint32_t counter = 0;
  uint32_t bitmask = 0;

  /* Get the Pin Number */
  bitmask = (uint32_t)hspi->Init.TxPinNumber;

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
  if (IrqTx == 0)
  {
    TimHandle.hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback(TimHandle.hdma[TIM_DMA_ID_CC1]);
    __HAL_DMA_CLEAR_FLAG(TimHandle.hdma[TIM_DMA_ID_CC1], __HAL_DMA_GET_HT_FLAG_INDEX(TimHandle.hdma[TIM_DMA_ID_CC1]));
    IrqTx = 1;
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

  if (hspi->ErrorCode != HAL_SPI_EMUL_ERROR_NONE)
  {
    /* Set the SPI Emulation state ready to be able to start again the process */
    hspi->State = HAL_SPI_EMUL_STATE_READY;

    HAL_SPI_Emul_ErrorCallback(hspi);
  }
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

/**
  * @brief  Initializes the SPI Emulation Receive Complete.
  * @param  hspi: SPI Emulation Handle
  * @retval None
  */
__weak void HAL_SPI_Emul_TxCpltCallback(SPI_Emul_HandleTypeDef *hspi)
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
__weak void HAL_SPI_Emul_TxHalfCpltCallback(SPI_Emul_HandleTypeDef *hspi)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SPI_Emul_TransferComplete could be implemented in the user file
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
  hdma_rx.Init.Mode                 = DMA_CIRCULAR;                 /* Circular DMA mode                      */
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
  hdma_rx.Instance->NDTR = 2 * 10 * (hspi_emul->Init.DataSize);
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
    hspi->pTxBuffPtr  = pData;
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

    SPI_Emul_EnableRessources();
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  This function is executed in case of Receive Complete for one Frame.
  * @param  None
  * @retval None
  */
static void SPI_Emul_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  uint32_t counter_format = 0;
  if ((hspi_emul->RxXferCount + (10)) <= (hspi_emul->RxXferSize))
  {
    for (counter_format = (10); counter_format < (20); counter_format++)
    {
      hspi_emul->pRxBuffPtr[hspi_emul->RxXferCount] = SPI_Emul_ReceiveFormatFrame(hspi_emul, (uint32_t*) & pBuffer_Rx[(hspi_emul->Init.DataSize)*counter_format]);
      hspi_emul->RxXferCount ++;
    }
  }
  else
  {
    SPI_Emul_Reception_Process_Complete();
    HAL_SPI_Emul_RxCpltCallback(hspi_emul);
  }

}

/**
  * @brief  This function is executed in case of Transfer Half for one Frame.
  * @param  None
  * @retval None
  */
static void SPI_Emul_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma)
{
  uint32_t counter_format;
  if ((hspi_emul->RxXferCount + (10)) <= (hspi_emul->RxXferSize))
  {
    for (counter_format = 0; counter_format < (10); counter_format++)
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
static void SPI_Emul_Reception_Process_Complete()

{
  /* Disable the Input Capture channel */
  TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_DISABLE);

  /* Disable the Peripheral */
  __HAL_TIM_DISABLE(&TimHandle);

  /* Set TC flag in the SR registre software */
  __HAL_SPI_EMUL_SET_FLAG(hspi_emul, SPI_EMUL_FLAG_TC);


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
  if (IrqRx == 0)
  {

    TimHandle.hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback(TimHandle.hdma[TIM_DMA_ID_CC2]);
    __HAL_DMA_CLEAR_FLAG(TimHandle.hdma[TIM_DMA_ID_CC2], __HAL_DMA_GET_HT_FLAG_INDEX(TimHandle.hdma[TIM_DMA_ID_CC2]));
    IrqRx = 1;
  }
  /* Transfer complete callback */
  else
  {
    TimHandle.hdma[TIM_DMA_ID_CC2]->XferCpltCallback(TimHandle.hdma[TIM_DMA_ID_CC2]);

    __HAL_DMA_CLEAR_FLAG(TimHandle.hdma[TIM_DMA_ID_CC2], __HAL_DMA_GET_TC_FLAG_INDEX(TimHandle.hdma[TIM_DMA_ID_CC2]));
    IrqRx = 0;
  }
}

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
      for (counter_format = 0; counter_format < 20; counter_format++)
      {
        SPI_Emul_TransmitFormatFrame(hspi, *(pTxData + counter_format), (uint32_t*)(pBuffer_Tx + counter_format*8));
        hspi_emul->TxXferCount++;
      }
    }
    SPI_Emul_EnableRessources();

    if (hspi->Init.CLKPhase == SPI_EMUL_PHASE_1EDGE)
    {
      TIMx->EGR |= 0x02;
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
