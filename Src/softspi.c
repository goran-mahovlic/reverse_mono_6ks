/**
 * @file    softspi.c
 * @author  Myth
 * @version 0.2
 * @date    2021.10.12
 * @brief   STM32 SoftSPI Library
 */

#include "softspi.h"

uint8_t SPI_Delay = 1;

#define SCLK_Set HAL_GPIO_WritePin(SoftSPIx->SCLK_GPIO, SoftSPIx->SCLK_Pin, GPIO_PIN_SET)
#define SCLK_Clr HAL_GPIO_WritePin(SoftSPIx->SCLK_GPIO, SoftSPIx->SCLK_Pin, GPIO_PIN_RESET)

#define MOSI_Set HAL_GPIO_WritePin(SoftSPIx->MOSI_GPIO, SoftSPIx->MOSI_Pin, GPIO_PIN_SET)
#define MOSI_Clr HAL_GPIO_WritePin(SoftSPIx->MOSI_GPIO, SoftSPIx->MOSI_Pin, GPIO_PIN_RESET)

#define MISO_Read HAL_GPIO_ReadPin(SoftSPIx->MISO_GPIO, SoftSPIx->MISO_Pin)

#define SS_Set HAL_GPIO_WritePin(SoftSPIx->SS_GPIO, SoftSPIx->SS_Pin, GPIO_PIN_SET)
#define SS_Clr HAL_GPIO_WritePin(SoftSPIx->SS_GPIO, SoftSPIx->SS_Pin, GPIO_PIN_RESET)

#define Delay SoftSPI_Delay_us(SoftSPIx->Delay_Time)

uint8_t SoftSPI_GPIOx_Pin_Init(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode, uint32_t Pull);

HAL_StatusTypeDef SoftSPI_Init(SoftSPI_TypeDef *SoftSPIx)
{
    SoftSPIx->SCLK_GPIO = SoftSPIx->SCLK_GPIO;
    SoftSPIx->SCLK_Pin = SoftSPIx->SCLK_Pin;

    if (!SoftSPI_GPIOx_Pin_Init(SoftSPIx->SCLK_GPIO, SoftSPIx->SCLK_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL))
        return HAL_ERROR;

    SoftSPIx->MOSI_GPIO = SoftSPIx->MOSI_GPIO;
    SoftSPIx->MOSI_Pin = SoftSPIx->MOSI_Pin;

    if (!SoftSPI_GPIOx_Pin_Init(SoftSPIx->MOSI_GPIO, SoftSPIx->MOSI_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL))
        return HAL_ERROR;

    SoftSPIx->MISO_GPIO = SoftSPIx->MISO_GPIO;
    SoftSPIx->MISO_Pin = SoftSPIx->MISO_Pin;

    if (!SoftSPI_GPIOx_Pin_Init(SoftSPIx->MISO_GPIO, SoftSPIx->MISO_Pin, GPIO_MODE_INPUT, GPIO_NOPULL))
        return HAL_ERROR;

    SoftSPIx->SS_GPIO = SoftSPIx->SS_GPIO;
    SoftSPIx->SS_Pin = SoftSPIx->SS_Pin;

    if (!SoftSPI_GPIOx_Pin_Init(SoftSPIx->SS_GPIO, SoftSPIx->SS_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL))
        return HAL_ERROR;

    SoftSPIx->Delay_Time = SoftSPIx->Delay_Time;

    return HAL_OK;
}

void SoftSPI_SetSS(SoftSPI_TypeDef *SoftSPIx)
{
    SS_Set;
}

void SoftSPI_ClrSS(SoftSPI_TypeDef *SoftSPIx)
{
    SS_Clr;
}

uint8_t SoftSPI_WriteRead(SoftSPI_TypeDef *SoftSPIx, uint8_t byte)
{
    uint8_t data = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        SCLK_Clr;
        Delay_us(SPI_Delay);

        if (byte & 0x80)
            MOSI_Set;
        else
            MOSI_Clr;

        Delay_us(SPI_Delay);
        byte <<= 1;
        SCLK_Set;
        Delay_us(SPI_Delay);

        data <<= 1;
        if (MISO_Read == GPIO_PIN_SET){
            data |= 0x01;
            HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
            }
        else{
            HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
        }

        Delay_us(SPI_Delay);
    }

    return data;
}

uint16_t readSPI(uint8_t  command) {
    int result = 0;

    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(TS_DIN_GPIO_Port, TS_DIN_Pin, command & (1 << i));
        HAL_GPIO_WritePin(TS_CLK_GPIO_Port, TS_CLK_Pin, GPIO_PIN_SET);
        Delay_us(10);
        HAL_GPIO_WritePin(TS_CLK_GPIO_Port, TS_CLK_Pin, GPIO_PIN_RESET);
        Delay_us(10);
    }

    for (int i = 11; i >= 0; i--) {
        HAL_GPIO_WritePin(TS_CLK_GPIO_Port, TS_CLK_Pin, GPIO_PIN_SET);
        Delay_us(10);
        result |= (HAL_GPIO_ReadPin(TS_DOUT_GPIO_Port, TS_DOUT_Pin) << i);
        HAL_GPIO_WritePin(TS_CLK_GPIO_Port, TS_CLK_Pin, GPIO_PIN_RESET);
        Delay_us(10);
    }

    return result;
}

void SoftSPI_WriteBuffer(SoftSPI_TypeDef *SoftSPIx, uint8_t *pWrite, uint32_t len)
{
    //uint8_t data;
    uint8_t byte;
    uint8_t i, j;

    for (j = 0; j < len; j++)
    {
        //data = 0;
        byte = pWrite[j];

        for (i = 0; i < 8; i++)
        {
            SCLK_Clr;
            Delay_us(SPI_Delay);
            if (byte & 0x80)
                MOSI_Set;
            else
                MOSI_Clr;

        Delay_us(SPI_Delay);        }
    }
}

void SoftSPI_WriteReadBuff(SoftSPI_TypeDef *SoftSPIx, uint8_t *pWrite, uint8_t *pRead, uint32_t len)
{
    uint8_t data;
    uint8_t byte;
    uint8_t i, j;

    for (j = 0; j < len; j++)
    {
        data = 0;
        byte = pWrite[j];

        for (i = 0; i < 8; i++)
        {
            SCLK_Clr;
            Delay_us(SPI_Delay);
            if (byte & 0x80)
                MOSI_Set;
            else
                MOSI_Clr;

            Delay_us(SPI_Delay);
            byte <<= 1;
            SCLK_Set;
            Delay_us(SPI_Delay);
            data <<= 1;
            if (MISO_Read == GPIO_PIN_SET){
            data |= 0x01;
            HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
            }
            else{
                HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
            }

            Delay_us(SPI_Delay);        }

        pRead[j] = data;
    }

}

uint8_t SoftSPI_GPIOx_Pin_Init(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode, uint32_t Pull)
{
    switch ((uint32_t)(GPIOx))
    {
    case (uint32_t)GPIOA:
    {
        GPIO_InitTypeDef GPIO_Initure;
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_Initure.Pin = Pin;
        GPIO_Initure.Mode = Mode;
        GPIO_Initure.Pull = Pull;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_Initure);
    }
    break;

    case (uint32_t)GPIOB:
    {
        GPIO_InitTypeDef GPIO_Initure;
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_Initure.Pin = Pin;
        GPIO_Initure.Mode = Mode;
        GPIO_Initure.Pull = Pull;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_Initure);
    }
    break;

    case (uint32_t)GPIOC:
    {
        GPIO_InitTypeDef GPIO_Initure;
        __HAL_RCC_GPIOC_CLK_ENABLE();

        GPIO_Initure.Pin = Pin;
        GPIO_Initure.Mode = Mode;
        GPIO_Initure.Pull = Pull;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &GPIO_Initure);
    }
    break;

    case (uint32_t)GPIOD:
    {
        GPIO_InitTypeDef GPIO_Initure;
        __HAL_RCC_GPIOD_CLK_ENABLE();

        GPIO_Initure.Pin = Pin;
        GPIO_Initure.Mode = Mode;
        GPIO_Initure.Pull = Pull;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOD, &GPIO_Initure);
    }
    case (uint32_t)GPIOE:
    {
        GPIO_InitTypeDef GPIO_Initure;
        __HAL_RCC_GPIOD_CLK_ENABLE();

        GPIO_Initure.Pin = Pin;
        GPIO_Initure.Mode = Mode;
        GPIO_Initure.Pull = Pull;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOE, &GPIO_Initure);
    }
    case (uint32_t)GPIOF:
    {
        GPIO_InitTypeDef GPIO_Initure;
        __HAL_RCC_GPIOD_CLK_ENABLE();

        GPIO_Initure.Pin = Pin;
        GPIO_Initure.Mode = Mode;
        GPIO_Initure.Pull = Pull;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOF, &GPIO_Initure);
    }
     case (uint32_t)GPIOG:
    {
        GPIO_InitTypeDef GPIO_Initure;
        __HAL_RCC_GPIOD_CLK_ENABLE();

        GPIO_Initure.Pin = Pin;
        GPIO_Initure.Mode = Mode;
        GPIO_Initure.Pull = Pull;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOG, &GPIO_Initure);
    }       
    break;

    default:
        return 0;
    }

    return 1;
}
