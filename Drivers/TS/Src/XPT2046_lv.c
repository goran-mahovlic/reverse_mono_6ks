/**
 * @file XPT2046.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "XPT2046_lv.h"
#include "config.h"
#include "main.h"
#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"

#if USE_XPT2046

#include <stddef.h>
//#include LV_DRV_INDEV_INCLUDE
//#include LV_DRV_DELAY_INCLUDE

/*********************
 *      DEFINES
 *********************/
//#define CMD_X_READ  0b10010000
//#define CMD_Y_READ  0b11010000

#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)
 
#define Delay_us(us) \
    do { \
         uint32_t start = SysTick->VAL; \
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
         while((start - SysTick->VAL) < ticks); \
    } while (0)


#define DELAY_MS(ms) \
    do { \
        for (uint32_t i = 0; i < ms; ++i) { \
            Delay_us(1000); \
        } \
    } while (0)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void StartSoftSPI(void);

/**********************
 *  STATIC VARIABLES
 **********************/
int16_t avg_buf_x[XPT2046_AVG];
int16_t avg_buf_y[XPT2046_AVG];
uint8_t avg_last;

uint32_t avg_x = 0;
uint32_t avg_y = 0;
uint32_t raw_x = 0;
uint32_t raw_y = 0;


volatile uint16_t last_x = 0;
volatile uint16_t last_y = 0;
uint16_t x = 0;
uint16_t y = 0;
uint8_t last_state = 0;
uint8_t nsamples = 0;

extern volatile uint8_t touchPressed;

static SoftSPI_TypeDef soft_spi;

/* Параметры ориентации */
static touchOrienation _orient;
static uint16_t _width, _height;

//static void _XPT2046_TouchSelect(void);
//static void _XPT2046_TouchUnselect(void);
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize the XPT2046
 */
void xpt2046_lv_init(void)
{
    XPT2046_init(&soft_spi, XPT2046_LANDSCAPE, XPT2046_HOR_RES, XPT2046_VER_RES);
/*
for (int i=0;i<4;i++){
  HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
  DELAY_MS(1000);
  HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
  DELAY_MS(1000);
}
*/


}

/**
 * Get the current position and state of the touchpad
 * @param data store the read data here
 * @return false: because no ore data to be read
 */
#define READ_X 0x90
#define READ_Y 0xD0

static void StartSoftSPI(void){

  soft_spi.SCLK_GPIO=TS_CLK_GPIO_Port;
  soft_spi.SCLK_Pin=TS_CLK_Pin;

  //soft_spi.MISO_GPIO=D1_GPIO_Port;
  //soft_spi.MISO_Pin=D1_Pin;

  soft_spi.MOSI_GPIO=TS_DIN_GPIO_Port;
  soft_spi.MOSI_Pin=TS_DIN_Pin;

  soft_spi.MISO_GPIO=TS_DOUT_GPIO_Port;
  soft_spi.MISO_Pin=TS_DOUT_Pin;

  soft_spi.SS_GPIO=TS_CS_GPIO_Port;
  soft_spi.SS_Pin=TS_CS_Pin;



  soft_spi.Delay_Time=5;

  SoftSPI_Init(&soft_spi);

  HAL_GPIO_WritePin(TS_CS_GPIO_Port, TS_CS_Pin, GPIO_PIN_SET);
  DELAY_MS(10);
  HAL_GPIO_WritePin(TS_CS_GPIO_Port, TS_CS_Pin, GPIO_PIN_RESET);
  DELAY_MS(10);
}
void XPT2046_init(SoftSPI_TypeDef *spi, touchOrienation orientation, const uint16_t width, const uint16_t height) {
    StartSoftSPI();

	_orient = orientation;
	_width = width;
	_height = height;
}

//XPT2046_AVG
bool xpt2046_getXY(uint16_t* x, uint16_t* y)
{
    //last_state = LV_INDEV_STATE_PR;
    //_XPT2046_TouchSelect();
   
    nsamples = 0;
    avg_x = 0;
    avg_y = 0;
    for(uint8_t i = 0; i < XPT2046_AVG; i++)
    {  
        nsamples++;
        uint16_t y_raw;
        uint16_t x_raw;        
        HAL_GPIO_WritePin(TS_CS_GPIO_Port, TS_CS_Pin, GPIO_PIN_RESET);
        x_raw = readSPI(READ_X);
        y_raw = readSPI(READ_Y);
        HAL_GPIO_WritePin(TS_CS_GPIO_Port, TS_CS_Pin, GPIO_PIN_SET);
        
        avg_x += x_raw; //(((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
        avg_y += y_raw; //(((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
        DELAY_MS(1);
    }

    if(nsamples < XPT2046_AVG)
    {
        return false;
    }

    raw_x = (avg_x / XPT2046_AVG);
    raw_y = (avg_y / XPT2046_AVG);

    if(raw_x <  XPT2046_X_MIN) raw_x = XPT2046_X_MIN;
    if(raw_x >  XPT2046_X_MAX) raw_x = XPT2046_X_MAX;

    raw_y = (avg_y / XPT2046_AVG);
    if(raw_y < XPT2046_Y_MIN) raw_y = XPT2046_Y_MIN;
    if(raw_y > XPT2046_Y_MAX) raw_y = XPT2046_Y_MAX;

    #if XPT2046_XY_SWAP != 0
    int16_t swap_tmp;
    swap_tmp = *x;
    *x = *y;
    *y = swap_tmp;
    #endif

    *x = (raw_x - XPT2046_X_MIN) * XPT2046_HOR_RES / (XPT2046_X_MAX - XPT2046_X_MIN);
    *y = (raw_y - XPT2046_Y_MIN) * XPT2046_VER_RES / (XPT2046_Y_MAX - XPT2046_Y_MIN);
    #if XPT2046_X_INV != 0
         (*x) =  XPT2046_HOR_RES - (*x);
    #endif
    #if XPT2046_Y_INV != 0
        (*y) =  XPT2046_VER_RES - (*y);
    #endif
   last_x = (*x);
   last_y = (*y);

    return true;

}


void xpt2046_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    
    if(!touchPressed)
    {
        last_state = LV_INDEV_STATE_RELEASED;
    } 
    else{
        if(HAL_GPIO_ReadPin(TS_IRQ_GPIO_Port,TS_IRQ_Pin)){
            touchPressed = false;
        }
        else{
            xpt2046_getXY(&x, &y);
            data->point.x = (lv_coord_t)x;
            data->point.y = (lv_coord_t)y;
            data->state = LV_INDEV_STATE_PRESSED;// ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
            }
    }
    //return false;
}

#endif

/**********************
 *   STATIC FUNCTIONS
 **********************/