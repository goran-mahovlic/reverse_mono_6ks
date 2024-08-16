
//	MIT License
//
//	Copyright (c) 2017 Matej Artnak
//
//	Permission is hereby granted, free of charge, to any person obtaining a copy
//	of this software and associated documentation files (the "Software"), to deal
//	in the Software without restriction, including without limitation the rights
//	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//	copies of the Software, and to permit persons to whom the Software is
//	furnished to do so, subject to the following conditions:
//
//	The above copyright notice and this permission notice shall be included in all
//	copies or substantial portions of the Software.
//
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//	SOFTWARE.
//
//
//
//-----------------------------------
//	ILI9341 Driver library for STM32
//-----------------------------------
//
//	While there are other libraries for ILI9341 they mostly require either interrupts, DMA or both for fast drawing
//	The intent of this library is to offer a simple yet still reasonably fast alternatives for those that
//	do not wish to use interrupts or DMA in their projects.
//
//	Library is written for STM32 HAL library and supports STM32CUBEMX. To use the library with Cube software
//	you need to tick the box that generates peripheral initialization code in their own respective .c and .h file
//
//
//-----------------------------------
//	Performance
//-----------------------------------
//	Settings:	
//	--SPI @ 50MHz 
//	--STM32F746ZG Nucleo board
//	--Redraw entire screen
//
//	++		Theoretical maximum FPS with 50Mhz SPI calculated to be 40.69 FPS
//	++		320*240 = 76800 pixels, each pixel contains 16bit Color information (2x8)
//	++		Theoretical Max FPS: 1/((320*240*16)/50000000)
//
//	With ART Accelerator, instruction prefetch, CPI ICACHE and CPU DCACHE enabled:
//
//	-FPS:									39.62
//	-SPI utilization:			97.37%
//	-MB/Second:						6.09
//
//	With ART Accelerator, instruction prefetch, CPI ICACHE and CPU DCACHE disabled:
//
//	-FPS:									35.45
//	-SPI utilization:			87.12%
//	-MB/Second:						5.44
//	
//	ART Accelerator, instruction prefetch, CPI ICACHE and CPU DCACHE settings found in MXCUBE under "System-> CORTEX M7 button"
//
//
//
//-----------------------------------
//	How to use this library
//-----------------------------------
//
//	-generate SPI peripheral and 3 GPIO_SPEED_FREQ_VERY_HIGH GPIO outputs
//	 		++Library reinitializes GPIOs and SPIs generated by gpio.c/.h and spi.c/.h using MX_X_Init(); calls
//			++reinitialization will not clash with previous initialization so generated initializations can be laft as they are
//	-If using MCUs other than STM32F7 you will have to change the #include "stm32f4xx_hal.h" in the ILI9341_STM32_Driver.h to your respective .h file
//	-define your HSPI_INSTANCE in ILI9341_STM32_Driver.h
//	-define your CS, DC and RST outputs in ILI9341_STM32_Driver.h
//	-check if ILI9341_SCREEN_HEIGHT and ILI9341_SCREEN_WIDTH match your LCD size
//			++Library was written and tested for 320x240 screen size. Other sizes might have issues**
//	-in your main program initialize LCD with ILI9341_Init();
//	-library is now ready to be used. Driver library has only basic functions, for more advanced functions see ILI9341_GFX library	
//
//-----------------------------------

/* Includes ------------------------------------------------------------------*/
#include "ILI9341_STM32_Driver.h"
#include "config.h"
#include "lv_conf.h"
#include "../lvgl/lvgl.h"
#include "main.h"

//#define MONOX
#define SATURN
#define USE_DMA

/* Global Variables ------------------------------------------------------------------*/
volatile uint16_t LCD_HEIGHT = ILI9341_SCREEN_HEIGHT;
volatile uint16_t LCD_WIDTH	 = ILI9341_SCREEN_WIDTH;
/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;

uint16_t * lcdAddr = (uint16_t *) 0x60000004;

/*For LittlevGL*/
static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p);
void monitor_cb(lv_disp_drv_t * d, uint32_t t, uint32_t p);
static lv_disp_drv_t disp_drv;
static volatile int32_t x1_flush;
static volatile int32_t y1_flush;
static volatile int32_t x2_flush;
static volatile int32_t y2_flush;
static volatile int32_t y_flush_act;
static volatile const lv_color_t * buf_to_flush;

static volatile bool refr_qry;
static volatile uint32_t t_last = 0;

static void DMA_TransferComplete(DMA_HandleTypeDef *han);
static void DMA_TransferError(DMA_HandleTypeDef *han);
void DMA2_Stream0_IRQHandler(void);

void LV_tft_init(void)
{
	static lv_color_t buf[ILI9341_SCREEN_WIDTH * FB_SIZE];
	static lv_disp_draw_buf_t disp_buf;
	
	lv_disp_draw_buf_init(&disp_buf, buf, NULL, ILI9341_SCREEN_WIDTH * FB_SIZE);
    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Used to copy the buffer's content to the display*/
    /*Set a display buffer*/
    disp_drv.draw_buf = &disp_buf;	
	disp_drv.flush_cb = tft_flush_cb;
	disp_drv.monitor_cb = monitor_cb;
	disp_drv.hor_res = ILI9341_SCREEN_WIDTH;
	disp_drv.ver_res = ILI9341_SCREEN_HEIGHT;


#if LV_USE_GPU
    /*Fill a memory array with a color*/
    disp_drv.gpu_fill_cb = gpu_fill;
#endif

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

/**
 * Monitor refresh time
 * */
void monitor_cb(lv_disp_drv_t * d, uint32_t t, uint32_t p)
{
    t_last = t;
	uint32_t tmp = t;
    //lv_obj_invalidate(lv_scr_act());   /*Continuously refresh the whole screen*/
}

static void DMA_TransferComplete(DMA_HandleTypeDef *han)
{
	y_flush_act ++;
	if(y_flush_act > y2_flush) {	
			lv_disp_flush_ready(&disp_drv);

	} else {
	  buf_to_flush += x2_flush - x1_flush + 1;

	/*##-7- Start the DMA transfer using the interrupt mode ####################*/
	/* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	/* Enable All the DMA interrupts */
	HAL_StatusTypeDef err;

	ILI9341_Set_Address(x1_flush, y_flush_act, x2_flush, y_flush_act+1);
	HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_SET);
	err = HAL_DMA_Start_IT(han,(uint32_t)buf_to_flush, (uint32_t)lcdAddr, (x2_flush - x1_flush + 1));

	if( err != HAL_OK)
	  {
	    while(1);
	  }

	}
}
static void DMA_TransferError(DMA_HandleTypeDef *han)
{
    while(1);
}

#ifdef USE_DMA
static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
	int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
	int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
	int32_t act_x2 = area->x2 > ILI9341_SCREEN_WIDTH - 1 ? ILI9341_SCREEN_WIDTH - 1 : area->x2;
	int32_t act_y2 = area->y2 > ILI9341_SCREEN_HEIGHT - 1 ? ILI9341_SCREEN_HEIGHT - 1 : area->y2;
	
	x1_flush = act_x1;
	y1_flush = act_y1;
	x2_flush = act_x2;
	y2_flush = act_y2;
	y_flush_act = act_y1;
	buf_to_flush = color_p;

	/*Use DMA instead of DMA2D to leave it free for GPU*/
	HAL_StatusTypeDef err;
	//HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_RESET);
    ILI9341_Set_Address(x1_flush, y_flush_act, x2_flush, y_flush_act + 1);
	HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_SET);
	err = HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0,(uint32_t)buf_to_flush, (uint32_t)lcdAddr, (x2_flush - x1_flush + 1));	
	
	if(err != HAL_OK)
	{
		while(1);	
	}

}


#else

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

static void tft_flush_cb(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
        /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/
		uint16_t x, y;
		uint16_t temp2 = 0;
		lv_color_t temp;
		temp = *color_p;
		for(y = area->y1; y <= area->y2; y++)
		{
			for(x = area->x1; x <= area->x2; x++)
			{
			temp = *color_p;
			temp2 = temp.full;
			ILI9341_Draw_Pixel(x, y, temp2);
			color_p++;
			}
		}

		lv_disp_flush_ready(&disp_drv);

}
#endif

/*****************************************************************************
 * @name       :void ILI9341_Write_Command(uint16_t data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void ILI9341_Write_Command(uint16_t data)
{ 
	HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_RESET);
	//HAL_Delay(1);
	LCD->LCD_REG=data; 
}

/*****************************************************************************
 * @name       :void ILI9341_Write_Data(uint16_t data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void ILI9341_Write_Data(uint16_t data)
{
	HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);
	LCD->LCD_RAM=data; 
}


void lcdDelay(uint32_t delayms)
{
		HAL_Delay(delayms);
/*
	#ifdef USERTOS
	osDelay(delayms);
	#else
HAL_Delay(delayms);
	#endif
	*/
}

/* Set Address - Location block - to draw into */
void ILI9341_Set_Address(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{
ILI9341_Write_Command(0x2A);
ILI9341_Write_Data(X1>>8);
ILI9341_Write_Data(X1);
ILI9341_Write_Data(X2>>8);
ILI9341_Write_Data(X2);

ILI9341_Write_Command(0x2B);
ILI9341_Write_Data(Y1>>8);
ILI9341_Write_Data(Y1);
ILI9341_Write_Data(Y2>>8);
ILI9341_Write_Data(Y2);

ILI9341_Write_Command(0x2C);
}

/*HARDWARE RESET*/
void ILI9341_Reset(void)
{
	//HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_RESET);
	//ILI9341_Write_Command(0x01); //only software for blackboard
	// 

	//lcdDelay(500);
	//HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_SET);
	
HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
//lcdDelay(200);
//HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
lcdDelay(200);
HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);	

}

/*Ser rotation of the screen - changes x0 and y0*/
void ILI9341_Set_Rotation(uint8_t Rotation) 
{
	
uint8_t screen_rotation = Rotation;

ILI9341_Write_Command(0x36);
lcdDelay(1);
	
switch(screen_rotation) 
	{
		case SCREEN_VERTICAL_1:
			ILI9341_Write_Data(0x40|0x08);
			LCD_WIDTH = 320;
			LCD_HEIGHT = 480;
			break;
		case SCREEN_HORIZONTAL_1:
			ILI9341_Write_Data(0x20|0x08);
			LCD_WIDTH  = 480;
			LCD_HEIGHT = 320;
			break;
		case SCREEN_VERTICAL_2:
			ILI9341_Write_Data(0x80|0x08);
			LCD_WIDTH  = 320;
			LCD_HEIGHT = 480;
			break;
		case SCREEN_HORIZONTAL_2:
			ILI9341_Write_Data(0x40|0x80|0x20|0x08);
			LCD_WIDTH  = 480;
			LCD_HEIGHT = 320;
			break;
		default:
			//EXIT IF SCREEN ROTATION NOT VALID!
			break;
	}
}

/*Enable LCD display*/
void ILI9341_Enable(void)
{
HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
}

/*Initialize LCD display*/
void ILI9341_Init(void)
{
//DMA_Init();
  HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream0, HAL_DMA_XFER_CPLT_CB_ID, DMA_TransferComplete);
  HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream0, HAL_DMA_XFER_ERROR_CB_ID, DMA_TransferError);

ILI9341_Enable();
//ILI9341_SPI_Init();
ILI9341_Reset();

//SOFTWARE RESET
ILI9341_Write_Command(0x01);
lcdDelay(500);

#ifdef MONOX

//POWER CONTROL B
ILI9341_Write_Command(0xCF);
ILI9341_Write_Data(0x00);
ILI9341_Write_Data(0xC1);
ILI9341_Write_Data(0x30);

//POWER ON SEQUENCE CONTROL
ILI9341_Write_Command(0xED);
ILI9341_Write_Data(0x64);
ILI9341_Write_Data(0x03);
ILI9341_Write_Data(0x12);
ILI9341_Write_Data(0x81);

//DRIVER TIMING CONTROL A
ILI9341_Write_Command(0xE8);
ILI9341_Write_Data(0x85);
ILI9341_Write_Data(0x10);
ILI9341_Write_Data(0x7A);

//POWER CONTROL A
ILI9341_Write_Command(0xCB);
ILI9341_Write_Data(0x39);
ILI9341_Write_Data(0x2C);
ILI9341_Write_Data(0x00);
ILI9341_Write_Data(0x34);
ILI9341_Write_Data(0x02);

//PUMP RATIO CONTROL
ILI9341_Write_Command(0xF7);
ILI9341_Write_Data(0x20);

//DRIVER TIMING CONTROL B
ILI9341_Write_Command(0xEA);
ILI9341_Write_Data(0x00);
ILI9341_Write_Data(0x00);

//POWER CONTROL,VRH[5:0]
ILI9341_Write_Command(0xC0);
ILI9341_Write_Data(0x1B);

//POWER CONTROL,SAP[2:0];BT[3:0]
ILI9341_Write_Command(0xC1);
ILI9341_Write_Data(0x01);

//VCM CONTROL
ILI9341_Write_Command(0xC5);
ILI9341_Write_Data(0x30);
ILI9341_Write_Data(0x30);

//VCM CONTROL 2
ILI9341_Write_Command(0xC7);
ILI9341_Write_Data(0xB7);

//PIXEL FORMAT
ILI9341_Write_Command(0x3A);
ILI9341_Write_Data(0x55);

//MEMORY ACCESS CONTROL
ILI9341_Write_Command(0x36);
ILI9341_Write_Data(0xA8);

//FRAME RATIO CONTROL, STANDARD RGB COLOR
ILI9341_Write_Command(0xB1);
ILI9341_Write_Data(0x00);
ILI9341_Write_Data(0x12);

//DISPLAY FUNCTION CONTROL
ILI9341_Write_Command(0xB6);
ILI9341_Write_Data(0x0A);
ILI9341_Write_Data(0xA2);

//3GAMMA FUNCTION DISABLE
ILI9341_Write_Command(0xF2);
ILI9341_Write_Data(0x00);

//GAMMA CURVE SELECTED
ILI9341_Write_Command(0x26);
ILI9341_Write_Data(0x01);

//GAMMA CURVE SELECTED
ILI9341_Write_Command(0x36);
ILI9341_Write_Data(0xA8);

//GAMMA CURVE SELECTED
ILI9341_Write_Command(0x44);
ILI9341_Write_Data(0x02);

//POSITIVE GAMMA CORRECTION
ILI9341_Write_Command(0xE0);
ILI9341_Write_Data(15);
ILI9341_Write_Data(42);
ILI9341_Write_Data(40);
ILI9341_Write_Data(8);
ILI9341_Write_Data(14);
ILI9341_Write_Data(8);
ILI9341_Write_Data(84);
ILI9341_Write_Data(169);
ILI9341_Write_Data(67);
ILI9341_Write_Data(10);
ILI9341_Write_Data(15);
ILI9341_Write_Data(0);
ILI9341_Write_Data(0);
ILI9341_Write_Data(0);
ILI9341_Write_Data(0);

//NEGATIVE GAMMA CORRECTION
ILI9341_Write_Command(0xE1);
ILI9341_Write_Data(0);
ILI9341_Write_Data(21);
ILI9341_Write_Data(23);
ILI9341_Write_Data(7);
ILI9341_Write_Data(17);
ILI9341_Write_Data(6);
ILI9341_Write_Data(43);
ILI9341_Write_Data(86);
ILI9341_Write_Data(60);
ILI9341_Write_Data(5);
ILI9341_Write_Data(16);
ILI9341_Write_Data(15);
ILI9341_Write_Data(63);
ILI9341_Write_Data(63);
ILI9341_Write_Data(15);



#endif
#ifdef SATURN

	ILI9341_Write_Command(0xe0);
	ILI9341_Write_Data(0x00);
	ILI9341_Write_Data(0x03);
	ILI9341_Write_Data(0x0c);
	ILI9341_Write_Data(0x09);
	ILI9341_Write_Data(0x17);
	ILI9341_Write_Data(0x09);
	ILI9341_Write_Data(0x3e);
	ILI9341_Write_Data(0x89);
	ILI9341_Write_Data(0x49);
	ILI9341_Write_Data(0x08);
	ILI9341_Write_Data(0x0d);
	ILI9341_Write_Data(0x0a);
	ILI9341_Write_Data(0x13);
	ILI9341_Write_Data(0x15);
	ILI9341_Write_Data(0x0f);

	ILI9341_Write_Command(0xe1);
	ILI9341_Write_Data(0x00);
	ILI9341_Write_Data(0x11);
	ILI9341_Write_Data(0x15);
	ILI9341_Write_Data(0x03);
	ILI9341_Write_Data(0x0f);
	ILI9341_Write_Data(0x05);
	ILI9341_Write_Data(0x2d);
	ILI9341_Write_Data(0x34);
	ILI9341_Write_Data(0x41);
	ILI9341_Write_Data(0x02);
	ILI9341_Write_Data(0x0b);
	ILI9341_Write_Data(0x0a);
	ILI9341_Write_Data(0x33);
	ILI9341_Write_Data(0x37);
	ILI9341_Write_Data(0x0f);

	ILI9341_Write_Command(0xc0);
	ILI9341_Write_Data(0x17);
	ILI9341_Write_Data(0x15);

	ILI9341_Write_Command(0xc1);
	ILI9341_Write_Data(0x41);

	ILI9341_Write_Command(0xc5);
	ILI9341_Write_Data(0x00);
	ILI9341_Write_Data(0x12);
	ILI9341_Write_Data(0x80);

	ILI9341_Write_Command(0x3a);
	ILI9341_Write_Data(0x55);

	ILI9341_Write_Command(0xb0);
	ILI9341_Write_Data(0x00);

	ILI9341_Write_Command(0xb1);
	ILI9341_Write_Data(0xa0);

	ILI9341_Write_Command(0xb4);
	ILI9341_Write_Data(0x02);

	ILI9341_Write_Command(0xe9);
	ILI9341_Write_Data(0x00);

	ILI9341_Write_Command(0xf7);
	ILI9341_Write_Data(0xa9);
	ILI9341_Write_Data(0x51);
	ILI9341_Write_Data(0x2c);
	ILI9341_Write_Data(0x82);

	ILI9341_Write_Command(0xb6);
	ILI9341_Write_Data(0x02);
	ILI9341_Write_Data(0x02);

	ILI9341_Write_Command(0x36);
	ILI9341_Write_Data(0xe8);

#endif

//EXIT SLEEP
ILI9341_Write_Command(0x11);
lcdDelay(120);

//TURN ON DISPLAY
ILI9341_Write_Command(0x29);

//STARTING ROTATION
ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
ILI9341_Fill_Screen(BLUE);

LV_tft_init();
//ILI9341_Set_Address(0, 0, 240, 320);
}

//INTERNAL FUNCTION OF LIBRARY, USAGE NOT RECOMENDED, USE Draw_Pixel INSTEAD
/*Sends single pixel Color information to LCD*/
void ILI9341_Draw_Color(uint16_t Color)
{

ILI9341_Write_Data(Color);

}

//INTERNAL FUNCTION OF LIBRARY
/*Sends block Color information to LCD*/
void ILI9341_Draw_Color_Burst(uint16_t Color, uint32_t Size)
{

HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_SET);
  for(uint32_t i = 0; i < Size /*(x2-x1+1)*(y2-y1+1)*/; i++)
  {
      LCD->LCD_RAM = Color;
  }
}

//FILL THE ENTIRE SCREEN WITH SELECTED Color (either #define-d ones or custom 16bit)
/*Sets address (entire screen) and Sends Height*Width ammount of Color information to LCD*/
void ILI9341_Fill_Screen(uint16_t Color) //void LCD_Fill(lv_color_t * page_buff)
{	
//HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_RESET);
ILI9341_Set_Address(0,0, LCD_WIDTH - 1, LCD_HEIGHT - 1 );
HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_SET);
  unsigned int i; 
	uint32_t total_point=LCD_HEIGHT*LCD_WIDTH;
	for(i=0;i<total_point;i++)
	{ 
		LCD->LCD_RAM = Color;
	}

}


/*****************************************************************************
 * @name       :void LCD_DrawPoint(uint16_t x,uint16_t y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void ILI9341_Draw_Pixel(uint16_t x,uint16_t y, uint16_t color)
{
	ILI9341_SetCursor(x,y); 
	ILI9341_WriteData_16Bit(color); 
}

/*****************************************************************************
 * @name       :void ILI9341_SetCursor(uint16_t Xpos, uint16_t Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void ILI9341_SetCursor(uint16_t Xpos, uint16_t Ypos)
{	
	ILI9341_Set_Address(Xpos, Ypos, Xpos, Ypos);	
} 

/*****************************************************************************
 * @name       :void ILI9341_WriteData_16Bit(uint16_t Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void ILI9341_WriteData_16Bit(uint16_t Data)
{
	HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_SET);
	 LCD->LCD_RAM = Data;
}

uint16_t Color_To_565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

//DRAW RECTANGLE OF SET SIZE AND HEIGTH AT X and Y POSITION WITH CUSTOM Color
//
//Rectangle is hollow. X and Y positions mark the upper left corner of rectangle
//As with all other draw calls x0 and y0 locations dependant on screen orientation
//

void ILI9341_Draw_Rectangle(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint16_t Color)
{
	
if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;
if((X+Width-1)>=LCD_WIDTH)
	{
		Width=LCD_WIDTH-X;
	}
if((Y+Height-1)>=LCD_HEIGHT)
	{
		Height=LCD_HEIGHT-Y;
	}  

ILI9341_Set_Address(X, Y, X+Width-1, Y+Height-1);
ILI9341_Draw_Color_Burst(Color, Height*Width);
}

//DRAW LINE FROM X,Y LOCATION to X+Width,Y LOCATION
void ILI9341_Draw_Horizontal_Line(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Color)
{
if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;
if((X+Width-1)>=LCD_WIDTH)
	{
		Width=LCD_WIDTH-X;
	}
ILI9341_Set_Address(X, Y, X+Width-1, Y);
ILI9341_Draw_Color_Burst(Color, Width);
}

//DRAW LINE FROM X,Y LOCATION to X,Y+Height LOCATION
void ILI9341_Draw_Vertical_Line(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Color)
{
if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;
if((Y+Height-1)>=LCD_HEIGHT)
	{
		Height=LCD_HEIGHT-Y;
	}
ILI9341_Set_Address(X, Y, X, Y+Height-1);
ILI9341_Draw_Color_Burst(Color, Height);
}

uint16_t ILI9341_Read_Data(void)
{
	volatile uint16_t data;  //·АЦ№±»УЕ»Ї
	HAL_GPIO_WritePin(FMC_A1_REAL_GPIO_Port, FMC_A1_REAL_Pin, GPIO_PIN_SET);
	data=LCD->LCD_RAM;
	return data;
}

/*****************************************************************************
 * @name       :uint16_t Lcd_ReadData_16Bit(void)
 * @date       :2018-11-13 
 * @function   :Read an 16-bit value from the LCD screen
 * @parameters :None
 * @retvalue   :read value
******************************************************************************/	
uint16_t Lcd_ReadData_16Bit(void)
{
	uint16_t r,g,b;
	//dummy data
	r = ILI9341_Read_Data();
	lcdDelay(1);//СУК±1us
	//8bit:red data	
	//16bit:red and green data
	r = ILI9341_Read_Data();
	lcdDelay(1);//СУК±1us
	//8bit:green data
    //16bit:blue data
	g = ILI9341_Read_Data();
	
    b = g>>8;
	g = r&0xFF; 
	r = r>>8;

	return Color_To_565(r, g, b);
}

/*****************************************************************************
 * @name       :uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
 * @date       :2018-11-13 
 * @function   :Read a pixel color value at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :the read color value
******************************************************************************/	
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
	uint16_t color;
	if(x>=LCD_WIDTH||y>=LCD_HEIGHT)
	{
		return 0;		
	}
	ILI9341_SetCursor(x,y); 
	LCD_ReadRAM_Prepare();
	color = Lcd_ReadData_16Bit(); 
	return color;
}

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	ILI9341_Write_Command(ILI9341_CMD_MEMORY_WRITE);
}	 

/*****************************************************************************
 * @name       :void LCD_ReadRAM_Prepare(void)
 * @date       :2018-11-13 
 * @function   :Read GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_ReadRAM_Prepare(void)
{
	ILI9341_Write_Command(ILI9341_CMD_MEMORY_READ);
}

