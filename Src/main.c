/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>
#include "../lvgl/lvgl.h"
#include "../lvgl/demos/benchmark/lv_demo_benchmark.h"
#include "touchpad.h"
#include "XPT2046_lv.h"
#include "../src/ui/ui.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define if you want to run LVGL benchmark
#define USE_EEZ_PROJECT

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
/*
  extern int16_t last_x;
  extern int16_t last_y;
  extern uint8_t last_state;
  */
 //volatile uint8_t touchPressed = false;
volatile uint32_t motorSpeed = 40000;
volatile uint32_t ZPosition = 10000;
volatile uint32_t maxPosition = 20000;
bool initialHomeZ = false;
Stepper_t stepperZ;
int32_t get_var_motor_speed();
void set_var_motor_speed(int32_t value);
void action_move_down(lv_event_t * e);
void action_move_up(lv_event_t * e);
void action_home_z(lv_event_t * e);
void action_middle_z(lv_event_t * e);
void action_lcd(lv_event_t * e);
bool get_var_intial_home_z();
void set_var_intial_home_z(bool value);
int32_t get_var_current_position();
void set_var_current_position(int32_t value);
const char *get_var_current_operation();
void set_var_current_operation(const char *value);
char currentOP[200] = "Idle";
uint8_t lcdStartDrawing[1] = { 0xFB };
uint8_t lcd_black[2] = { 0x00, 0xFF };
uint8_t lcd_white[2] = { 0x00, 0x00 };
uint8_t readID[1] = { 0xF0 };
uint8_t spiTX[2] = { 0x00 };
uint8_t spiRX[2] = { 0x00 };
void readInput();
void LCD_readID();
void FANoff();
void LCD_reset();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RNG_Init(void);
static void MX_FMC_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
int _gettimeofday( struct timeval *tv, void *tzvp )
{
    // you can add code here there many example in google search.
    return 0;  // return non-zero for error
} // end _gettimeofday()

/* Just some function that I randomly found - It is mentioning Exposuring so maybe that could be some pointer

void FUN_000194b0(int param_1)

{
  int iVar1;
  
  FUN_00018eb0(0xd,0,0xfe,0x100,0x18);
  if (param_1 == 1) {
    iVar1 = DAT_00019520;
    if (*(char *)(iVar1 + 0x14) == '\0') {
      FUN_0000b834(0x38,0x101,&DAT_00019524,0xffe0);
    }
    else {
      FUN_0000b734(0x24,0xfe,s_Exposuring_00019534,0xffe0);
    }
  }
  else {
    FUN_00018e80(0xd);
    iVar1 = DAT_00019520;
    if (*(char *)(iVar1 + 0x14) == '\0') {
      FUN_0000b834(0x51,0x101,&DAT_00019540,0xffe0);
    }
    else {
      FUN_0000b734(0x56,0xfe,&DAT_0001954c,0xffe0);
    }
  }
  return;
}


*/

void LCD_readID(){
 // for (uint8_t i=0;i<256;i++){
    HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port,SPI3_NSS_Pin,GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_SPI_Transmit(&hspi3, readID, 1, 50);
    HAL_Delay(60);
    HAL_SPI_Receive(&hspi3, spiRX, 2, 100);
    HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port,SPI3_NSS_Pin,GPIO_PIN_SET);
  //  spiTX[1]++;
    //HAL_Delay(100);
  //  if(spiRX[0]!=0x00){
  //    break;
  //  }
  //  if(spiRX[1]!=0x00){
  //    break;
  //  }
 // }
    HAL_Delay(500);
}

void readInput(){
  //F11, F12, F13, F14, F15

 // HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, !HAL_GPIO_ReadPin(PF11_GPIO_Port,PF11_Pin));
 // HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, !HAL_GPIO_ReadPin(PF12_GPIO_Port,PF12_Pin));
 // HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, !HAL_GPIO_ReadPin(PF13_GPIO_Port,PF13_Pin));
 // HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, !HAL_GPIO_ReadPin(PF14_GPIO_Port,PF14_Pin));
 // HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, !HAL_GPIO_ReadPin(PF15_GPIO_Port,PF15_Pin));

  //HAL_GPIO_WritePin(LCD_PB10_GPIO_Port,LCD_PB10_Pin,GPIO_PIN_SET);
  //HAL_GPIO_WritePin(LCD_PB10_GPIO_Port,LCD_PB10_Pin,GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(LCD_PB11_GPIO_Port,LCD_PB11_Pin,GPIO_PIN_SET);
  //HAL_GPIO_WritePin(LCD_PB11_GPIO_Port,LCD_PB11_Pin,GPIO_PIN_RESET);
}

void LCD_panel_half_black()
{
  HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port,SPI3_NSS_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_SPI_Transmit(&hspi3, lcdStartDrawing, 2, 1000);
  HAL_Delay(60);
  HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port,SPI3_NSS_Pin,GPIO_PIN_SET);
  HAL_Delay(6000);
  HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port,SPI3_NSS_Pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, lcdStartDrawing, 2, 100);
  int i=0;
  int j=0;
  for (i = 0; i < 5760; ++i){
    for (j = 0; j < 3600; ++j){
      if (j<1800){
        HAL_SPI_Transmit(&hspi3, lcd_white, 2, 100); // Pixel data
      }
      else{
        HAL_SPI_Transmit(&hspi3, lcd_black, 2, 100); // Pixel data     
        }
      }
  }
  HAL_Delay(60);
  HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port,SPI3_NSS_Pin,GPIO_PIN_SET);
  }

bool get_var_intial_home_z(){
  return initialHomeZ;
}
void set_var_intial_home_z(bool value){
  initialHomeZ = value;
}

const char *get_var_current_operation(){
return currentOP;
}
void set_var_current_operation(const char *value){
//*currentOP = &value;
}

int32_t get_var_current_position(){
  return ZPosition;
}
void set_var_current_position(int32_t value){
  ZPosition = value;
}

int32_t get_var_motor_speed(){
  return motorSpeed;
}

void FANoff(){
  HAL_GPIO_WritePin(FAN_GPIO_Port,FAN_Pin, GPIO_PIN_RESET);
}

void set_var_motor_speed(int32_t value){
  motorSpeed = value;
	setMaxSpeed(&stepperZ, motorSpeed);
	setSpeed(&stepperZ, motorSpeed);
	setAcceleration(&stepperZ, motorSpeed/5);  
}

void action_move_down(lv_event_t * e){
  strcpy(currentOP, "Move DOWN");
  ZPosition = ZPosition - 1000;
  if (ZPosition >= 200){
    runToNewPosition(&stepperZ,ZPosition);
  }
}
void action_move_up(lv_event_t * e){
  strcpy(currentOP, "Move UP");
  if (ZPosition < 20000){
    ZPosition = ZPosition + 1000;
    runToNewPosition(&stepperZ,ZPosition);
  }
}
void action_home_z(lv_event_t * e){
  strcpy(currentOP, "HOME Z");
	moveTo(&stepperZ, -20000);  
  while (HAL_GPIO_ReadPin(HOME_SW_GPIO_Port,HOME_SW_Pin)){
    //ZPosition = -200;
    run(&stepperZ);
  }
  ZPosition = 0;
  setCurrentPosition(&stepperZ,0);
  initialHomeZ = true;
}
void action_middle_z(lv_event_t * e){
  strcpy(currentOP, "MIDDLE Z");
  ZPosition = 10000;
  runToNewPosition(&stepperZ,ZPosition);
}

void LCD_reset(){
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin, GPIO_PIN_SET);
  // FPGA BOOT TIME?
  HAL_Delay(3000);
}

void action_lcd(lv_event_t * e){
  LCD_reset();
  LCD_readID();
  LCD_panel_half_black();
}

void InitFullStep(void){
HAL_GPIO_WritePin(MOTOR_M0_GPIO_Port,MOTOR_M0_Pin,GPIO_PIN_RESET);
HAL_GPIO_WritePin(MOTOR_M1_GPIO_Port,MOTOR_M1_Pin,GPIO_PIN_RESET);
}

void InitSmartTuneDynamicDecay(){
HAL_GPIO_WritePin(MOTOR_DEC0_GPIO_Port,MOTOR_DEC0_Pin,GPIO_PIN_RESET);
HAL_GPIO_WritePin(MOTOR_DEC1_GPIO_Port,MOTOR_DEC1_Pin,GPIO_PIN_RESET);
}

void InitSlowDecay(){
HAL_GPIO_WritePin(MOTOR_DEC0_GPIO_Port,MOTOR_DEC0_Pin,GPIO_PIN_SET);
HAL_GPIO_WritePin(MOTOR_DEC1_GPIO_Port,MOTOR_DEC1_Pin,GPIO_PIN_SET);
}

void motor_init(){
  InitFullStep();
  InitSmartTuneDynamicDecay();
	/*##-3- Initialize X axis stepper. ###*/
  InitStepper(&stepperZ, DRIVER, MOTOR_STEP_Pin, MOTOR_STEP_GPIO_Port, MOTOR_DIR_Pin, MOTOR_DIR_GPIO_Port,1);
	setMaxSpeed(&stepperZ, motorSpeed);
	setSpeed(&stepperZ, motorSpeed);
	setAcceleration(&stepperZ, motorSpeed);
  setEnablePin(&stepperZ, MOTOR_ENABLE_Pin, MOTOR_ENABLE_GPIO_Port);
	enableOutputs(&stepperZ);
  setCurrentPosition(&stepperZ,ZPosition);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t color = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RNG_Init();
  MX_FMC_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  //motor_init();
  FANoff();
  readInput();
  //LCD_reset();
  //LCD_readID();
  lv_init();
  ILI9341_Init();
  lv_touchpad_init();
  HAL_Delay(100);
  #ifdef USE_EEZ_PROJECT
  //lv_demo_benchmark();
  ui_init();
  //lv_demo_widgets();
  //lv_demo_stress();
  #else
    for (int i= 0; i<10;i++){
      lcd_fill_rand_colors();
      HAL_Delay(300);
    }
  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  #ifdef USE_EEZ_PROJECT

  lv_task_handler();
  ui_tick();
  HAL_Delay(10);
  #else
    ILI9341_Draw_Text ("MONO X6",10,60,RED, 6, BLACK);
    ILI9341_Draw_Text ("RESIN",10,130,GREEN, 6,BLACK);
    ILI9341_Draw_Text ("HACKED",10,200,BLUE, 6,BLACK);
    HAL_Delay(2000);
    lcd_random_points();
    ILI9341_Draw_Text ("MONO X6",10,60,RED, 6, BLACK);
    ILI9341_Draw_Text ("RESIN",10,130,GREEN, 6,BLACK);
    ILI9341_Draw_Text ("HACKED",10,200,BLUE, 6,BLACK);
    lcd_random_points();
    lcd_random_circles();
    lcd_random_points();
    lcd_random_rectangles();
    lcd_random_points();
    lcd_random_lines();
  #endif
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 6;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 6;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MOTOR_ENABLE_Pin|MOTOR_DIR_Pin|MOTOR_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, UV_LED_Pin|LCD_RSTG2_Pin|TS_DIN_Pin|MOTOR_M0_Pin
                          |MOTOR_DEC1_Pin|MOTOR_DEC0_Pin|MOTOR_M1_Pin|MOTOR_nSLEEP_Pin
                          |SPI3_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LCD_RST_Pin|FMC_A1_REAL_Pin|LCD_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_PB10_GPIO_Port, LCD_PB10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_PB11_GPIO_Port, LCD_PB11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TS_CS_Pin|TS_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_ENABLE_Pin MOTOR_DIR_Pin MOTOR_STEP_Pin */
  GPIO_InitStruct.Pin = MOTOR_ENABLE_Pin|MOTOR_DIR_Pin|MOTOR_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : D1_Pin */
  GPIO_InitStruct.Pin = D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HOME_SW_Pin */
  GPIO_InitStruct.Pin = HOME_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HOME_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_Pin */
  GPIO_InitStruct.Pin = FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UV_LED_Pin LCD_RST_Pin LCD_RSTG2_Pin LCD_BL_Pin
                           MOTOR_M0_Pin MOTOR_DEC1_Pin MOTOR_DEC0_Pin MOTOR_M1_Pin
                           MOTOR_nSLEEP_Pin */
  GPIO_InitStruct.Pin = UV_LED_Pin|LCD_RST_Pin|LCD_RSTG2_Pin|LCD_BL_Pin
                          |MOTOR_M0_Pin|MOTOR_DEC1_Pin|MOTOR_DEC0_Pin|MOTOR_M1_Pin
                          |MOTOR_nSLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_PB10_Pin LCD_PB11_Pin */
  GPIO_InitStruct.Pin = LCD_PB10_Pin|LCD_PB11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TS_CS_Pin TS_CLK_Pin */
  GPIO_InitStruct.Pin = TS_CS_Pin|TS_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_DOUT_Pin */
  GPIO_InitStruct.Pin = TS_DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TS_DOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_DIN_Pin */
  GPIO_InitStruct.Pin = TS_DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TS_DIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_IRQ_Pin */
  GPIO_InitStruct.Pin = TS_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TS_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_A1_REAL_Pin SPI3_NSS_Pin */
  GPIO_InitStruct.Pin = FMC_A1_REAL_Pin|SPI3_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
