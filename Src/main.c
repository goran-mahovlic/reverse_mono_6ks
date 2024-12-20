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
#include "INA219.h"
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
I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
SRAM_HandleTypeDef hsram1;

INA219_t ina219;
Stepper_t stepperZ;

bool running = false;
bool calibrated = false;
bool direction = false; // down - used onlly under start
bool initialHomeZ = false;
bool start = false;
bool positive_diff = true;

double sensor = 0.01;
double max_diff = 0.01;
double sensor_min = 0.01;
double sensor_max = 5.00;
double current_diff = 0.00;

int32_t loops = 0;
int32_t motorSpeed = 10000;
int32_t ZPosition = 1000;
int32_t maxPosition = 2000;

char currentOP[200] = "Idle";

void set_var_positive_diff(bool value);
void set_var_loops(int32_t value);
void set_var_current_diff(double value);
void set_var_sensor_min(double value);
void set_var_sensor_max(double value);
void set_var_sensor(double value);
void set_var_max_diff(double value);
void set_var_calibrated(bool value);
void set_var_intial_home_z(bool value);
void set_var_motor_speed(int32_t value);
void set_var_current_operation(const char *value);
void set_var_current_position(int32_t value);

bool get_var_positive_diff();
int32_t get_var_loops();
double get_var_current_diff();
double get_var_sensor_min();
double get_var_sensor_max();
double get_var_sensor();
double get_var_max_diff();
bool get_var_calibrated();
bool get_var_intial_home_z();
int32_t get_var_motor_speed();
const char *get_var_current_operation();
int32_t get_var_current_position();

void getSensorValue(uint8_t sample_number);

void action_move_down(lv_event_t * e);
void action_move_up(lv_event_t * e);
void action_home_z(lv_event_t * e);
void action_middle_z(lv_event_t * e);
void action_lcd(lv_event_t * e);

void FANon();
void FANoff();
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
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
int _gettimeofday( struct timeval *tv, void *tzvp )
{
    // you can add code here there many example in google search.
    return 0;  // return non-zero for error
} // end _gettimeofday()

void getSensorValue(uint8_t sample_number){
  uint8_t sampling_counter = 0;
  double sensor_sampling = 0;

  while(sampling_counter<sample_number){
    sampling_counter++;
    sensor_sampling += INA219_ReadBusVoltage(&ina219);  
  }
  sensor = (sensor_sampling/sample_number)/1000 - sensor_min;
}

void find_max(){
  sensor_max = sensor;
}

void start_action(){
  if(start){
    if (running == true){
      if (currentPosition(&stepperZ)==1000){
        direction=true;
        loops++;
      }
      else if (currentPosition(&stepperZ)<=0){
        direction=false;
        loops++;
      }


      if (direction){
        ZPosition--;
        moveTo(&stepperZ, currentPosition(&stepperZ)-1);
      }
      else{
        ZPosition++;
        moveTo(&stepperZ, currentPosition(&stepperZ)+1);
      }

      run(&stepperZ);
      HAL_Delay(10); // waiting for Voltage stabilisation
      getSensorValue(200);
      double mm_position = ZPosition;
      double mm_sensor =  sensor*2.195*100;
      double tmp_diff = mm_sensor - mm_position;
      // Calculate the absolute difference
      if(tmp_diff>0){
        positive_diff = true;
      }
      else{
        positive_diff = false;
      }
      double abs_diff = fabs(tmp_diff)/100;
      current_diff = abs_diff;
        // Update max_diff if the absolute difference is greater
        if(ZPosition<980){
        if (abs_diff > max_diff) {
            max_diff = abs_diff;
        } 
    }
    }
  }
  else{
    getSensorValue(20);
  }
}

void action_lcd(lv_event_t * e){
}

void action_start(lv_event_t *e) {
  start=true;
  running = true;
}
bool get_var_running() {
    return running;
}

void set_var_running(bool value) {
    running = value;
}

void action_stop(lv_event_t *e) {
    // TODO: Implement action stop here
    // Home again and find starting position
    running = false;
    start = false;
    loops = 0;
    //calibrate();
    //sensor = 0.01;
    //max_diff = 0.01;
    stop(&stepperZ);
    //running = false;
}

void action_clear(lv_event_t *e) {
    // TODO: Implement action clear here
  initialHomeZ = false;
  ZPosition = 10000;
  calibrated=false;
}

void calibrate(){
  moveTo(&stepperZ, -20000);
  while (HAL_GPIO_ReadPin(HOME_SW_GPIO_Port,HOME_SW_Pin)){
    //getSensorValue(10);
    run(&stepperZ); 
  }
  // Now we are home, we now need to move up until sensor value changes
  getSensorValue(10);  
  double tmp_sensor = sensor;

  while (tmp_sensor == sensor){
    getSensorValue(200);
    moveTo(&stepperZ, currentPosition(&stepperZ)+1);
    run(&stepperZ); 
  }
  sensor_min = sensor;
  // Now move one step back
  moveTo(&stepperZ, currentPosition(&stepperZ)-1);
  run(&stepperZ);

  // This should be sensor starting point - set Home and Calibrated
  setCurrentPosition(&stepperZ,0);
  initialHomeZ = true;
  ZPosition = 0;
  calibrated=true;
  max_diff = 0.01;
  loops = 0;
  getSensorValue(200);
    // TODO: Implement action start here
  //Step by step go to 5V
  
  // Move sensor to 10000 mm (MAX sensor) or 5V
  ZPosition = currentPosition(&stepperZ);
  moveTo(&stepperZ, 1000);
  //runToNewPosition(&stepperZ,ZPosition);
  while (currentPosition(&stepperZ)<1000){
    run(&stepperZ);
    //getSensorValue(10);
  }
  run(&stepperZ);
  getSensorValue(100);
  // Find maximum
  find_max();
}

void action_calibrate(lv_event_t *e) {
    // TODO: Implement action calibrate here
  calibrate();
}

double get_var_max_diff() {
    return max_diff;
}

void set_var_max_diff(double value) {
    max_diff = value;
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
  ZPosition = currentPosition(&stepperZ);
  return ZPosition;
}
void set_var_current_position(int32_t value){
  ZPosition = value;
}

int32_t get_var_motor_speed(){
  return motorSpeed;
}

double get_var_sensor() {
    return sensor;
}

void set_var_sensor(double value) {
    sensor = value;
}

void FANoff(){
  HAL_GPIO_WritePin(FAN_GPIO_Port,FAN_Pin, GPIO_PIN_RESET);
}
void FANon(){
  HAL_GPIO_WritePin(FAN_GPIO_Port,FAN_Pin, GPIO_PIN_SET);
}

bool get_var_calibrated() {
    return calibrated;
}

void set_var_calibrated(bool value) {
    calibrated = value;
}

void set_var_motor_speed(int32_t value){
  motorSpeed = value;
	setMaxSpeed(&stepperZ, motorSpeed);
	setSpeed(&stepperZ, motorSpeed);
	setAcceleration(&stepperZ, motorSpeed/5);  
}

void action_move_down(lv_event_t * e){
  strcpy(currentOP, "Move DOWN");
  ZPosition = ZPosition - 1;
  if (ZPosition >= 1){
    runToNewPosition(&stepperZ,ZPosition);
  }
}
void action_move_up(lv_event_t * e){
  strcpy(currentOP, "Move UP");
  if (ZPosition < 20000){
    ZPosition = ZPosition + 1;
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

bool get_var_positive_diff() {
    return positive_diff;
}

void set_var_positive_diff(bool value) {
    positive_diff = value;
}


int32_t get_var_loops() {
    return loops;
}

void set_var_loops(int32_t value) {
    loops = value;
}

double get_var_current_diff() {
    return current_diff;
}

void set_var_current_diff(double value) {
    current_diff = value;
}


double get_var_sensor_min() {
    return sensor_min;
}

void set_var_sensor_min(double value) {
    sensor_min = value;
}

double get_var_sensor_max() {
    return sensor_max;
}

void set_var_sensor_max(double value) {
    sensor_max = value;
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
// uint16_t color = 0;
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
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  motor_init();
  FANoff();
  FANon();
  lv_init();
  ILI9341_Init();
  lv_touchpad_init();

  ui_init();

  while(!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS))
  {

  }
  // HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    lv_task_handler();
    ui_tick();
    HAL_Delay(10);
    start_action();

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 18;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
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
  HAL_GPIO_WritePin(LED_D1_GPIO_Port, LED_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, UV_LED_Pin|TS_DIN_Pin|MOTOR_M0_Pin|MOTOR_DEC1_Pin
                          |MOTOR_DEC0_Pin|MOTOR_M1_Pin|MOTOR_nSLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LCD_RST_Pin|LCD_RST_old_Pin|FMC_A1_REAL_Pin|LCD_BL_Pin
                          |SPI3_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_PB10_GPIO_Port, LCD_PB10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_PB11_GPIO_Port, LCD_PB11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TS_CS_Pin|TS_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_ENABLE_Pin MOTOR_DIR_Pin MOTOR_STEP_Pin */
  GPIO_InitStruct.Pin = MOTOR_ENABLE_Pin|MOTOR_DIR_Pin|MOTOR_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_D1_Pin */
  GPIO_InitStruct.Pin = LED_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED_D1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HOME_SW_Pin D6_Pin D7_Pin D0_Pin
                           D1_Pin D2_Pin D3_Pin CLK_Pin */
  GPIO_InitStruct.Pin = HOME_SW_Pin|D6_Pin|D7_Pin|D0_Pin
                          |D1_Pin|D2_Pin|D3_Pin|CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_Pin CS0_Pin */
  GPIO_InitStruct.Pin = FAN_Pin|CS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IO2_Pin C2_Pin C1_Pin I00_Pin
                           R_C_Pin */
  GPIO_InitStruct.Pin = IO2_Pin|C2_Pin|C1_Pin|I00_Pin
                          |R_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : UV_LED_Pin LCD_RST_Pin LCD_BL_Pin MOTOR_M0_Pin
                           MOTOR_DEC1_Pin MOTOR_DEC0_Pin MOTOR_M1_Pin MOTOR_nSLEEP_Pin */
  GPIO_InitStruct.Pin = UV_LED_Pin|LCD_RST_Pin|LCD_BL_Pin|MOTOR_M0_Pin
                          |MOTOR_DEC1_Pin|MOTOR_DEC0_Pin|MOTOR_M1_Pin|MOTOR_nSLEEP_Pin;
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

  /*Configure GPIO pin : LCD_RST_old_Pin */
  GPIO_InitStruct.Pin = LCD_RST_old_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_old_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : CMD_Pin */
  GPIO_InitStruct.Pin = CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CMD_GPIO_Port, &GPIO_InitStruct);

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
