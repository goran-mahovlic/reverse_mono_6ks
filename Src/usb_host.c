/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
//Include the fatfs library here to be inside user code blocks
#include "fatfs.h"
FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
USBH_HandleTypeDef hUSB_Host; /* USB Host handle */
/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
void USB_Error_Handler(void)
{
  /* USER CODE BEGIN USB_Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
 HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,GPIO_PIN_SET);
  while(1)
  {
  }
  /* USER CODE END USB_Error_Handler */
}

static void MSC_Application(void)
{
  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
  uint8_t rtext[100];                                   /* File read buffer */

  /* Register the file system object to the FatFs module */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    USB_Error_Handler();
  }
  else
  {
      /* Create and Open a new text file object with write access */
      if(f_open(&MyFile, "Even.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
      {
        /* 'STM32.TXT' file Open for write Error */
        USB_Error_Handler();
      }
      else
      {
        /* Write data to the text file */
        res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);

        if((byteswritten == 0) || (res != FR_OK))
        {
          /* 'STM32.TXT' file Write or EOF Error */
          USB_Error_Handler();
        }
        else
        {
          /* Close the open text file */
          f_close(&MyFile);

        /* Open the text file object with read access */
        if(f_open(&MyFile, "Even.TXT", FA_READ) != FR_OK)
        {
          /* 'STM32.TXT' file Open for read Error */
          USB_Error_Handler();
        }
        else
        {
          /* Read data from the text file */
          res = f_read(&MyFile, rtext, sizeof(rtext), (void *)&bytesread);

          if((bytesread == 0) || (res != FR_OK))
          {
            /* 'STM32.TXT' file Read or EOF Error */
            USB_Error_Handler();
          }
          else
          {
            /* Close the open text file */
            f_close(&MyFile);

            /* Compare read data with the expected data */
            if((bytesread != byteswritten))
            {
              /* Read data is different from the expected data */
              USB_Error_Handler();
            }
            else
            {
          /* Success of the demo: no error occurrence */
              //BSP_LED_On(LED4);
              HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,GPIO_PIN_SET);
            }
          }
        }
      }
    }
  }

  /* Unlink the USB disk I/O driver */
  FATFS_UnLinkDriver(USBDISKPath);
}


#define SECTOR_SIZE 512  // Standard sector size
#define NUM_SECTORS_TO_CLEAR 16  // Number of sectors to clear

void USB_Error_Handler(void);

void Clear_Sectors(void) {
    uint8_t buffer[SECTOR_SIZE];
    memset(buffer, 0x00, SECTOR_SIZE);  // Fill buffer with zeros

    for (int i = 0; i < NUM_SECTORS_TO_CLEAR; i++) {
        if (USBH_MSC_Write(&hUsbHostFS, 0, i, buffer, 1) != USBH_OK) {
            USB_Error_Handler();  // Handle errors, e.g., disconnect
            return;
        }
    }
}

void Format_USB_Drive(void) {
    // Unmount the current file system
    f_mount(NULL, "", 1);
    uint8_t buffer[SECTOR_SIZE];
    // Clear first few sectors to reset the MBR
    Clear_Sectors();

    // Attempt to mount and format the drive
    if (f_mount(&USBDISKFatFs, "", 0) == FR_NO_FILESYSTEM) {
        // Create a new FAT32 partition
        if (f_mkfs("", FM_FAT32, 0, buffer, sizeof(buffer)) != FR_OK) {
            USB_Error_Handler();  // Handle errors
        } else {
            // Successfully formatted
            // Mount the newly created file system
            if (f_mount(&USBDISKFatFs, "", 0) != FR_OK) {
                USB_Error_Handler();  // Handle errors
            }
        }
    } else {
        USB_Error_Handler();  // Handle errors if the mount fails
    }
}

//void USB_Error_Handler(void);

void Send_SCSI_Command(uint8_t *cmd, uint8_t cmd_len) {
    MSC_HandleTypeDef *MSC_Handle = (MSC_HandleTypeDef*) hUsbHostFS.pActiveClass->pData;

    USBH_StatusTypeDef status;
    status = USBH_BulkSendData(&hUsbHostFS, cmd, cmd_len, MSC_Handle->OutPipe, 100);
    if (status != USBH_OK) {
        USB_Error_Handler();
    }
}

void Reset_Flash_Controller(void) {
    uint8_t cmd[6];

    // SCSI Test Unit Ready command
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = 0x00;  // TEST UNIT READY

    Send_SCSI_Command(cmd, sizeof(cmd));

    // SCSI Start/Stop Unit command
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = 0x1B;  // START STOP UNIT
    cmd[4] = 0x01;  // Start the unit

    Send_SCSI_Command(cmd, sizeof(cmd));
}

void Main_USB_Repair(void) {
    // Initialize USB Host and wait for USB drive to be ready
    USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS);
    USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS);
    USBH_Start(&hUsbHostFS);

    while (Appli_state != APPLICATION_READY) {
        USBH_Process(&hUsbHostFS);
    }

    Reset_Flash_Controller();
    // Once USB drive is ready, attempt to repair
    Format_USB_Drive();

    // Clean up and stop USB host
    USBH_Stop(&hUsbHostFS);
}
/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */

  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */

  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostFS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
    //Main_USB_Repair();
    //HAL_Delay(1000);
    MSC_Application();
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;
  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

