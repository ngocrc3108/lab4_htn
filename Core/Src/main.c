/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_PERIOD_MS 30 // ms
#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define TRIANGLE_LENGTH 20
#define TRIANGLE_HEIGHT 16 // ~~ 20sin(60)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  BSP_GYRO_Init(); // L3GD20_FULLSCALE_500
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
  BSP_LCD_SelectLayer(1);//select on which layer we write
  BSP_LCD_DisplayOn();//turn on LCD
  BSP_LCD_Clear(LCD_COLOR_WHITE);//clear the LCD on white color
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);//set text background color
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  float xAngle = 0, yAngle = 0, zAngle = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    float rawData[3];
    BSP_GYRO_GetXYZ(rawData);
    for(uint8_t i = 0; i < 3; i++) {
        // Convert Gyro raw to degrees per second, L3GD20_FULLSCALE_500
    	rawData[i] = rawData[i]*0.01750;
    }
    xAngle = rawData[0] * SAMPLE_PERIOD_MS / 1000;
    yAngle = rawData[1] * SAMPLE_PERIOD_MS / 1000;
    zAngle = rawData[2] * SAMPLE_PERIOD_MS / 1000;

    char stringBuf[100];

    sprintf(stringBuf, "xAngle: %f\nyAngle: %f\nyAngle: %f", xAngle, yAngle, zAngle);

    //zAngle = rawData[2] * SAMPLE_PERIOD_MS / 1000;
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    if(xAngle >= 0) {
    	// draw top triangle
    	BSP_LCD_FillTriangle(LCD_WIDTH/2 - TRIANGLE_LENGTH/2, LCD_WIDTH/2, LCD_WIDTH/2  + TRIANGLE_LENGTH/2,
    			5 + TRIANGLE_HEIGHT,5 , 5 + TRIANGLE_HEIGHT);
    } else {
    	// draw bottom triangle
    	BSP_LCD_FillTriangle(LCD_WIDTH/2 - TRIANGLE_LENGTH/2, LCD_WIDTH/2, LCD_WIDTH/2  + TRIANGLE_LENGTH/2,
    		LCD_HEIGHT - 5 - TRIANGLE_HEIGHT, LCD_HEIGHT - 5, LCD_HEIGHT - 5 - TRIANGLE_HEIGHT);
    }
    if(yAngle >= 0) {
    	// draw right triangle
    	BSP_LCD_FillTriangle(LCD_WIDTH - 5 - TRIANGLE_HEIGHT, LCD_WIDTH - 5 - TRIANGLE_HEIGHT, LCD_WIDTH - 5,
    						LCD_HEIGHT/2 - TRIANGLE_LENGTH/2, LCD_HEIGHT/2 + TRIANGLE_LENGTH/2, LCD_HEIGHT/2);
    } else {
    	// draw left triangle
    	BSP_LCD_FillTriangle(5, TRIANGLE_HEIGHT + 5, TRIANGLE_HEIGHT + 5,
    			LCD_HEIGHT/2, LCD_HEIGHT/2 - TRIANGLE_LENGTH/2, LCD_HEIGHT/2 + TRIANGLE_LENGTH/2);
    }
    HAL_Delay(SAMPLE_PERIOD_MS);

    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
