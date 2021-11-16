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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "bmp280.h"
#include "ccs811.h"
#include "lc709203f.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

volatile uint32_t uptime_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vprint(const char *fmt, va_list argp) {
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) {
        HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 0xffffff);
    }
}

void cprintf(const char *fmt, ...) {
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

void vATaskLED( void *pvParameters )
{
    for(;;)
    {
    	HAL_Delay(2000);
    	HAL_GPIO_TogglePin(GPIOB, STATUS_LED_Pin);
    }
}

void vASensorReadings( void *pvParameters )
{
	/*bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_1;
	bmp280.i2c = &hi2c1;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
	  cprintf("BMP280 Initialization Failed \r\n");
	}

	bool bme280p = bmp280.id == BME280_CHIP_ID;
	cprintf("BOSH:%s \r\n", bme280p ? "BME280" : "BMP280");

	ccs811.addr = CCS811_I2C_ADDR_0;
	ccs811.i2c = &hi2c1;

	while (!CCS811_init(&ccs811)) {
	   cprintf("CCS811 Initialization Failed\r\n");
	}

    for(;;)
    {
    	while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
    		cprintf("Temperature/Pressure Reading Failed\r\n");
    		HAL_Delay(2000);
    	}
    	HAL_Delay(100);

    	cprintf("Pressure: %.2f Pa \r\n", pressure);
    	cprintf("Humidity: %.2f % \r\n", humidity);
    	cprintf("Temperature: %.2f C \r\n", temperature);
    	HAL_Delay(100);

    	CCS811_Measure_CO2_TVOC(&ccs811, &co2, &tvoc);
    	cprintf("Co2 %d\r\n", co2);
    	cprintf("TVoC %d\r\n", tvoc);

    	HAL_Delay(5000);
    }*/
}

/** Initialize the millisecond timer. */
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t i = 0, ret;
  HAL_Delay(100);

  cprintf("\n\r=====");
  for(i = 1; i < 128; i++) {
      ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 5);
      if(ret == HAL_OK) {
    	  cprintf("\n\r0x%X", i);
      }
  }
  cprintf("\n\r=====\n\r");

  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //xTaskCreate(vATaskLED, "LED", configMINIMAL_STACK_SIZE, NULL, 4, NULL);

  //xTaskCreate(vASensorReadings, "SENSORS", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

                                                     BMP280_HandleTypedef bmp280;
                                                     CCS811_HandleTypedef ccs811;

                                                     	 LC709203_HandleTypedef lc709203;
                                                     	lc709203.addr = LC709203F_I2CADDR_DEFAULT;
                                                     	lc709203.power = LC709203F_POWER_OPERATE;
                                                     	lc709203.adjust= LC709203F_APA_500MAH;
                                                     	lc709203.tempmode = LC709203F_TEMPERATURE_THERMISTOR;
                                                     	lc709203.arsoc = 600;
                                                     	lc709203.i2c = &hi2c1;

                                                     	lc709203.version = 0;

                                                     	if (LC709203_Init(&lc709203) != LC709203F_STATUS_OK) {

                                                     	}

                                                     	if (LC709203_Read_IC_Version(&lc709203) != LC709203F_STATUS_OK) {

                                                     	}
                                                     	cprintf("LC709203_VIC %d\n\r", lc709203.version);

                                                     	HAL_Delay(2000);

                                                     	float temperature, humidity, pressure;
                                                     	uint32_t co2, tvoc;

                                                     	BMP280_Init_Default_Params(&bmp280.params);
                                                     	bmp280.addr = BMP280_I2C_ADDRESS_1;
                                                     	bmp280.i2c = &hi2c1;

                                                     	while (!BMP280_Init(&bmp280, &bmp280.params)) {
                                                     		cprintf("BMP280 Initialization Failed \r\n");
                                                     	}

                                                     	bool bme280p = bmp280.id == BME280_CHIP_ID;
                                                     	cprintf("BOSH:%s \r\n", bme280p ? "BME280" : "BMP280");
                                                     	HAL_Delay(2000);

                                                     	ccs811.addr = CCS811_I2C_ADDR_0;
                                                     		ccs811.i2c = &hi2c1;

                                                     		while (!CCS811_Init(&ccs811)) {
                                                     			cprintf("CCS811 Initialization Failed\r\n");
                                                     		}

                                                     	HAL_Delay(2000);

                                                     	for(;;) {
                                                     			if (LC709203_Read_Voltage(&lc709203) != LC709203F_STATUS_OK) {

                                                     			}
                                                     			cprintf("V: %d \n\r", lc709203.voltage);

                                                     			if (LC709203_Read_Cell_Percent(&lc709203) != LC709203F_STATUS_OK) {

                                                     			}
                                                     			cprintf("P: %d \n\r", lc709203.celllife);

                                                     			HAL_Delay(10000);

                                                     			while (!BMP280_Read_Float(&bmp280, &temperature, &pressure, &humidity)) {
                                                     				cprintf("Temperature/Pressure Reading Failed\r\n");
                                                     			    HAL_Delay(2000);
                                                     			}

                                                     			cprintf("Pressure: %.2f Pa \r\n", pressure);
                                                     			cprintf("Humidity: %.2f % \r\n", humidity);
                                                     			cprintf("Temperature: %.2f C \r\n", temperature);
                                                     			HAL_Delay(10000);

                                                     			CCS811_Measure_CO2_TVOC(&ccs811, &co2, &tvoc);
                                                     			cprintf("Co2 %d\r\n", co2);
                                                     			cprintf("TVoC %d\r\n", tvoc);

                                                     			HAL_Delay(10000);
                                                     		}
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Def Struct CCS811 / BME280 */
  /*bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_1;
  bmp280.i2c = &hi2c1;

  while (!bmp280_init(&bmp280, &bmp280.params)) {
  	cprintf("BMP280 Initialization Failed \r\n");
  }

  bool bme280p = bmp280.id == BME280_CHIP_ID;
  cprintf("BOSH:%s \r\n", bme280p ? "BME280" : "BMP280");

  ccs811.addr = CCS811_I2C_ADDR_0;
  ccs811.i2c = &hi2c1;

  while (!ccs811_init(&ccs811)) {
    	cprintf("CCS811 Initialization Failed\r\n");
  }*/

  //while (1)
  //{
  //HAL_Delay(2000);
  //HAL_GPIO_TogglePin(GPIOB, STATUS_LED_Pin);

	/*HAL_Delay(100);
	while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
		cprintf("Temperature/Pressure Reading Failed\r\n");
		HAL_Delay(2000);
	}
	HAL_Delay(100);

	cprintf("Pressure: %.2f Pa \r\n", pressure);
	cprintf("Humidity: %.2f % \r\n", humidity);
	cprintf("Temperature: %.2f C \r\n", temperature);
	HAL_Delay(1000);
e
	ccs811_measure_CO2_TVOC(&ccs811, &co2, &tvoc);
	cprintf("Co2 %d\r\n", co2);
	cprintf("TVoC %d\r\n", tvoc);*/
	// CCS811
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(5000);
    cprintf("B\n\r");
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */

	/*ccs811.addr = CCS811_I2C_ADDR_0;
	ccs811.i2c = &hi2c1;

	while (!CCS811_Init(&ccs811)) {
		cprintf("CCS811 Initialization Failed\r\n");
	}

	HAL_Delay(6000);*/

	/*LC709203_HandleTypedef lc709203;
	lc709203.addr = LC709203F_I2CADDR_DEFAULT;
	lc709203.power = LC709203F_POWER_OPERATE;
	lc709203.adjust= LC709203F_APA_500MAH;
	lc709203.tempmode = LC709203F_TEMPERATURE_THERMISTOR;
	lc709203.arsoc = 600;
	lc709203.i2c = &hi2c1;

	lc709203.version = 0;

	if (LC709203_Init(&lc709203) != LC709203F_STATUS_OK) {

	}

	if (LC709203_Read_IC_Version(&lc709203) != LC709203F_STATUS_OK) {

	}
	cprintf("LC709203_VIC %d\n\r", lc709203.version);

	HAL_Delay(5000); */

	/* Infinite loop */
	for(;;) {
		/*if (LC709203_Read_Voltage(&lc709203) != LC709203F_STATUS_OK) {

		}
		cprintf("V: %d \n\r", lc709203.voltage);

		if (LC709203_Read_Cell_Percent(&lc709203) != LC709203F_STATUS_OK) {

		}
		cprintf("P: %d \n\r", lc709203.celllife);*/

		//HAL_Delay(10000);

		/*while (!BMP280_Read_Float(&bmp280, &temperature, &pressure, &humidity)) {
			cprintf("Temperature/Pressure Reading Failed\r\n");
		    HAL_Delay(2000);
		}

		cprintf("Pressure: %.2f Pa \r\n", pressure);
		cprintf("Humidity: %.2f % \r\n", humidity);
		cprintf("Temperature: %.2f C \r\n", temperature);*/
		//HAL_Delay(10000);

		/*CCS811_Measure_CO2_TVOC(&ccs811, &co2, &tvoc);
		cprintf("Co2 %d\r\n", co2);
		cprintf("TVoC %d\r\n", tvoc);*/

		HAL_Delay(10000);
	}
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
