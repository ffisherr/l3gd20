
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI5_Init(void);
static void MX_UART4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


uint8_t dataRecieved	= 0;
uint8_t dataTransmitted	= 0;

float translate(uint16_t result){
		return result * 0.07;
}
 
float ms(float data){
	return 0.1F * data;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t address = 0;
	uint8_t data = 0;
	uint8_t data_return[2] = {0};
 
	uint16_t OUT_X_L = 0;	
	uint16_t OUT_Y_L = 0;
	uint16_t OUT_Z_L = 0;
 
	float fOUT_X_L;
	float fOUT_Y_L;
	float fOUT_Z_L;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI5_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  uint8_t mag = 0xAA;
  
  //****************************************
  HAL_UART_Transmit(&huart4, &mag, 1, 0x1000);
  //****************************************
  
  uint8_t who_am_i, counter = 0;
  
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
  HAL_Delay(2000);
  
  	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_RESET);
	address = 0x20;	//00100000 - CTRL_REG_1
	HAL_SPI_TransmitReceive(&hspi5, &address, &data, sizeof(data), 0x1000);
	address = 0x0F; //00001111
	HAL_SPI_TransmitReceive(&hspi5, &address, &data, sizeof(data), 0x1000);
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_SET);
	
  HAL_UART_Transmit(&huart4, &data, 1, 0x1000);
    
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_RESET);
	address = 0x8F; //10001111 - WHO_AM_I - READ
	HAL_SPI_TransmitReceive(&hspi5, &address, &data, sizeof(data), 0x1000);
	address = 0x00; //00000000
	HAL_SPI_TransmitReceive(&hspi5, &address, &data, sizeof(data), 0x1000);
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_SET);
	
  HAL_UART_Transmit(&huart4, &data, 1, 0x1000);
    
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_RESET);
	address = 0x23; //00100011 - CTRL_REG4 
	HAL_SPI_TransmitReceive(&hspi5, &address, &data, sizeof(data), 0x1000);
	address = 0x30;	//00110000
	HAL_SPI_TransmitReceive(&hspi5, &address, &data, sizeof(data), 0x1000);
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_SET);
  
  HAL_UART_Transmit(&huart4, &data, 1, 0x1000);
  
  HAL_Delay(3000);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        mag = 0xBB;
      
  //****************************************
        HAL_UART_Transmit(&huart4, &mag, 1, 0x1000);
  //****************************************
      
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_RESET);
		address = 0xE8;		
		HAL_SPI_TransmitReceive(&hspi5, &address, &data, sizeof(data), 0x1000);
		address = 0x00;
		HAL_SPI_TransmitReceive(&hspi5, &address, &data_return[0], sizeof(data_return[0]), 0x1000);
		address = 0x00;
		HAL_SPI_TransmitReceive(&hspi5, &address, &data_return[1], sizeof(data_return[1]), 0x1000);
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_SET);
      
		OUT_X_L = data_return[0] | (data_return[1] << 8);  
  //****************************************
        HAL_UART_Transmit(&huart4, (uint8_t*)&OUT_X_L, 2, 0x1000);
  //****************************************
		fOUT_X_L = translate(OUT_X_L);
		fOUT_X_L = ms(fOUT_X_L);	
        

  
		//;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_RESET);
		address = 0xEA;		
		HAL_SPI_TransmitReceive(&hspi5, &address, &data, sizeof(data), 0x1000);
		address = 0x00;
		HAL_SPI_TransmitReceive(&hspi5, &address, &data_return[0], sizeof(data_return[0]), 0x1000);
		address = 0x00;
		HAL_SPI_TransmitReceive(&hspi5, &address, &data_return[1], sizeof(data_return[1]), 0x1000);
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_SET);
        
		OUT_Y_L = data_return[0] | (data_return[1] << 8);
          //****************************************
        HAL_UART_Transmit(&huart4, (uint8_t*)&OUT_Y_L, 2, 0x1000);
  //****************************************
		fOUT_Y_L = translate(OUT_Y_L);
		fOUT_Y_L = ms(fOUT_Y_L);
        

  
		//;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_RESET);
		address = 0xEC;		
		HAL_SPI_TransmitReceive(&hspi5, &address, &data, sizeof(data), 0x1000);
		address = 0x00;
		HAL_SPI_TransmitReceive(&hspi5, &address, &data_return[0], sizeof(data_return[0]), 0x1000);
		address = 0x00;
		HAL_SPI_TransmitReceive(&hspi5, &address, &data_return[1], sizeof(data_return[1]), 0x1000);
	HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_SET);
		OUT_Z_L = data_return[0] | (data_return[1] << 8);  
        //****************************************
        HAL_UART_Transmit(&huart4, (uint8_t*)&OUT_Z_L, 2, 0x1000);
  //****************************************
        
		fOUT_Z_L = translate(OUT_Z_L);
		fOUT_Z_L = ms(fOUT_Z_L);
        

        mag = 0xCC;
        HAL_UART_Transmit(&huart4, &mag, 1, 0x1000);
        HAL_Delay(100);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HYRO_CSn_GPIO_Port, HYRO_CSn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HYRO_CSn_Pin */
  GPIO_InitStruct.Pin = HYRO_CSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(HYRO_CSn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
