
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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "BME.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t tempread[8];
uint8_t config[2];
uint8_t comp[32];
volatile int temperature_raw, pressure_raw, humidity_raw = 0;
int finaltemp = 0;
uint32_t finalpressure, final_humidity = 0;
unsigned char dig_H1, dig_H3;
signed char dig_H6;
unsigned short dig_T1, dig_P1;
signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

	//Config
	BME280_CONFIG_SETUP();
	
	//Get Comp values
	BME280_GET_COMP_VALS();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	BME280_GET_RAW_VALS();
	BME280_CALC_FINAL_VALS();
  }
  /* USER CODE END 3 */

}

void BME280_CONFIG_SETUP(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(10);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SET CS LOW
	config[0] = CTRLMEASREG;
	config[1] = CTRLMEASVAL;
	HAL_SPI_Transmit(&hspi2, config, 2, 1000); //CONFIG
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(10);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SET CS LOW
	config[0] = CONFIGREG;
	config[1] = CONFIGVAL;
	HAL_SPI_Transmit(&hspi2, config, 2, 1000); //CONFIG
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(10);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SET CS LOW
	config[0] = CTRLHUMREG;
	config[1] = CTRLHUMVAL;
	HAL_SPI_Transmit(&hspi2, config, 2, 1000); //CONFIG
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(10);
}

void BME280_GET_COMP_VALS(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SET CS LOW
	config[0] = COMPTEMPPRES;
	HAL_SPI_Transmit(&hspi2, config, 1, 10); 
	HAL_SPI_Receive(&hspi2, comp, 24, 120);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(10);
	
	dig_T1 = (comp[0])+(comp[1]<<8);
	dig_T2 = (comp[2])+(comp[3]<<8);
	dig_T3 = (comp[4])+(comp[5]<<8);
	dig_P1 = (comp[6])+(comp[7]<<8);
	dig_P2 = (comp[8])+(comp[9]<<8);
	dig_P3 = (comp[10])+(comp[11]<<8);
	dig_P4 = (comp[12])+(comp[13]<<8);
	dig_P5 = (comp[14])+(comp[15]<<8);
	dig_P6 = (comp[16])+(comp[17]<<8);
	dig_P7 = (comp[18])+(comp[19]<<8);
	dig_P8 = (comp[20])+(comp[21]<<8);
	dig_P9 = (comp[22])+(comp[23]<<8);
	
	config[0] = COMPHUMINIT;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SET CS LOW
	HAL_SPI_Transmit(&hspi2, config, 1, 10); 
	HAL_SPI_Receive(&hspi2, &comp[24], 1, 120);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(10);
	dig_H1 = comp[24];

	config[0] = COMPHUMREST;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SET CS LOW
	HAL_SPI_Transmit(&hspi2, config, 1, 10); 
	HAL_SPI_Receive(&hspi2, &comp[25], 7, 120);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(10);
	dig_H2 = (comp[25])+(comp[26]<< 8);
	dig_H3 = comp[27];
	dig_H4 = (comp[28] << 4) +(comp[29] & 0xF);
	dig_H5 = (comp[29] & 0xF0) +(comp[30]<< 4);
	dig_H6 = comp[31];
}

void BME280_GET_RAW_VALS(void){
  BME280_CONFIG_SETUP();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SET CS LOW
	config[0] = RAWREAD;
	HAL_SPI_Transmit(&hspi2, config, 1, 10); //GET ID
	HAL_SPI_Receive(&hspi2, tempread, 8, 10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
	temperature_raw =(tempread[3]<<12)+(tempread[4]<<4)+(tempread[5]>>4);
	pressure_raw = (tempread[0]<<12)+(tempread[1]<<4)+(tempread[2]>>4);
	humidity_raw = (tempread[6] << 8) + (tempread[7]);
}

void BME280_CALC_FINAL_VALS(void){
	int var1, var2, t_fine;
	var1 = ((((temperature_raw >> 3) - ((int)dig_T1 << 1))) * ((int)dig_T2)) >> 11;
	var2 = (((((temperature_raw >> 4) - ((int)dig_T1)) * ((temperature_raw >> 4) - ((int)dig_T1))) >> 12) * ((int)dig_T3)) >> 14;
	t_fine = (var1 + var2);
	finaltemp = (t_fine * 5 + 128) >> 8;
	
	var1 = (((int)t_fine) >> 1) - 64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int)dig_P6);
	var2 = var2 + ((var1 * ((int)dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int) dig_P4) << 16);
	var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int) dig_P2) * var1) >> 1 )) >> 18;
	var1 = ((((32768 + var1)) * ((int)dig_P1)) >> 15); 
	if (var1 == 0)
	{
		finalpressure = 0;
	}
	else{
		finalpressure = (((uint32_t) (((int)1048576)-pressure_raw) - (var2 >> 12))) * 3125;
		if (finalpressure < 0x80000000){
			finalpressure = (finalpressure << 1) / (( uint32_t)var1);
		}
		else{
			finalpressure = (finalpressure / (uint32_t)var1) * 2;
		}
		var1 = (((int)dig_P9) * ((int) ((( finalpressure >> 3) * ( finalpressure >> 3)) >> 13))) >> 12;
		var2 = (((int) (finalpressure >> 2)) * ((int)dig_P8)) >> 13;
		finalpressure = ((uint32_t)((int)finalpressure + ((var1 + var2 + dig_P7) >> 4)))/100; //kPA
	}
	
	var1 = (t_fine - ((int) 76800));
	var1 = (((((humidity_raw << 14) - (((int) dig_H4) << 20) - (((int)dig_H5) * var1)) + ((int) 16384)) >> 15) * \
	(((((((var1 * ((int) dig_H6)) >> 10) * (((var1 * ((int) dig_H3)) >> 11) + ((int) 32768))) >> 10) + \
	((int) 2097152)) * ((int) dig_H2) + 8192) >> 14));
	
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int)dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419330400 : var1);
	final_humidity = (var1 >> 12)/1024;	
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
