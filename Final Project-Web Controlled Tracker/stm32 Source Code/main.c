
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int avt,avd,avl,avr,dvert,dhoriz;
int MinServoVal = 600;
int MaxServoVal = 2100;
int percentageDifference = 30;
int XAccumulator = 1500;
int YAccumulator = 1500;
int LDR1,LDR2,LDR3,LDR4;
char test2[48];
uint8_t receive[1];
uint32_t adc1, adc2,adc3,adc4;
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
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  	HAL_TIM_Base_Start(&htim2);
  	// Start PWM at for TIM2 CHANNEl 1 and CHANNEL 2
  	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, YAccumulator);
  	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, XAccumulator);
	 HAL_Delay(200);
    while (1)
    {

    	 	 HAL_ADC_Start(&hadc2);//Start ADC2 to read Values
    		  HAL_ADC_PollForConversion(&hadc2, 100);//Read the first Conversion read from LDR1
    		  adc1 = HAL_ADC_GetValue(&hadc2);
    		  HAL_ADC_PollForConversion(&hadc2, 100);//Read the second Conversion read from LDR2
    		  adc2 = HAL_ADC_GetValue(&hadc2);
    		  HAL_ADC_PollForConversion(&hadc2, 100);//Read the third Conversion read from LDR3
    		 adc3 = HAL_ADC_GetValue(&hadc2);
    		 HAL_ADC_PollForConversion(&hadc2, 100);//Read the first Conversion read from LDR4
    		 adc4 = HAL_ADC_GetValue(&hadc2);
    		  HAL_Delay (200);
    		  HAL_ADC_Stop (&hadc2);
    		  LDR1 = (int)adc1;//------		Convert 			       ------//
    		  LDR2 = (int)adc2;//------		ADC Readings			   ------//
    		  LDR3 = (int)adc3;//------		to type (INT) so		   ------//
    		  LDR4 = (int)adc4;//------		that proper value is sent   ------//
    		  checkForCommand();//Check if Any commands have been sent by the user
    		  sprintf(test2,"LDR1: %d\r\nLDR2: %d\r\nLDR3: %d\r\nLDR4: %d\r\n",LDR1,LDR2,LDR3,LDR4);//Format the data to be sent
    		  HAL_UART_Transmit(&huart2,test2, sizeof(test2), 10); //Transmit Data to Raspberry Pi
    		  HAL_Delay(100); //Small Delay to give UART enough time to send all the data
    }
}
void checkForCommand()
{
	HAL_UART_Receive_IT(&huart2,receive,sizeof(receive));
	if(receive[0] == 'A')
	{
		 avt = (LDR1 + LDR2) / 2; // average value top
		 avd = (LDR3 + LDR4) / 2; // average value down
		 avl = (LDR1 + LDR3) / 2; // average value left
		 avr = (LDR2 + LDR4) / 2; // average value right
		 dvert = avt - avd; // check the difference of up and down
		 dhoriz = avl - avr;// check the difference of left and right
		 if(-1 * percentageDifference > dvert || dvert > percentageDifference)
		 {
			 if(avt > avd)
			 {
				 ++YAccumulator;
				 if(YAccumulator > MaxServoVal)
					 YAccumulator = MaxServoVal;
			 }
			 else if(avt < avd)
			 {
				 --YAccumulator;
				 if(YAccumulator < MinServoVal)
					 YAccumulator = MinServoVal;
			 }
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, YAccumulator);
		 }
		 if(-1 * percentageDifference > dhoriz || dhoriz > percentageDifference)
		 {
			 if(avl > avr)
			 {
				 ++XAccumulator;
				 if(XAccumulator > MaxServoVal)
					 XAccumulator = MaxServoVal;
			 }
			 else if(avl < avr)
			 {
				 --XAccumulator;
				 if(XAccumulator < MinServoVal)
					 XAccumulator = MinServoVal;
			 }
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, XAccumulator);
		 }
		 HAL_Delay(200);
	}
	else if(receive[0] == 'U')
	{
		if(YAccumulator > MinServoVal)
		{
			if(YAccumulator - 150 < MinServoVal)//Checks this condition, to prevent the servo from Moving from its Min Value
				YAccumulator = MinServoVal;
			else
				YAccumulator -=150;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, YAccumulator);
			HAL_Delay(200);
			receive[0]= '\0';
			return;
		}
	}
	else if (receive[0] == 'D')
	{
		if(YAccumulator < MaxServoVal)
		{
			if(YAccumulator + 150 > MaxServoVal)//Checks this condition, to prevent the servo from Moving from its Max Value
				YAccumulator = MaxServoVal;
			else
				YAccumulator +=150;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, YAccumulator);
			HAL_Delay(200);
			receive[0]= '\0';
			return;
		}
	}
	else if(receive[0] == 'L')
	{
		if(XAccumulator < MaxServoVal)
		{
			if(XAccumulator + 150 > MaxServoVal)//Checks this condition, to prevent the servo from Moving from its Max Value
				XAccumulator = MaxServoVal;
			else
				XAccumulator +=150;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, XAccumulator);
			HAL_Delay(100);
			receive[0]= '\0';
			return;

		}
	}
	else if (receive[0] == 'R')
	{
		if(XAccumulator > MinServoVal)
		{
			if(XAccumulator - 150 < MinServoVal)//Checks this condition, to prevent the servo from Moving from its Min Value
				XAccumulator = MinServoVal;
			else
				XAccumulator -=150;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, XAccumulator);
			HAL_Delay(100);
			receive[0]= '\0';
			return;
		}
	}
	else
		return;

}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 50000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	__NOP();
}
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
