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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sct.h"
#include "stdio.h"

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */






static uint8_t stop_watch = 0;
static uint32_t timer = 0;
static uint8_t timing = 0;
static uint8_t seconds = 0;
static uint8_t tenthsec = 0;

static uint8_t S1 = 0;
static uint32_t pressed_S1 = 0;
static uint8_t switch1 = 0;
static uint8_t switch1_count = 0;

static uint8_t S2 = 0;
static uint32_t pressed_S2 = 0;
static uint8_t switch2 = 0;
static uint8_t switch2_count = 0;

static uint8_t mid_time_sec = 0;
static uint8_t mid_time_tenth = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 void start_watch(){
 uint16_t seconds = 0;
 uint8_t tenthsec = 0;
 timer = HAL_GetTick();
 if(HAL_GetTick()-timer>100){
 tenthsec++;

 if(tenthsec>9){tenthsec=0;} //preteceni desetin sekundy
 }

 if(HAL_GetTick()-timer>1000){
 seconds++;
 if(seconds>99){seconds=0;} //preteceni sekund
 }
 }

 void stop_watch(){
 }

 void reset_watch(){
 }

 void show_mid_time(){

 }


 */

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

	sct_init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if(stop_watch == 0){

			switch(timing){
			case 0:
				timer = HAL_GetTick();	// casovac stopek
				timing = 1;
			break;

			case 1:
				if (HAL_GetTick() - timer > 100) {
					tenthsec++;
					if (tenthsec > 9) {
							tenthsec = 0;
						} //preteceni desetin sekundy
					}


				if (HAL_GetTick() - timer > 1000) {
					seconds++;
					if (seconds > 99) {
						seconds = 0;
					} //preteceni sekund
					timing = 0;
				}
			break;
			}
		}



		switch(S1){ // S1 s osetrenymi zakmity
		case 0:
			switch1 = HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin);
			if(switch1==0){
				switch1_count++;
				pressed_S1 = HAL_GetTick();
				S1 = 1;
			}
		break;

		case 1:
			if(HAL_GetTick() - pressed_S1>150){
				S1=0;
			}
		break;
		}


		switch(S2){ // S2 s osetrenymi zakmity
		case 0:
			switch2 = HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin);
			if(switch2==0){
				switch2_count++;
				pressed_S2 = HAL_GetTick();
				S2 = 1;
			}
		break;

		case 1:
			if(HAL_GetTick() - pressed_S2>150){
				S2=0;
			}
		break;
		}



		if (switch2_count == 1) {
			stop_watch = 1; // STOP
			mid_time_sec = seconds;
			mid_time_tenth = tenthsec;
		}

		if (switch2_count == 2) { // RESET
			seconds = 0;
			tenthsec = 0;
		}

		if (switch2_count == 3) { // START
			stop_watch = 0;
		}

		if (switch2_count > 3) {
			switch2_count = 0;
		} // udavani poctu stavu tlacitka



		if (switch1_count > 1){
			switch1_count = 0;
		}


		//switch(stopwatch){


		static uint32_t lastDisplayTicks = 0;

		if (HAL_GetTick() >= lastDisplayTicks + 100) { // osetreni pro prepisovani displeje

			if(switch1_count == 0){
				sct_value((seconds*10)+tenthsec , seconds/12, 2); // hodnota pro vypis, LED bargraf, desetinna carka
				}
			if(switch1_count == 1){
				sct_value((mid_time_sec*10)+mid_time_tenth , mid_time_sec/12, 2);
				}
		lastDisplayTicks = HAL_GetTick();
		}




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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCT_NOE_Pin|SCT_CLK_Pin|SCT_SDI_Pin|SCT_NLA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S2_Pin S1_Pin */
  GPIO_InitStruct.Pin = S2_Pin|S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCT_NOE_Pin SCT_CLK_Pin SCT_SDI_Pin SCT_NLA_Pin */
  GPIO_InitStruct.Pin = SCT_NOE_Pin|SCT_CLK_Pin|SCT_SDI_Pin|SCT_NLA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	while (1) {
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
