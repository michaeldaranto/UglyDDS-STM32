/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "si5351.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define f_min       6950000      // change to suit
#define f_max       7250000


volatile uint32_t	count1=0;
volatile uint32_t	count2=0;
volatile uint8_t 	flag =0;
volatile uint32_t	vfo = 7030000  ; //start freq - change to suit
volatile uint32_t	radix = 100;  //start step size - change to suit
const	 uint32_t	BW = 2400;
const	 uint32_t	IF = 12000000;
const	 uint32_t	bfo=IF-(BW/2);
const 	int32_t 	correction = 978;

uint8_t		fflag=0x00;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void display_init();
void display_radix();
void display_frequency();
void Init_Si5351();
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  display_init();
  display_frequency();
  HAL_Delay(100);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  Init_Si5351();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void display_init() {

	ssd1306_Init();
  HAL_Delay(100);
 // ssd1306_Fill(Black);
 // ssd1306_UpdateScreen();
  ssd1306_SetCursor(0, 50);
  ssd1306_WriteString("UglyDD5", Font_7x10, White);
  HAL_Delay(100);
  ssd1306_SetCursor(0, 0);

  display_frequency();
  display_radix();
  HAL_Delay(100);
  ssd1306_UpdateScreen();
  HAL_Delay(100);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

  count1= (__HAL_TIM_GET_COUNTER(htim))/4;

  	if (count2>count1)
  			vfo -= radix;
   	if (vfo <= f_min)
  	  		vfo = f_min;
  	if(count2<count1)
  			vfo += radix;
  	if (vfo >= f_max)
  	  		vfo = f_max;

  count2=count1;

  display_frequency();

  si5351_SetupCLK0((bfo-vfo), SI5351_DRIVE_STRENGTH_4MA);
  si5351_EnableOutputs((1<<0) | (1<<2));
 // HAL_Delay(100);
  fflag =0xFF;
  }

void Int2Char(uint32_t f){
	__disable_irq();
	char g[11];
	  g[0]= ((f % 1000000000) / 100000000);
	  g[1]= ((f % 100000000) / 10000000);
      g[2]= ((f % 10000000) / 1000000);
      g[3]= -2;
      g[4]= ((f % 1000000) / 100000);
      g[5]= ((f % 100000) / 10000);
      g[6]= ((f % 10000) / 1000);
      g[7]= -2;
      g[8]= ((f % 1000) / 100);
      g[9]= ((f % 100) / 10);
      g[10]= ((f % 10) / 1);

    uint8_t count=0;
    while(f!=0) {
      		  f=f/10;
      		  count++;
      		}

    for (uint8_t i =(9-count); i<11; i++){
    	ssd1306_WriteChar(g[i]+48, Font_11x18, White);
      }
    __enable_irq();
}

void Init_Si5351(){

    	si5351_Init(correction);
       	si5351_SetupCLK0((bfo-vfo), SI5351_DRIVE_STRENGTH_4MA);
       	si5351_SetupCLK2(bfo, SI5351_DRIVE_STRENGTH_4MA);


       	si5351_EnableOutputs((1<<0) | (1<<2));
}

void display_radix(){

	ssd1306_SetCursor(78, 50);

	switch (radix) {
    case 1:
    	ssd1306_WriteString("    1", Font_7x10, White);
           break;
    case 10:
    	ssd1306_WriteString("   10", Font_7x10, White);
           break;
    case 100:
    	ssd1306_WriteString("  100", Font_7x10, White);
           break;
    case 1000:
    	ssd1306_WriteString("   1k", Font_7x10, White);
           break;
    case 10000:
    	ssd1306_WriteString("  10k", Font_7x10, White);
           break;
    case 100000:
    	ssd1306_WriteString(" 100k", Font_7x10, White);
           break;
 }

   	ssd1306_WriteString("Hz", Font_7x10, White);
   	ssd1306_UpdateScreen();
}

void display_frequency(){

	ssd1306_SetCursor(0, 0);
	Int2Char(vfo);
	ssd1306_UpdateScreen();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  if(GPIO_Pin == GPIO_PIN_0) {

	  switch (radix) {
	 	  	        case 1:
	 	  	          radix = 10;
	 	  	          break;
	 	  	        case 10:
	 	  	          radix = 100;
	 	  	          break;
	 	  	        case 100:
	 	  	          radix = 1000;
	 	  	          break;
	 	  	        case 1000:
	 	  	          radix = 10000;
	 	  	          break;
	 	  	        case 10000:
	 	  	          radix = 100000;
	 	  	          break;
	 	  	        case 100000:
	 	  	          radix = 1;
	 	  	          break;
	 	  	      }
	  display_radix();

  } else {
      __NOP();
  }
}

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
