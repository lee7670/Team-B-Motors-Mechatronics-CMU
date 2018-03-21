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
#include "stm32l1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
//#include <math.h>
#include <stdlib.h>
#include "pid_controller.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
#define WHEELRAD 90.0
#define CENTERDIS 350
#define KP 0
#define KI 0
#define KD 0
PIDControl PIDR;
PIDControl PIDL;
unsigned char Rx_indx, Rx_data[2], Rx_Buffer[100];
int Transfer_cplt;
enum states {
  STP,
  MOV
};

enum states state;
struct motor{
	float setRPM;
	bool dir;
	float distance_traveled;
	float prevpos;
	float setDis;
	uint16_t prevcount;
};
struct motor left;
struct motor right;
bool distance_set = false;
bool angle_set = false;
/* USER CODE END PV */
//reassign printf to usart1
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void setLin(float dis, float spd);
void setArc(float r, float w, float phi);
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period,
uint16_t pulse);
static void initMot(void);

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim10);
  HAL_TIM_Base_Start(&htim11);
  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim5);
  Transfer_cplt = 0;
  HAL_UART_Receive_IT(&huart1, Rx_data, 1);   //activate UART receive interrupt
  initMot();
  PIDInit(&PIDR, KP, KI, KD, .1, 0, 255, AUTOMATIC, DIRECT);
  PIDInit(&PIDL, KP, KI, KD, .1, 0, 255, AUTOMATIC, DIRECT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */
	  if(Transfer_cplt >= 1)
	  {
		  char cmd[100];
		  memset(cmd,0,sizeof cmd);
		  strcpy(cmd, Rx_Buffer);
		  cmd[Transfer_cplt]='\0';
		  char* tkpnt;
		  tkpnt = strtok(cmd, " ");
		  if(strncmp(tkpnt, "l",1)==0){
		  }else if(strncmp(tkpnt, "l",1)==0){
			tkpnt = strtok(NULL, " ");
			float lindis = atof(tkpnt);
			tkpnt = strtok(NULL, " ");
			float linspd = atof(tkpnt);
			setLin(lindis, linspd);
			state = MOV;
		  }
		  else if (strncmp(tkpnt, "r",1)==0)
		  {
			tkpnt = strtok(NULL, " ");
			float rotomega = atof(tkpnt);
			tkpnt = strtok(NULL, " ");
			float rotphi = atof(tkpnt);
			float rotR = 0.0;
			setArc(rotR, rotomega, rotphi);
			state = MOV;
		  }
		  else if (strncmp(tkpnt, "a",1)==0)
		  {
			tkpnt = strtok(NULL, " ");
			float rotR = atof(tkpnt);
			tkpnt = strtok(NULL, " ");
			float rotomega = atof(tkpnt);
			tkpnt = strtok(NULL, " ");
			float rotphi = atof(tkpnt);
			setArc(rotR, rotomega, rotphi);
			state = MOV;
		  }
		  else
		  {
			state = STP;
		  }
	  }

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 3200000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim10, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim10);

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim11, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim11);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
     PA2   ------> LCD_SEG1
     PA3   ------> LCD_SEG2
     PA4   ------> ADC_IN4
     PC4   ------> TS_G9_IO1
     PC5   ------> TS_G9_IO2
     PB0   ------> TS_G3_IO1
     PB1   ------> TS_G3_IO2
     PB10   ------> LCD_SEG10
     PB11   ------> LCD_SEG11
     PB12   ------> LCD_SEG12
     PB13   ------> LCD_SEG13
     PB14   ------> LCD_SEG14
     PB15   ------> LCD_SEG15
     PC6   ------> LCD_SEG24
     PC7   ------> LCD_SEG25
     PC8   ------> LCD_SEG26
     PC9   ------> LCD_SEG27
     PA8   ------> LCD_COM0
     PA9   ------> LCD_COM1
     PA10   ------> LCD_COM2
     PA15   ------> LCD_SEG17
     PC10   ------> LCD_SEG28
     PC11   ------> LCD_SEG29
     PB3   ------> LCD_SEG7
     PB4   ------> LCD_SEG8
     PB5   ------> LCD_SEG9
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IDD_CNT_EN_Pin|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : IDD_CNT_EN_Pin PC0 PC1 PC2 
                           PC3 */
  GPIO_InitStruct.Pin = IDD_CNT_EN_Pin|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG1_Pin SEG2_Pin COM0_Pin COM1_Pin 
                           COM2_Pin SEG12_Pin */
  GPIO_InitStruct.Pin = SEG1_Pin|SEG2_Pin|COM0_Pin|COM1_Pin 
                          |COM2_Pin|SEG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IDD_Measurement_Pin */
  GPIO_InitStruct.Pin = IDD_Measurement_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IDD_Measurement_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GRP9_Sampling_Pin GRP9_Ground_Pin */
  GPIO_InitStruct.Pin = GRP9_Sampling_Pin|GRP9_Ground_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GRP3_Sampling_Pin GRP3_Ground_Pin */
  GPIO_InitStruct.Pin = GRP3_Sampling_Pin|GRP3_Ground_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG6_Pin SEG7_Pin SEG8_Pin SEG9_Pin 
                           SEG10_Pin SEG11_Pin SEG3_Pin SEG4_Pin 
                           SEG5_Pin */
  GPIO_InitStruct.Pin = SEG6_Pin|SEG7_Pin|SEG8_Pin|SEG9_Pin 
                          |SEG10_Pin|SEG11_Pin|SEG3_Pin|SEG4_Pin 
                          |SEG5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG18_Pin SEG19_Pin SEG20_Pin SEG21_Pin 
                           SEG22_Pin SEG23_Pin */
  GPIO_InitStruct.Pin = SEG18_Pin|SEG19_Pin|SEG20_Pin|SEG21_Pin 
                          |SEG22_Pin|SEG23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(state == STP){
		return;
	}
	if(right.dir == true) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	} else if (right.setRPM == 0.0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	}
	if(left.dir == true) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	} else if (left.setRPM == 0.0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	}
	uint16_t newposition1;
	uint16_t newposition2;
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
		if(right.prevcount < __HAL_TIM_GET_COUNTER(&htim3)){
			newposition1 = 65535-__HAL_TIM_GET_COUNTER(&htim3) +right.prevcount;
		}else{
			newposition1 =abs(__HAL_TIM_GET_COUNTER(&htim3)-right.prevcount);
		}
	}else{
		if(right.prevcount < __HAL_TIM_GET_COUNTER(&htim3)){
			newposition1 = __HAL_TIM_GET_COUNTER(&htim3) + 65535 - right.prevcount;
		}else{
			newposition1 =abs(__HAL_TIM_GET_COUNTER(&htim3)-right.prevcount);
		}
	}
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
			if(left.prevcount < __HAL_TIM_GET_COUNTER(&htim2)){
				newposition2 = 65535-__HAL_TIM_GET_COUNTER(&htim2) +left.prevcount;
			}else{
				newposition2 =abs(__HAL_TIM_GET_COUNTER(&htim2)-left.prevcount);
			}
		}else{
			if(left.prevcount < __HAL_TIM_GET_COUNTER(&htim3)){
				newposition2 = __HAL_TIM_GET_COUNTER(&htim2) + 65535 - left.prevcount;
			}else{
				newposition2 =abs(__HAL_TIM_GET_COUNTER(&htim2)-left.prevcount);
			}
		}
	float vel1 = ((float)newposition1) * 1000.0 / (100.0); //encoder pulses per second
	float rpm1 = ((vel1 * 60)/1500); //Measured motor RPM
	float vel2 = ((float)newposition2) * 1000.0 / (100.0); //encoder pulses per second
	float rpm2 = ((vel2 * 60)/1500);
	float realspeed1 = (rpm1 * 2 * M_PI *WHEELRAD)/60; //Linear speed in mm/s
	float realspeed2 = (rpm2 * 2 * M_PI *WHEELRAD)/60; //Linear speed in mm/s
	if (right.dir)
	{
	rpm1 = -1*rpm1;
	}
	if (left.dir)
	{
	rpm2 = -1*rpm2;
	}
	PIDSetpointSet(&PIDR,right.setRPM);
	PIDInputSet(&PIDR,rpm1);
	PIDSetpointSet(&PIDL,left.setRPM);
	PIDInputSet(&PIDL,rpm2);
	PIDCompute(&PIDR);
	PIDCompute(&PIDL);
	uint8_t speed1 = (uint8_t)PIDOutputGet(&PIDR);
	uint8_t speed2 = (uint8_t)PIDOutputGet(&PIDL);
	setPWM(htim11,TIM_CHANNEL_1, 255, (uint8_t)speed1);
	setPWM(htim10,TIM_CHANNEL_1, 255, (uint8_t)speed2);
    right.distance_traveled = right.distance_traveled + (realspeed1*100*1e-3); //integrate linear velocity to obtain distance
	left.distance_traveled = left.distance_traveled + (realspeed2*100*1e-3); //integrate linear velocity to obtain distance
	if ((abs(right.distance_traveled-right.setDis) <= 5)||(abs(left.distance_traveled-left.setDis) <= 5) ){
		right.setRPM = 0.0; //Brake
		right.dir = false;
		left.setRPM = 0.0; //Brake
		left.dir = false;
		right.setDis = 0.0;
		left.setDis = 0.0;
		angle_set = false; //Re-initialize the targeting to an angle rotation
		distance_set = false;
		state = STP;
	}
	right.prevpos = newposition1;
	left.prevpos = newposition2;
	return;
}
//Interrupt callback routine
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    if (huart->Instance == USART1)  //current UART
        {
        if (Rx_indx==0) {for (i=0;i<100;i++) Rx_Buffer[i]=0;}   //clear Rx_Buffer before receiving new data

        if (Rx_data[0]!=10) //if received data different from ascii 10 (newline)
            {
            Rx_Buffer[Rx_indx++]=Rx_data[0];    //add data to Rx_Buffer
            }
        else            //if received data = 13
            {
        	Transfer_cplt=Rx_indx;//transfer complete, data is ready to read
            Rx_indx=0;
            }

        HAL_UART_Receive_IT(&huart1, Rx_data, 1);   //activate UART receive interrupt every time
        }

}
void setLin(float dis, float spd){
  right.setRPM = (spd/(2*M_PI*WHEELRAD))*60;
  left.setRPM = right.setRPM;

  if (right.setRPM < 0){
    right.setRPM = -1*right.setRPM;
    left.setRPM = -1*left.setRPM;
    right.dir = true;
    left.dir = true;
  }
  else
  {
    right.dir = false;
    left.dir = false;
  }

  right.setDis = dis;
  left.setDis = dis;
  //Re-initialize targeting to a rotation/distance
  right.distance_traveled = 0.0;
  left.distance_traveled = 0.0;

  return;
}
void setArc(float R/*mm*/, float w/*degrees/s*/, float phi/*degrees*/){
  float scalingfactor = 60.0/360.0;
  right.setRPM = (w*(R+CENTERDIS))*scalingfactor*(1/WHEELRAD);
  left.setRPM = (w*(R-CENTERDIS))*scalingfactor*(1/WHEELRAD);

  if (right.setRPM < 0){
    right.setRPM = -1*right.setRPM;
    right.dir = true;
  }
  else
  {
    right.dir = false;
  }

  if (left.setRPM < 0){
    left.setRPM = -1*left.setRPM;
    left.dir = true;
  }
  else
  {
    left.dir = false;
  }
  right.setDis = abs((R+CENTERDIS)*phi*M_PI/180);
  left.setDis = abs((R-CENTERDIS)*phi*M_PI/180);
  if (phi < 0){
	  left.setRPM = -1*left.setRPM;
	  left.dir = true;
	  right.setRPM = -1*right.setRPM;
	  right.dir = true;
  }
  //Re-initialize targeting to a rotation/distance
  right.distance_traveled = 0.0;
  left.distance_traveled = 0.0;
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
/* USER CODE BEGIN 4 */
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period,
uint16_t pulse)
{
 HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
 TIM_OC_InitTypeDef sConfigOC;
 timer.Init.Period = period; // set the period duration
 HAL_TIM_PWM_Init(&timer); // Reinitialize with new period value
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = pulse; // set the pulse duration
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
 HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}
static void initMot(){
	right.prevcount = __HAL_TIM_GET_COUNTER(&htim3);
	left.prevcount = __HAL_TIM_GET_COUNTER(&htim2);
	right.dir = false;
	left.dir = false;
	right.setRPM = 0.0;
	right.setDis = 0.0;
	right.distance_traveled=0.0;
	right.prevpos = 0.0;
	left.setRPM = 0.0;
	left.setDis = 0.0;
	left.distance_traveled=0.0;
	left.prevpos = 0.0;
}
/* USER CODE END 4 */
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
