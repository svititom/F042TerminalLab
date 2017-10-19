/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"	
#define dfm_master_tim htim1
#define dfm_slave_tim htim2
#define adc_tim	htim3
#define pwm_tim htim14

#define CONVERSIONS (50)
#define REFRESH_PERIOD_MS (200)
#define ADC_CHANNELS (3)
#define USB_ENUMERATION_DELAY_MS (2000)
//Ascii CRLF
#define LF (10)
#define CR (13)

//value from datasheet
#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t)0x1FFFF7BA))
#define defaultDisplaymode (General)
uint16_t adc_buffer[ADC_CHANNELS*CONVERSIONS];

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
enum DisplayMode{General, Voltmeter};
volatile enum DisplayMode currentMode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void show_help() {
	char buf[225];
 //wait for first transmission to finish with a bit of space
	snprintf(buf, 225, "\r\n\n\nh or ? -> show this help\r\nm -> change mode\r\nq/w -> inc/dec pwm duty cycle by 10%%\r\ne/r -> inc/dec pwm freq rough\r\nd/f -> inc/dec pwm freq smooth\r\n042 Terminal Lab - Tomas Svitil, Dept of Measurement, FEL, CTU in Prague\r\n\n");
	CDC_Transmit_FS((uint8_t *)buf, 225);

	
}

void receive_command(uint8_t* Buf){
	if(Buf[0] == 'm'){
		char infoMsg[50];
		if (currentMode != defaultDisplaymode) {
			currentMode = defaultDisplaymode;
			sprintf(infoMsg,"\r\nDefault, m to change, h for help\r\n");
		} else {
			currentMode = Voltmeter;
			sprintf(infoMsg,"\r\nVoltmeter, m to change, h for help\r\n");
		}
		CDC_Transmit_FS((uint8_t *)infoMsg, strlen(infoMsg));
			
	} else	if(Buf[0] == 'q'){
		//increase DC
		uint16_t duty_cycle = ((float)pwm_tim.Instance->CCR1+1)/((float)pwm_tim.Instance->ARR+1)*100;
		uint16_t step = pwm_tim.Instance->ARR / 100;
		if(step < 1) {step = 1;}

		pwm_tim.Instance->CCR1 += step;		
		
		if(duty_cycle > 99){
			pwm_tim.Instance->CCR1 = pwm_tim.Instance->ARR;
		}
		
	}	else if(Buf[0] == 'w'){
		//decrease DC
		uint16_t duty_cycle = ((float)pwm_tim.Instance->CCR1+1)/((float)pwm_tim.Instance->ARR+1)*100;
		uint16_t step = pwm_tim.Instance->ARR / 100;
		if(step < 1) {step = 1;}

		if(step >  pwm_tim.Instance->CCR1){
			//if the step would underflow the counter, just zero it
			pwm_tim.Instance->CCR1 = 0;
		}
		else {
			pwm_tim.Instance->CCR1 -= step;
		}
		
	}
	else if(Buf[0] == 'e'){
		//increase freq rough
		uint16_t arr = pwm_tim.Instance->ARR;
		uint16_t new_arr;
		uint16_t duty_cycle = ((float)pwm_tim.Instance->CCR1+1)/((float)arr+1)*100;
		if(arr < 2){
			new_arr = 1;
		}
		else{
			new_arr = arr / 2;
		}
		pwm_tim.Instance->CCR1 = new_arr * (float)((float)duty_cycle / (float)100);
		//Why was this here???
		for(uint8_t i = 0; i < 100; i++);
		pwm_tim.Instance->ARR = new_arr;
	}
	else if(Buf[0] == 'r'){
		//decrease freq rough
		uint16_t arr = pwm_tim.Instance->ARR;
		uint16_t new_arr;
		uint16_t duty_cycle = ((float)pwm_tim.Instance->CCR1+1)/((float)arr+1)*100;
		if(arr > 32676){
			new_arr = 65535;
		}
		else{
			new_arr = arr * 2;
		}
		pwm_tim.Instance->CCR1 = new_arr * (float)((float)duty_cycle / (float)100);
		for(uint8_t i = 0; i < 100; i++);
		pwm_tim.Instance->ARR = new_arr;
	}
	else if(Buf[0] == 'd'){
		//increase smooth
		uint16_t arr = pwm_tim.Instance->ARR;
		uint16_t new_arr;
		uint16_t duty_cycle = ((float)pwm_tim.Instance->CCR1+1)/((float)arr+1)*100;
		if(arr < 2){
			new_arr = 1;
		}
		else{
			new_arr = arr - 1;
		}
		pwm_tim.Instance->CCR1 = new_arr * (float)((float)duty_cycle / (float)100);
		pwm_tim.Instance->ARR = new_arr;
	}
	else if(Buf[0] == 'f'){
		//decrease smooth
		uint16_t arr = pwm_tim.Instance->ARR;
		uint16_t new_arr;
		uint16_t duty_cycle = ((float)pwm_tim.Instance->CCR1+1)/((float)arr+1)*100;
		if(arr > 65534){
			new_arr = 65535;
		}
		else{
			new_arr = arr + 1;
		}
		pwm_tim.Instance->CCR1 = new_arr * (float)((float)duty_cycle / (float)100);
		pwm_tim.Instance->ARR = new_arr;
	} else if(Buf[0] == '?' || Buf[0] == 'h'){
			show_help();
	} else if(Buf[0] == CR || Buf[0] == LF) {
		//echo new lines as a "freeze" function
		CDC_Transmit_FS("\r\n", 2);
	}
}
/* USER CODE END 0 */

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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	//if the delay is not long enough, the device will be reported as not woring
	HAL_Delay(USB_ENUMERATION_DELAY_MS);
	
	HAL_ADC_Stop(&hadc);
  if(HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) {
		CDC_Transmit_FS("ADC Error!\n", 11); 
	}
	HAL_Delay(100);

	//Start timers for Direct Frequency Measurement
	HAL_TIM_Base_Start(&dfm_slave_tim);
	HAL_TIM_IC_Start(&dfm_slave_tim, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&dfm_master_tim);
	HAL_TIM_OC_Start(&dfm_master_tim, TIM_CHANNEL_1);
	
	//Start PWM Generation
	HAL_TIM_Base_Start(&pwm_tim);
	HAL_TIM_PWM_Start(&pwm_tim, TIM_CHANNEL_1);
	
	//Start the ADC trigger timer
	HAL_TIM_Base_Start(&adc_tim);
	HAL_TIM_OC_Start(&adc_tim, TIM_CHANNEL_1);
	
  //And start the ADC in Direct Memory Access mode
	HAL_ADC_Start_DMA(&hadc, (uint32_t *)&adc_buffer, CONVERSIONS*ADC_CHANNELS);

	CDC_Transmit_FS("\nType h or ? for help\r\n", 23);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 uint32_t freq_dfm;
  uint32_t freq_pwm;
  uint32_t calc[ADC_CHANNELS];
	int16_t difference;
  uint32_t vdda;
	float duty_cycle;
  char message[120];
	
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	for(int i = 0; i < ADC_CHANNELS; i++) {
			calc[i] = 0;
		}
	  //calculate adc values, average from #CONVERSION samples
	  for(int i = 0; i < ADC_CHANNELS*CONVERSIONS; i+= ADC_CHANNELS){  
			
			calc[0] += adc_buffer[i];
		  calc[1] += adc_buffer[i+1];
			calc[2] += adc_buffer[i+2];
			
	  }
		
		calc[2] /= CONVERSIONS;
		vdda = 3300;
		vdda *= *(VREFINT_CAL_ADDR);
		vdda /= calc[2];
		
	  for(int i = 0; i < ADC_CHANNELS - 1; i++){
			calc[i] /= CONVERSIONS;
			calc[i] *= vdda;
			calc[i] /= 4095;
	  }
		switch (currentMode) {
			
			case Voltmeter:
				difference = calc[0] - calc[1];
			sprintf(message, "ADC PA1: _%4dmV PA2: _%4dmV, PA1-PA2: _%4dmV, Vdda: _%4dmV\r", calc[0], calc[1], difference, vdda);
				break;
			default:
			case General:
				  duty_cycle = (float)pwm_tim.Instance->CCR1/(float)pwm_tim.Instance->ARR*100;
					freq_dfm = __HAL_TIM_GET_COMPARE(&dfm_slave_tim, TIM_CHANNEL_1);
					freq_pwm =  (HAL_RCC_GetPCLK1Freq())/((&pwm_tim)->Instance->PSC+1)/((&pwm_tim)->Instance->ARR+1);	
					sprintf(message, "ADC PA1: _%4dmV PA2: _%4dmV, Vdda: _%4dmV,   Freq in PA0: _%8dHz, PWM out PA4: _%dHz, Duty: _%3.1f%% \r", calc[0], calc[1], vdda, freq_dfm,freq_pwm,duty_cycle);
	
				break;				
		}
		CDC_Transmit_FS((uint8_t *) message,strlen(message));
	  HAL_Delay(REFRESH_PERIOD_MS);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  RCC_CRSInitTypeDef RCC_CRSInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enable the SYSCFG APB clock 
    */
  __HAL_RCC_CRS_CLK_ENABLE();

    /**Configures CRS 
    */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4799;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 9999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 9999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim14);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
