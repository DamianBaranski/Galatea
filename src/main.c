/**
 * @file main.c
 * @brief    main func
 * @version  V0.1
 * @date     1.02.2016
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include "cmsis_os.h"
#include "hardware.h"
#include <string.h>
#include <math.h>
#include "midlvl.h"
#include "floodfill.h"
#include <stdlib.h>
#include "print.h"

#define ILOSC_ANALIZOWANYCH 30

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId LogHandle;
osThreadId IR_ReadHandle;
osThreadId Encoder_ReadHandle;
osThreadId IMU_ReadHandle;
osThreadId KalmanHandle;
osThreadId Get_VoltageHandle;
osMessageQId LogQueueHandle;
osMutexId IR_DataHandle;
osMutexId IMU_DataHandle;
osMutexId Encoder_DataHandle;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartLog(void const * argument);
void StartIR_Read(void const * argument);
void StartEncoder_Read(void const * argument);
void StartIMU_Read(void const * argument);
void StartKalman(void const * argument);
void StartGet_Voltage(void const * argument);

/* USER CODE BEGIN PFP */
const float dT = 0.004; //ms
const float mnoznik_PID = 0.5;
#define MM2TICK 17//17.51592//44pola*40impulsow/100.48mm/obr
#define TICK2MM 0.058823//0.0570909

//Wszystkie dane do sterownika PID
/*struct
{
	float P;    //Nstawy sterownika
	float I;
	float D;

	float zadVl;  //Zadana prędkość koła Lewego
	float zadVr;  //Zadana prędkość koła prawego
	float pomVl;  //Prędkość koła Lewego
	float pomVr;  //Prędkość koła prawego
	//Chronione potrzebne do pid-a
	float Il; //Warotość całki
	float Ir;
	float Dl; //Warotość pochodnej
	float Dr;
	float sterL;
	float sterR;
	float errorL;
	float errorR;
} PID;*/

struct
{
	float P;    //Nstawy sterownika
	float I;
	float D;

	double zadSl; //zadana droga koła lewego
	double zadSr; //zadana droga koła prawego
	float zadVl;  //Zadana prędkość koła Lewego
	float zadVr;  //Zadana prędkość koła prawego
	double pomSl;  //droga koła Lewego
	double pomSr;  //droga koła prawego
	//Chronione potrzebne do pid-a
	float Il; //Warotość całki
	float Ir;
	float Dl; //Warotość pochodnej
	float Dr;
	float sterL;
	float sterR;
	float errorL;
	float errorR;
} PID;


floodfill FF;


float old_opticMeasure[4]; //Sprawdzanie scian //do wywalenia

float opticMeasure[4][ILOSC_ANALIZOWANYCH]; //pomiary podczas jednego ruchu
unsigned int numer_pomiaru=0;
float MeasureIR[2];    //Aktualny pomiar bocznych czujników
float V_max=200;//0; //Prędkość maksymalna
float V_pos;//wyliczone z V_max, V_max_curve
float a=250;//1000;	//Przyspieszenia robota
float turn_a=200;//przyspieszenie robota na obrotach
float V_max_curve=95*2;//*2; //Prędkość maksymalna na zakrętach // dla skretu na 80 mm
float curve_a=100.54*2;//*2; //przyspieszenie robota na zakrętach //dla skrętu na 80 mm
const float rozstaw_kol=32;
float mid=90;//srednia wartosc odczytu

/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int Min(int A, int B)
{
	return (A>B)?B:A;
}
int Abs(int A)
{
	return (A<0)?-A:A;
}

/* USER CODE END 0 */

int main(void)
{
	/* USER CODE BEGIN 1 */
	PID.P=25;
	PID.I=0.15;
	PID.D=1.5;

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_USART3_UART_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of IR_Data */
	//osMutexDef(IR_Data);
	//IR_DataHandle = osMutexCreate(osMutex(IR_Data));

	/* definition and creation of IMU_Data */
	//osMutexDef(IMU_Data);
	//IMU_DataHandle = osMutexCreate(osMutex(IMU_Data));

	/* definition and creation of Encoder_Data */
	//osMutexDef(Encoder_Data);
	//Encoder_DataHandle = osMutexCreate(osMutex(Encoder_Data));

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of Log */
	osThreadDef(Log, StartLog, osPriorityLow, 0, 128);
	LogHandle = osThreadCreate(osThread(Log), NULL);

	/* definition and creation of IR_Read */
	osThreadDef(IR_Read, StartIR_Read, osPriorityNormal, 0, 128);
	IR_ReadHandle = osThreadCreate(osThread(IR_Read), NULL);

	/* definition and creation of Encoder_Read */
	osThreadDef(Encoder_Read, StartEncoder_Read, osPriorityHigh, 0, 128);
	Encoder_ReadHandle = osThreadCreate(osThread(Encoder_Read), NULL);

	/* definition and creation of IMU_Read */
	//osThreadDef(IMU_Read, StartIMU_Read, osPriorityNormal, 0, 128);
	//IMU_ReadHandle = osThreadCreate(osThread(IMU_Read), NULL);

	/* definition and creation of Kalman */
	//osThreadDef(Kalman, StartKalman, osPriorityNormal, 0, 128);
	//KalmanHandle = osThreadCreate(osThread(Kalman), NULL);

	/* definition and creation of Get_Voltage */
	osThreadDef(Get_Voltage, StartGet_Voltage, osPriorityLow, 0, 128);
	Get_VoltageHandle = osThreadCreate(osThread(Get_Voltage), NULL);

	/* Create the queue(s) */
	/* definition and creation of LogQueue */
	//osMessageQDef(LogQueue, 64, uint16_t);
	//LogQueueHandle = osMessageCreate(osMessageQ(LogQueue), NULL);

	/* Start scheduler */
	osKernelStart();
	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1);
	/* USER CODE END 3 */
}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
			|RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12
			|RCC_PERIPHCLK_ADC34;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV2;
	PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV2;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/10000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
	HAL_ADC_Init(&hadc1);

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* ADC3 init function */
void MX_ADC3_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
	hadc3.Init.Resolution = ADC_RESOLUTION12b;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.Overrun = OVR_DATA_OVERWRITTEN;
	HAL_ADC_Init(&hadc3);

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	HAL_I2C_Init(&hi2c1);

	/**Configure Analogue filter
	 */
	HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	HAL_TIM_Encoder_Init(&htim1, &sConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1 | TIM_CHANNEL_2);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;//tu zmienialem
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

	HAL_TIM_PWM_Init(&htim2);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
	//	MotorStart();

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	HAL_TIM_Encoder_Init(&htim4, &sConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&huart3);
	initUart();
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE();
	__GPIOF_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();

	/*Configure GPIO pins : IR_LED1_OUT_Pin IR_LED2_OUT_Pin IR_LED3_OUT_Pin */
	GPIO_InitStruct.Pin = IR_LED4_OUT_Pin|IR_LED2_OUT_Pin|IR_LED3_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : IR_LED4_OUT_Pin */
	GPIO_InitStruct.Pin = IR_LED1_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(IR_LED1_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : L_MOTOR_OUT1_Pin L_MOTOR_OUT2_Pin */
	GPIO_InitStruct.Pin = L_MOTOR_OUT1_Pin|L_MOTOR_OUT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_L_Pin */
	GPIO_InitStruct.Pin = BUTTON_L_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BUTTON_L_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_R_Pin LED_Y_Pin LED_G_Pin R_MOTOR_OUT1_Pin
                           R_MOTOR_OUT2_Pin */
	GPIO_InitStruct.Pin = LED_R_Pin|LED_Y_Pin|LED_G_Pin|R_MOTOR_OUT1_Pin
			|R_MOTOR_OUT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_R_Pin */
	GPIO_InitStruct.Pin = BUTTON_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BUTTON_R_GPIO_Port, &GPIO_InitStruct);

}

/* StartDefaultTask function */
/*void StartDefaultTask(void const * argument)
{
	//Tu narazie jest regulator PID

	PID.pomVl=0;
	PID.pomVr=0;
	PID.zadVl=0;
	PID.zadVr=0;

	for(;;)
	{
		float prvErrL=PID.zadVl-PID.pomVl;
		float prvErrR=PID.zadVr-PID.pomVr;


		PID.pomVl = GetEncoderL()*TICK2MM/(dT*mnoznik_PID);
		PID.pomVr = GetEncoderR()*TICK2MM/(dT*mnoznik_PID);

		TIM4->CNT=0;//zeruje wartości enkoderów
		TIM1->CNT=0;

		PID.errorL=PID.zadVl-PID.pomVl;
		PID.errorR=PID.zadVr-PID.pomVr;

		PID.Ir+=PID.errorR*dT*mnoznik_PID;
		PID.Il+=PID.errorL*dT*mnoznik_PID;
		PID.Dr=(PID.errorR-prvErrR)/(dT*mnoznik_PID);
		PID.Dl=(PID.errorL-prvErrL)/(dT*mnoznik_PID);

		PID.sterL=PID.P*PID.errorL+PID.I*PID.Il+PID.D*PID.Dl;
		PID.sterR=PID.P*PID.errorR+PID.I*PID.Ir+PID.D*PID.Dr;
		MotorSetPWM2(PID.sterL,PID.sterR);

		osDelay(dT*1000*mnoznik_PID);
	}
}*/

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
	//Tu narazie jest regulator PID

	PID.pomSl=0;
	PID.pomSr=0;
	PID.zadVl=0;
	PID.zadVr=0;
	PID.zadSl=0;
	PID.zadSr=0;

	for(;;)
	{
		float prvErrL=PID.zadSl-PID.pomSl;
		float prvErrR=PID.zadSr-PID.pomSr;

		PID.zadSl+=PID.zadVl*dT*mnoznik_PID;
		PID.zadSr+=PID.zadVr*dT*mnoznik_PID;

		PID.pomSl += GetEncoderL()*TICK2MM;
		PID.pomSr += GetEncoderR()*TICK2MM;

		TIM4->CNT=0;//zeruje wartości enkoderów
		TIM1->CNT=0;

		PID.errorL=PID.zadSl-PID.pomSl;
		PID.errorR=PID.zadSr-PID.pomSr;

		PID.Ir+=PID.errorR*dT*mnoznik_PID;
		PID.Il+=PID.errorL*dT*mnoznik_PID;
		PID.Dr=(PID.errorR-prvErrR)/(dT*mnoznik_PID);
		PID.Dl=(PID.errorL-prvErrL)/(dT*mnoznik_PID);

		PID.sterL=PID.P*PID.errorL+PID.I*PID.Il+PID.D*PID.Dl;
		PID.sterR=PID.P*PID.errorR+PID.I*PID.Ir+PID.D*PID.Dr;
		MotorSetPWM2(PID.sterL,PID.sterR);

		osDelay(dT*1000*mnoznik_PID);
	}
}

const float tresholdFront = 210;
const float tresholdSide = 200;
const float tresholdFront1stWall = 105;


int pX,pY;
char orient, newOrient, tmpOrient;
float act_V;
char best_action, out, speedout, best_action2;
unsigned char speedRun;


/* StartLog function */
void StartLog(void const * argument)
{

	/* USER CODE BEGIN StartLog */
	HAL_ADC_Start(&hadc1);

	/*int measureInd = 0;

	do {
	while(getChar()!='s');
	osDelay(4000);
	measureInd=0;
	while(getChar()!='k' && measureInd < 31)
	{
		osDelay(50);
		measureInd++;
		printI(distMeasure(1));
		osDelay(50);
		print(", ");
		printI(distMeasure(2));
		osDelay(50);
		print(", ");
		printI(distMeasure(3));
		osDelay(50);
		print(", ");
		printI(distMeasure(4));
		print(", ");
		osDelay(150);
		printI(blindMeasure(1));
		osDelay(50);
		print(", ");
		printI(blindMeasure(2));
		osDelay(50);
		print(", ");
		printI(blindMeasure(3));
		osDelay(50);
		print(", ");
		printI(blindMeasure(4));
		print("\n");
		//profiler_perform(10.0,0.0,0.0,'B');
		profiler_perform(7.07,0.0,0.0,'B');
		osDelay(200);
	}

	}while(getChar()!='q');*/

	osDelay(4000);

	/*while(1) {
	profiler_perform(180,0.0,V_max,'F');
	profiler_perform(180,V_max,V_max,'F');
	profiler_perform(180,V_max,0.0,'F');
	osDelay(3000);
	}*/

	/*profiler_perform(100,0.0,V_max_curve,'F');
	profiler_perform(0.0,V_max_curve,V_max_curve,'R');
	profiler_perform(0.0,V_max_curve,V_max_curve,'L');
	profiler_perform(100,V_max_curve,0.0,'F');*/

	int destX=4;
	int destY=4;
	floodfill_init(&FF,0,0,destX,destY);

	speedRun=0;
	pX=0;
	pY=0;
	orient=NORTH;
	act_V=0.0;
	char wall;

	while(HAL_GPIO_ReadPin(BUTTON_R_GPIO_Port,BUTTON_R_Pin)) {
	mid = (opticMeasure[0][numer_pomiaru-1 < 0 ? ILOSC_ANALIZOWANYCH-1 : numer_pomiaru] + opticMeasure[1][numer_pomiaru-1 < 0 ? ILOSC_ANALIZOWANYCH-1 : numer_pomiaru])/2;
	HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
	osDelay(10);
	}

	while(HAL_GPIO_ReadPin(BUTTON_L_GPIO_Port,BUTTON_L_Pin))
	{
		//Szukam urządzeń I2C
		/*uint8_t i;
		for(i=0;i<128;++i)
		{
			uint8_t* data = {0x01};
			if(HAL_I2C_Master_Transmit(&hi2c1,i,data,1,1000)==HAL_OK)
			{
				print("Znaleziono: ");
				printI(i);
				print("\n");
			}

		}*/
		//HAL_Delay(1);
	}
	HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,1);
	osDelay(300);
	setVisited(&FF,0,0);
	if (opticMeasure[2][numer_pomiaru-1 < 0 ? ILOSC_ANALIZOWANYCH-1 : numer_pomiaru] < tresholdFront1stWall) {
		addWall(&FF,0,0,NORTH);
		profiler_perform(-90,0.0,0.0,'T');
		orient=EAST;
	}
	else {
		addWall(&FF,0,0,EAST);
	}
	while (1) {
		if (pX == FF.EndX && pY == FF.EndY) {
			if (FF.EndX==destX && FF.EndY==destY) {
				changeStartEnd(&FF,destX,destY,0,0);
				if (speedRun == 1) {
					actValue(&FF);
					actValue2(&FF);
					bestActionforAll(&FF,pX,pY,orient);
					vTaskResume(Encoder_ReadHandle);
				}
			} else {

				while(HAL_GPIO_ReadPin(BUTTON_L_GPIO_Port,BUTTON_L_Pin));
				HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,1);
				osDelay(1500);
				HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,0);
				changeStartEnd(&FF,0,0,destX,destY);
				speedRun=1;
				V_max = 400;
				a = 300;
				float Stmp=80;
				V_pos = sqrt(2*Stmp*a+V_max_curve*V_max_curve);

				/*FF.Wall[0][0]= 14;
				FF.Wall[0][1]= 8;
				FF.Wall[0][2]= 10;
				FF.Wall[0][3]= 11;
				FF.Wall[0][4]= 14;
				FF.Wall[0][5]= 10;
				FF.Wall[0][6]= 8;
				FF.Wall[0][7]= 11;
				FF.Wall[1][0]= 12;
				FF.Wall[1][1]= 2;
				FF.Wall[1][2]= 10;
				FF.Wall[1][3]= 10;
				FF.Wall[1][4]= 8;
				FF.Wall[1][5]= 9;
				FF.Wall[1][6]= 4;
				FF.Wall[1][7]= 11;
				FF.Wall[2][0]= 7;
				FF.Wall[2][1]= 12;
				FF.Wall[2][2]= 10;
				FF.Wall[2][3]= 10;
				FF.Wall[2][4]= 1;
				FF.Wall[2][5]= 4;
				FF.Wall[2][6]= 2;
				FF.Wall[2][7]= 11;
				FF.Wall[3][0]= 12;
				FF.Wall[3][1]= 1;
				FF.Wall[3][2]= 12;
				FF.Wall[3][3]= 9;
				FF.Wall[3][4]= 7;
				FF.Wall[3][5]= 6;
				FF.Wall[3][6]= 8;
				FF.Wall[3][7]= 9;
				FF.Wall[4][0]= 5;
				FF.Wall[4][1]= 4;
				FF.Wall[4][2]= 0;
				FF.Wall[4][3]= 0;
				FF.Wall[4][4]= 8;
				FF.Wall[4][5]= 9;
				FF.Wall[4][6]= 5;
				FF.Wall[4][7]= 5;
				FF.Wall[5][0]= 4;
				FF.Wall[5][1]= 0;
				FF.Wall[5][2]= 0;
				FF.Wall[5][3]= 0;
				FF.Wall[5][4]= 0;
				FF.Wall[5][5]= 0;
				FF.Wall[5][6]= 2;
				FF.Wall[5][7]= 3;
				FF.Wall[6][0]= 4;
				FF.Wall[6][1]= 0;
				FF.Wall[6][2]= 0;
				FF.Wall[6][3]= 0;
				FF.Wall[6][4]= 0;
				FF.Wall[6][5]= 0;
				FF.Wall[6][6]= 8;
				FF.Wall[6][7]= 9;
				FF.Wall[7][0]= 6;
				FF.Wall[7][1]= 2;
				FF.Wall[7][2]= 2;
				FF.Wall[7][3]= 2;
				FF.Wall[7][4]= 0;
				FF.Wall[7][5]= 2;
				FF.Wall[7][6]= 2;
				FF.Wall[7][7]= 3;
				int ii =0;
				int jj=0;
				for (ii=0;ii<8;ii++) {
					for (jj=0;jj<8;jj++) {
						if (ii != 4 || jj != 6) {
							setVisited(&FF,ii,jj);
						}
					}
				}*/
				actValue2(&FF);
				actValue(&FF);
				bestActionforAll(&FF,pX,pY,orient);
				vTaskResume(Encoder_ReadHandle);
			}
		}
		if (speedRun==0) {
			wall=0;
			out=0;
			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0);
			HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,0);
			HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,0);
			vTaskResume(Encoder_ReadHandle);
			vTaskSuspend(NULL);
			best_action='N';
			out=walls();
			//dodawnie scian i ustawianie ze bylem
			if (orient == NORTH) {
				if (out & 1) {
					wall |= NORTH;
				}
				if (out & 2) {
					wall |= EAST;
				}
				if (out & 4) {
					wall |= WEST;
				}
				addWall(&FF,pX,pY+1,wall);
				setVisited(&FF,pX,pY+1);
				actValue(&FF);
				best_action=bestAction(&FF,pX,pY+1,orient);
				pY=pY+1;
			}
			if (orient == SOUTH) {
				if (out & 1) {
					wall |= SOUTH;
				}
				if (out & 2) {
					wall |= WEST;
				}
				if (out & 4) {
					wall |= EAST;
				}
				addWall(&FF,pX,pY-1,wall);
				setVisited(&FF,pX,pY-1);
				actValue(&FF);
				best_action=bestAction(&FF,pX,pY-1,orient);
				pY=pY-1;
			}
			if (orient == EAST) {
				if (out & 1) {
					wall |= EAST;
				}
				if (out & 2) {
					wall |= SOUTH;
				}
				if (out & 4) {
					wall |= NORTH;
				}
				addWall(&FF,pX+1,pY,wall);
				setVisited(&FF,pX+1,pY);
				actValue(&FF);
				best_action=bestAction(&FF,pX+1,pY,orient);
				pX=pX+1;
			}
			if (orient == WEST) {
				if (out & 1) {
					wall |= WEST;
				}
				if (out & 2) {
					wall |= NORTH;
				}
				if (out & 4) {
					wall |= SOUTH;
				}
				addWall(&FF,pX-1,pY,wall);
				setVisited(&FF,pX-1,pY);
				actValue(&FF);
				best_action=bestAction(&FF,pX-1,pY,orient);
				pX=pX-1;
			}
			if (out & 1) {
				HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,1);
			}
			if (out & 2) {
				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
			}
			if (out & 4) {
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
			}
			if (best_action == 'R') {
				orient=orient<<1;
			}
			if (best_action == 'L') {
				orient=orient<<3;
			}
			if (best_action == 'I' || best_action == 'E') {
				orient=orient<<2;
			}
			if (orient>8) {
				orient=orient>>4;
			}
			vTaskSuspend(NULL);
		} else {//speedrun
			speedout=0;
			best_action='N';
			if (orient == NORTH) {
				pY=pY+1;
				if (FF.Wall[pX][pY] & NORTH) {
					speedout |= 1;
				}
				if (FF.Wall[pX][pY] & EAST) {
					speedout |= 2;
				}
				if (FF.Wall[pX][pY] & WEST) {
					speedout |= 4;
				}
				best_action = FF.Action[pX][pY];
				if (best_action == 'F') {
					best_action2 = FF.Action[pX][pY+1];
				}
			}
			if (orient == SOUTH) {
				pY=pY-1;
				if (FF.Wall[pX][pY] & SOUTH) {
					speedout |= 1;
				}
				if (FF.Wall[pX][pY] & WEST) {
					speedout |= 2;
				}
				if (FF.Wall[pX][pY] & EAST) {
					speedout |= 4;
				}
				best_action = FF.Action[pX][pY];
				if (best_action == 'F') {
					best_action2 = FF.Action[pX][pY-1];
				}
			}
			if (orient == EAST) {
				pX=pX+1;
				if (FF.Wall[pX][pY] & EAST) {
					speedout |= 1;
				}
				if (FF.Wall[pX][pY] & SOUTH) {
					speedout |= 2;
				}
				if (FF.Wall[pX][pY] & NORTH) {
					speedout |= 4;
				}
				best_action = FF.Action[pX][pY];
				if (best_action == 'F') {
					best_action2 = FF.Action[pX+1][pY];
				}
			}
			if (orient == WEST) {
				pX=pX-1;
				if (FF.Wall[pX][pY] & WEST) {
					speedout |= 1;
				}
				if (FF.Wall[pX][pY] & NORTH) {
					speedout |= 2;
				}
				if (FF.Wall[pX][pY] & SOUTH) {
					speedout |= 4;
				}
				best_action = FF.Action[pX][pY];
				if (best_action == 'F') {
					best_action2 = FF.Action[pX-1][pY];
				}
			}
			if (best_action == 'R') {
				orient=orient<<1;
			}
			if (best_action == 'L') {
				orient=orient<<3;
			}
			if (best_action == 'I' || best_action == 'E') {
				orient=orient<<2;
			}
			if (orient>8) {
				orient=orient>>4;
			}
			vTaskSuspend(NULL);
		}
	}
	/* Infinite loop */
	for(;;)
	{
		osDelay(100);
	}
	/* USER CODE END StartLog */
}

/* StartIR_Read function */
void StartIR_Read(void const * argument)
{
	/* USER CODE BEGIN StartIR_Read */
	/* Infinite loop */

	for(;;)
	{
		MeasureIR[0]=linearIR(1,distMeasure(1));
		MeasureIR[1]=linearIR(2,distMeasure(2));
		opticMeasure[0][numer_pomiaru]=MeasureIR[0];
		opticMeasure[1][numer_pomiaru]=MeasureIR[1];
		//int tmp1=distMeasure(3);
		//int tmp2=distMeasure(4);
		opticMeasure[3][numer_pomiaru]=linearIR(4,/*tmp2*/distMeasure(4));
		opticMeasure[2][numer_pomiaru]=linearIR(3,/*tmp1*/distMeasure(3));
		if (numer_pomiaru<ILOSC_ANALIZOWANYCH-1)
			numer_pomiaru++;
		else
			numer_pomiaru=0;
		/*printI(tmp1);
		print(" ");
		printI(tmp2);
		print(" ");
		printF(opticMeasure[2][numer_pomiaru-1>=0?numer_pomiaru-1:30],2);
		print(" ");
		printF(opticMeasure[3][numer_pomiaru-1>=0?numer_pomiaru-1:30],2);
		print("\n");*/
		/*printF(MeasureIR[0],2);
		print(" ");
		printF(MeasureIR[1],2);
		print("\n");*/
		osDelay(dT*1000*4);
		/*float d1 = distMeasure(1);
		printI(linearIR(1,d1));
		print(", ");
		osDelay(100);
		float d2 = distMeasure(2);
		printI(linearIR(2,d2));
		print(", ");
		osDelay(100);
		float d3 = distMeasure(3);
		printI(linearIR(3,d3));
		print(", ");
		osDelay(100);
		float d4 = distMeasure(4);
		printI(linearIR(4,d4));
		print(", ");
		osDelay(100);
		printI(d1);
		print(", ");
		osDelay(100);
		printI(d2);
		print(", ");
		osDelay(100);
		printI(d3);
		print(", ");
		osDelay(100);
		printI(d4);
		print("\n");
		osDelay(250);*/
	}
	/* USER CODE END StartIR_Read */
}

/* StartEncoder_Read function */
//task zawiera w sobie część planowania ruchu - w momencie gdy planer miał
//złą informację o bocznej ścianie i wyznaczył skręt na nią, ten task to sprawdza
//i na nowo przelicza ruch (tylko dla eksploracji)
//ograniczenia - nie zadziała jeśli planer miał zle informację o OBU ścianach bocznych,
//a ten task uznał że należy skręcić w drugą stronę
void StartEncoder_Read(void const * argument)
{
	/* USER CODE BEGIN StartEncoder_Read */
	char bef_act;
	char BA, BA2;

	vTaskSuspend(NULL);
	/* Infinite loop */
	for(;;)
	{
		if (speedRun == 0) {

			profiler_perform(60,act_V,V_max,'F');
			vTaskResume(LogHandle);
			act_V=V_max;
			profiler_perform(60,V_max,V_max,'F');
			BA = best_action;
			if (BA == 'F') {
				profiler_perform(60,V_max,V_max,'F');
			} else if (BA == 'R') {
				profiler_perform(60,V_max,0.0,'F');
				osDelay(50);
				profiler_perform(-90,0.0,0.0,'T');
				act_V=0.0;
				osDelay(500);
				if (opticMeasure[2][numer_pomiaru-1 < 0 ? ILOSC_ANALIZOWANYCH-1 : numer_pomiaru] <
						tresholdFront1stWall &&
						opticMeasure[3][numer_pomiaru-1 < 0 ? ILOSC_ANALIZOWANYCH-1 : numer_pomiaru] <
						tresholdFront1stWall) {
					addWall(&FF,pX,pY,orient);
					actValue(&FF);
					BA = bestAction(&FF,pX,pY,orient);
					if (BA == 'L') {
						osDelay(100);
						profiler_perform(90,0.0,0.0,'T');
						orient = orient << 4;
						osDelay(100);
					} else if (BA == 'R') {
						osDelay(100);
						profiler_perform(-90,0.0,0.0,'T');
						orient = orient << 1;
						osDelay(100);
					} else if (BA == 'I') {
						osDelay(100);
						profiler_perform(-180,0.0,0.0,'T');
						orient = orient << 2;
						osDelay(100);
					}
					if (orient > 8) {
						orient = orient >> 4;
					}
				}
			} else if (BA == 'L') {
				profiler_perform(60,V_max,0.0,'F');
				osDelay(50);
				profiler_perform(90,0.0,0.0,'T');
				act_V=0.0;
				osDelay(500);
				if (opticMeasure[2][numer_pomiaru-1 < 0 ? ILOSC_ANALIZOWANYCH-1 : numer_pomiaru] <
						tresholdFront1stWall &&
						opticMeasure[3][numer_pomiaru-1 < 0 ? ILOSC_ANALIZOWANYCH-1 : numer_pomiaru] <
						tresholdFront1stWall) {
					addWall(&FF,pX,pY,orient);
					actValue(&FF);
					BA = bestAction(&FF,pX,pY,orient);
					if (BA == 'L') {
						osDelay(100);
						profiler_perform(90,0.0,0.0,'T');
						orient = orient << 4;
						osDelay(100);
					} else if (BA == 'R') {
						osDelay(100);
						profiler_perform(-90,0.0,0.0,'T');
						orient = orient << 1;
						osDelay(100);
					} else if (BA == 'I') {
						osDelay(100);
						profiler_perform(-180,0.0,0.0,'T');
						orient = orient << 2;
						osDelay(100);
					}
					if (orient > 8) {
						orient = orient >> 4;
					}
				}
			} else if (BA == 'I' || BA == 'E') {
				profiler_perform(60,V_max,0.0,'F');
				if (BA == 'E') {
					bef_act = 'E';
				}
				osDelay(50);
				profiler_perform(-180,0.0,0.0,'T');
				act_V=0.0;
				osDelay(50);
			} else if (BA == 'N') {
				profiler_perform(60,V_max,0.0,'F');
				act_V=0.0;
				break;
			}
			vTaskResume(LogHandle);
			vTaskSuspend(NULL);
		} else {//speedrun
			vTaskResume(LogHandle);
			if (bef_act == 'F' || bef_act == 'I' || bef_act == 'E') {
				//profiler_perform(45,act_V,V_max,'F');
				if (act_V == 0.0) {
					profiler_perform(10,act_V,sqrt(10.0*a),'F');
					act_V = sqrt(10.0*a);
				} else {
					profiler_perform(10,act_V,act_V,'F');
				}

				BA = best_action;
				if (BA == 'F') {
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,1);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,0);
					BA2 = best_action2;
					if (BA2 == 'F') {
						profiler_perform(170,act_V,V_max,'F');
						act_V=V_max;
					} else {
						profiler_perform(170,act_V,V_pos,'F');
						act_V=V_pos;
					}
					//profiler_perform(135,V_max,V_max,'F');
				} else if (BA == 'R') {
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,0);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
					//profiler_perform(45,V_max,V_max_curve,'F');
					//profiler_perform(141,V_max_curve,V_max_curve,'R');

					//profiler_perform(55,V_max,V_max_curve,'F');

					profiler_perform(70,act_V,V_max_curve,'F');
					profiler_perform(126,V_max_curve,V_max_curve,'R');
					profiler_perform(10,V_max_curve,V_max_curve,'F');

					act_V=V_max_curve;
				} else if (BA == 'L') {
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,0);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,0);
					//profiler_perform(45,V_max,V_max_curve,'F');
					//profiler_perform(141,V_max_curve,V_max_curve,'L');

					//profiler_perform(55,V_max,V_max_curve,'F');

					profiler_perform(70,act_V,V_max_curve,'F');
					profiler_perform(126,V_max_curve,V_max_curve,'L');
					profiler_perform(10,V_max_curve,V_max_curve,'F');

					act_V=V_max_curve;
				} else if (BA == 'I' || BA == 'E') {
					//profiler_perform(135,act_V,0.0,'F');
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,0);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
					profiler_perform(170,act_V,0.0,'F');
					osDelay(100);
					profiler_perform(-180,0.0,0.0,'T');
					act_V=0.0;
					osDelay(100);
					if (BA == 'E' && pX == 0 && pY == 0) {
						vTaskResume(LogHandle);
						vTaskSuspend(NULL);
					}
				} else if (BA == 'N') {
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,1);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
					//profiler_perform(135,act_V,0.0,'F');

					profiler_perform(170,act_V,0.0,'F');
					act_V=0.0;
					break;
				}

			} else {//obroty
				profiler_perform(10,act_V,act_V,'F');
				BA = best_action;
				if (BA == 'F') {
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,1);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,0);
					BA2 = best_action2;
					if (BA2 == 'F') {
						profiler_perform(80,act_V,V_max,'F');
						act_V=V_max;
					} else {
						profiler_perform(80,act_V,V_pos,'F');
						act_V=V_pos;
					}
					//profiler_perform(90,act_V,V_max,'F');
				} else if (BA == 'R') {
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,0);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
					//profiler_perform(141,V_max_curve,V_max_curve,'R');

					profiler_perform(126,V_max_curve,V_max_curve,'R');
					profiler_perform(10,V_max_curve,V_max_curve,'F');

					act_V=V_max_curve;
				} else if (BA == 'L') {
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,0);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,0);
					//profiler_perform(141,V_max_curve,V_max_curve,'L');

					profiler_perform(126,V_max_curve,V_max_curve,'L');
					profiler_perform(10,V_max_curve,V_max_curve,'F');

					act_V=V_max_curve;
				} else if (BA == 'I' || BA == 'E') {
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,0);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
					//profiler_perform(90,act_V,0.0,'F');

					profiler_perform(80,act_V,0.0,'F');

					osDelay(100);
					profiler_perform(-180,0.0,0.0,'T');
					act_V=0.0;
					osDelay(100);
					if (BA == 'E' && pX == 0 && pY == 0) {
						vTaskResume(LogHandle);
						vTaskSuspend(NULL);
					}
				} else if (BA == 'N') {
					HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
					HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,1);
					HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
					//profiler_perform(90,act_V,0.0,'F');

					profiler_perform(80,act_V,0.0,'F');

					act_V=0.0;
					break;
				}
			}
			bef_act = BA;
			/*vTaskResume(LogHandle);
			vTaskSuspend(NULL);*/


			/*profiler_perform(60,act_V,V_max,'F');
			vTaskResume(LogHandle);
			act_V=V_max;
			profiler_perform(60,V_max,V_max,'F');
			BA = best_action;
			if (BA == 'F') {
				profiler_perform(60,V_max,V_max,'F');
			} else if (BA == 'R') {
				profiler_perform(60,V_max,0.0,'F');
				osDelay(100);
				profiler_perform(-90,0.0,0.0,'T');
				act_V=0.0;
				osDelay(100);
			} else if (BA == 'L') {
				profiler_perform(60,V_max,0.0,'F');
				osDelay(100);
				profiler_perform(90,0.0,0.0,'T');
				act_V=0.0;
				osDelay(100);
			} else if (BA == 'I' || BA == 'E') {
				profiler_perform(60,V_max,0.0,'F');
				osDelay(100);
				profiler_perform(-180,0.0,0.0,'T');
				act_V=0.0;
				osDelay(100);
			} else if (BA == 'N') {
				profiler_perform(60,V_max,0.0,'F');
				act_V=0.0;
				break;
			}
			vTaskResume(LogHandle);
			vTaskSuspend(NULL);*/
		}
	}
	while(1) {
		SetSpeed(0.0,0.0);
		osDelay(500);
		HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,0);
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
		HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
		osDelay(500);
		HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,1);
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1);
		HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0);
		osDelay(500);
		HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,1);
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,0);
		HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);

	}
	/* USER CODE END StartEncoder_Read */
}

/* StartIMU_Read function */
void StartIMU_Read(void const * argument)
{
	/* USER CODE BEGIN StartIMU_Read */
	/* Infinite loop */
	for(;;)
	{
		osDelay(100);
	}
	/* USER CODE END StartIMU_Read */
}

/* StartKalman function */
void StartKalman(void const * argument)
{
	/* USER CODE BEGIN StartKalman */
	/* Infinite loop */
	for(;;)
	{
		osDelay(100);
	}
	/* USER CODE END StartKalman */
}

/* StartGet_Voltage function */
void StartGet_Voltage(void const * argument)
{
	/* USER CODE BEGIN StartGet_Voltage */
	/* Infinite loop */
	for(;;)
	{
		/*HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3,1000);
		float Napiecie = (float)HAL_ADC_GetValue(&hadc3)*0.003087143; //Napiecie w V
		if(Napiecie<6.4)
		{
			while(1)
			{
				HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
				osDelay(200);
			}
		}*/
		osDelay(5000);

		/* USER CODE END StartGet_Voltage */
	}
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
