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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd16x2_i2c.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * BANDERAS
 */
volatile int button_cal=0; //Bandera para detectar que P0 ha sido pulsado
volatile int button_cal_min=0; //Bandera para indicar que queremocalibrar ese valor como minimo
volatile int boton_cal_max = 0; //Bandera para indicar que queremocalibrar ese valor como minimo
volatile int boton_mood = 0; //Si es 0->modo normal, 1 modo automatico, 2 modo manual (potenciometro)

/*
 * ENTRADAS
 */
int sensor=0; //Variable que guarda el valor del LDR
int potenciometro=0; //Variable que guarda el valor del Potenciometro
uint32_t adc_value[2]; //Vector que almacena los datod por DMA del ADC
/*
 * PARAMETROS DE MOOD_AUTO
 */
int velocidad=500; //ms entre interrupciones del temporizador
float b=40; //Variable para red rojo RGB
float r=60; //Variable para red rojo RGB, tiene que ir desfasado con el azul
int auxr=0; //Variable que vale 1 cuando tiene que sumar y 0 cuando tiene que restar
int auxb=0; //Variable que vale 1 cuando tiene que sumar y 0 cuando tiene que restar
float incremento_azul=2; //Incremento del valor en cada interrupcion para el PWM del RGB azul
float incremento_rojo=1; //Incremento del valor en cada interrupcion para el PWM del RGB rojo

/*
 * PARAMETROS PWM
 */
volatile int Enviar_Led=0; //Valor que se le pasa por PWM para la intensidad del LED normal
volatile int Max_LDR = 4095;//Por encima de este valor no se encenderá el LED, antes estaba a 255
volatile int Min_LDR = 30; //Por debajo de este valor no se encenderá más el LED
int desactivar=0; //Desactiva el LED de iluminacion normal

/*
 * VARIABLES POR DEFINIR
 */

//INTERRUPCIONES PULSADORES
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_PIN_0){
		//poner cto antirrebotes...
		button_cal = 1; // Pongo una bandera para hacer el codigo en el main y así no bloquear
	}
	else if(GPIO_Pin == GPIO_PIN_1){
		//antiguamente no ponia button_cal, ponia p, que me lo devolvia la funcion antirrebotes
		//if(button_cal==1)//Condicion para que no pueda pulsarse la calibracion sin haberse indicado antes que se va a calibrar
		boton_cal_max = 1;
		button_cal=0;
	}
	else if(GPIO_Pin == GPIO_PIN_2){
		//if(button_cal==1)//Condicion para que no pueda pulsarse la calibracion sin haberse indicado antes que se va a calibrar
		button_cal_min = 1;
		button_cal=0;
	}
	else if(GPIO_Pin == GPIO_PIN_4){
		//hay rebotes, llamar al debouncer
		if(boton_mood==0)
		boton_mood=1;
		else if (boton_mood==1)boton_mood=2;
		else boton_mood=0;
		// Añadir un 4 modo o algo para calibrar

		}

	}

//FUNCION PARA ASIGNAR los valores del DMA a variables
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	sensor=adc_value[0];
	potenciometro=adc_value[1];
}


//INTERRUPCIONES TEMPORIZADOR
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	UNUSED(htim);
	Mood_Auto();
}

//ALGORITMO LUCES MODO AUTO
void Mood_Auto(){

	//RGB AZUL
	if(b<40) auxb=1;
	else if(b>100)auxb=0;
	if (auxb==1)b=b+incremento_azul;
	else b=b-incremento_azul;

	//RGB ROJO
	if(r<25) auxr=1;
	else if(r>55)auxr=0;
	if (auxr==1)r=r+incremento_rojo;
	else r=r-incremento_rojo;
}

//ALGORITMO PWM LED
int algoritmo(int val){

	if((val<=Max_LDR) && (val>=Min_LDR)){ //Por encima del maximo no quiero que se encienda a luz
	return 100-(val*100)/Max_LDR;
	}
	else if(val<Min_LDR){
		return 100-(Min_LDR*100)/Max_LDR;
	}
	else {
		return 0;
	}
}

//CALIBRAR MAXIMO DEL LDR
void CalibrarMax(int val){
	/*
	 * HAY QUE COMPROBAR QUE EL MAXIMO NO SEA MENOR QUE MINIMO
	 * LA VARIABLE QUE RECIBE ES EL VALOR DEL POTENCIOMETRO O DEL LDR
	 */
	if(val>Min_LDR){
	Max_LDR = val;
	}
}

//CALIBRAR MINIMO
void CalibrarMin(int val){
	/*
	 * HAY QUE COMPROBAR QUE EL MINIMO NO SEA MAYOR QUE EL MAXIMO
	 */
	if(val<Max_LDR)
	Min_LDR = val;
}

void cambio(int new_val, int mod){//Devuelve el porcentaje de luz que emite el LED respecto su total
	static int val=0; //buffer para almacenar los valores anteriores que se estaba enviando al LED
	static int modo=0; //buffer para almacenar el modo de funcionamiento previo

	//CAMBIO DE MODO
	if(modo!=mod){
		lcd16x2_i2c_setCursor(0,0);
		lcd16x2_i2c_printf("Modo: %d", mod);
		modo=mod;
	}

	//CAMBIO DE INTENSIDAD
	if((new_val-val)<4 || (new_val-val)>4){
		lcd16x2_i2c_setCursor(2,0);
		if(mod==1)lcd16x2_i2c_printf("Intensidad: 00 ");
		else lcd16x2_i2c_printf("Intensidad: %d", new_val);
		val=new_val;
	}
}


// ANTIRREBOTES PROFESOR
int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number){
 static uint8_t button_count=0;
 static int counter=0;

 if (*button_int==1){
 counter=HAL_GetTick();
 while (button_count<4){
 /*if (button_count==0) {
 counter=HAL_GetTick();
 button_count++;
 }*/
 if (HAL_GetTick()-counter>=20){
 counter=HAL_GetTick();
 if (HAL_GPIO_ReadPin(GPIO_port, GPIO_number)!=1){
 button_count=1;
 }
 else{
 button_count++;
 }
 if (button_count==4){ //Periodo antirebotes
 button_count=0;
 *button_int=0;
 return 1;
 }
 }
 }
 }
 return 0;
}

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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  if(lcd16x2_i2c_init(&hi2c1)){
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
  }

  	  /*
  	   * INICIAMOS TEMPORIZADORES Y EL CONVERTIDOR
   	  */
  	  HAL_TIM_Base_Start_IT(&htim12); //Activo el temporizador para el mood auto con interrupcion
  	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  	  HAL_ADC_Start_DMA(&hadc1, adc_value, 2); //Activamos el ADC por DMA

  	  lcd16x2_i2c_setCursor(0, 0);
  	  lcd16x2_i2c_clear();
	  lcd16x2_i2c_printf("Modo: 0");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  		  	  /*
	  	 	  	  * LDR Y LED NORMAL
	  		  	  */
	//En modos 0 y 2 se desactiva el temporizador del RGB
	//En cualquier modo se establece su valor correspondiente al LED principal
	//Este LED principal se llama al salir de la comprobacion del modo actual
	if(boton_mood==0){ //Si está el modo normal, desactivo el modo automatico
	  	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	  	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	  	desactivar=0;
	  	Enviar_Led = algoritmo(sensor);
	}
	else if(boton_mood==2){ //Si está el modo normal, desactivo el modo automatico
	  	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	  	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	  	desactivar=0;
	  	Enviar_Led = algoritmo(potenciometro);
	}
	else{ //Activo el modo automatico, el normal lo desactivo luego en funcion de "desactivar"
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	    desactivar=1;
	    Enviar_Led=0;
	}
	//Dependiendo del modo en el que estemos mandaremos un valor u otro por PWM al LED principal
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Enviar_Led);

					/*
					 * Mostrar por pantalla
					 */
	cambio(Enviar_Led, boton_mood);

					/*
					 * CALIBRACION DE LAS LECTURAS DEL LDR
					 */

  	//Habilito y deshabilito el resto de inteerrupciones, en lugar de poner banderas
	if(button_cal==0){
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	}
	else{
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	}

	//Si las interrupciones han sido habilitadas, habremos indicado que queremos calibrar con el valor actual
	//Se puede calibrar en el modo 0 y 2, y una vez calibrado estos valores sirven para ambos, excepto el 1 que no usa ese LED
		  if (boton_cal_max==1){
			if(boton_mood==0)CalibrarMax(sensor);
			else if(boton_mood==2)CalibrarMax(potenciometro);
			boton_cal_max=0;
			button_cal=0;
		}
		else if(button_cal_min==1){
			if(boton_mood==0)CalibrarMin(sensor);
			if(boton_mood==2)CalibrarMin(potenciometro);
			button_cal_min=0;
			button_cal=0;
		}

							/*
							 * MODO AUTOMATICO RGB (MOOD 1)
							 */

	//lOS VALORES b Y r, LOS CALCULO EL MOOD_AUTO DESPUES DE LA INTERRUPCION DEL TEMPORIZADOR
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, b);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, r);



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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 16000;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = velocidad;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
