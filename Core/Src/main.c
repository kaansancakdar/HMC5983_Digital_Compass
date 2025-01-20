/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "HMC5983.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// variables for receiving DATA

// adresses
// HMC5883l - ADDRESS
#define HMC5883l_ADDRESS (0x1E << 1)
// CONTROL REG A
#define HMC5883l_Enable_A (0x78)
// CONTROL REG B
#define HMC5883l_Enable_B (0xA0)
// MODE REGISTER
#define HMC5883l_MR (0x00)
// HMC5883l - MSB / LSB ADDRESSES
#define HMC5883l_ADD_DATAX_MSB (0x03)
#define HMC5883l_ADD_DATAX_LSB (0x04)
#define HMC5883l_ADD_DATAZ_MSB (0x05)
#define HMC5883l_ADD_DATAZ_LSB (0x06)
#define HMC5883l_ADD_DATAY_MSB (0x07)
#define HMC5883l_ADD_DATAY_LSB (0x08)
// SUM (MSB + LSB) DEFINE
#define HMC5883l_ADD_DATAX_MSB_MULTI (HMC5883l_ADD_DATAX_MSB | 0x80)
#define HMC5883l_ADD_DATAY_MSB_MULTI (HMC5883l_ADD_DATAY_MSB | 0x80)
#define HMC5883l_ADD_DATAZ_MSB_MULTI (HMC5883l_ADD_DATAZ_MSB | 0x80)

static Register_A_ Register_A;
static Register_B_ Register_B;
static Register_Mode_ Register_Mode;

uint8_t Register_A_Value=0;
uint8_t Register_B_Value=0;
uint8_t Register_Mode_Value=0;

uint8_t HMC5983_Init_OK=0;
uint8_t Read_Data_Register=0xC3;

uint8_t buffer[6];
int16_t Raw_X;
int16_t Raw_Y;
int16_t Raw_Z;

float mG_X;
float mG_Y;
float mG_Z;

float uT_X;
float uT_Y;
float uT_Z;

int16_t raw_mG_X;
int16_t raw_mG_Y;
int16_t raw_mG_Z;

char sendData[100];

float heading;
float heading_cal;
float magnetic_declination=-9.76;

uint8_t a;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */


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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Register_A.Set_Temperature_Sensor=Set_Temperature_Sensor_Off;
  Register_A.Set_Sample_Average=Sample_Avarage_8;
  Register_A.Set_OutputRate=OutpuRate_15;
  Register_A.Set_Measurement_Mode=Measurement_Mode_Normal;

  Register_B.Set_Gain=Set_Gain_4P7;
  Register_B.Set_Scale=Scale_4P7;

  Register_Mode.Set_I2C_HighSpeed=Set_I2C_HighSpeed_Off;
  Register_Mode.Set_Lowest_Power_Mod=Set_Lowest_Power_Mod_Off;
  Register_Mode.Set_SPI_Mode_Selection=SPI_Mode_Selection_4Wire;
  Register_Mode.Set_Operating_Mode=Operating_Mode_Continuous_Measurement;

  Register_A_Value=Set_Register_A(Register_A.Set_Temperature_Sensor, Register_A.Set_Sample_Average, Register_A.Set_OutputRate, Register_A.Set_Measurement_Mode);
  Register_B_Value=Set_Register_B(Register_B.Set_Gain);
  Register_Mode_Value=Set_Mode_Register(Register_Mode.Set_I2C_HighSpeed, Register_Mode.Set_Lowest_Power_Mod, Register_Mode.Set_SPI_Mode_Selection, Register_Mode.Set_Operating_Mode);

  HMC5983_Init_OK=HMC5983_Init(&hspi1, HMC5983_CONF_A, Register_A_Value, HMC5983_CONF_B, Register_B_Value, HMC5983_MODE, Register_Mode_Value);


  //Hard Iron Calibration Settings
  const float hard_iron[3]={3.43,-6.55,5.19};

  //Soft Iron Calibration Settings
  const float soft_ireon[3][3]={
		  {0.984,0.029,0.025},
		  {0.029,0.997,0.003},
		  {0.025,0.003,1.020}
  };

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  static float hi_cal[3];
	  float mag_data[]={uT_X,uT_Y,uT_Z

	  };
    /* USER CODE BEGIN 3 */

	  HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &Read_Data_Register, 1, 100);
	  HAL_SPI_Receive(&hspi1, buffer, 6, 100);
	  HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_SET);
	  Raw_X=((buffer[0]<<8)|buffer[1]); //X
	  Raw_Z=((buffer[2]<<8)|buffer[3]); //Z
	  Raw_Y=((buffer[4]<<8)|buffer[5]); //Y

	  mG_X=Raw_X*Register_B.Set_Scale; //mGauss Value
	  mG_Y=Raw_Y*Register_B.Set_Scale; //mGauss Value
	  mG_Z=Raw_Z*Register_B.Set_Scale; //mGauss Value

	  //For motionCal Program
	  raw_mG_X=mG_X; //int type of mG value
	  raw_mG_Y=mG_Y; //int type of mG value
	  raw_mG_Z=mG_Z; //int type of mG value


	  uT_X=mG_X/10.00;
	  uT_Y=mG_Y/10.00;
	  uT_Z=mG_Z/10.00;

	  //sprintf(sendData,"Raw:0,0,0,0,0,0,%d,%d,%d\r\n",raw_mG_X,raw_mG_Y,raw_mG_Z);
	  sprintf(sendData,"%.2f,%.2f,%.2f\r\n",uT_X,uT_Y,uT_Z);
	  HAL_UART_Transmit(&huart1, (uint8_t *)sendData, strlen(sendData), 1000);
//	  heading=atan2(uT_Y,uT_X);
//
//	  if(heading<0)
//	  {
//		  heading+=2*M_PI;
//	  }
//	  heading=heading*180/M_PI;
//
//	  //Apply Hard iron offsets
//	  for(int i=0; i<3; i++)
//	  {
//		  hi_cal[i]=mag_data[i]-hard_iron[i];
//	  }
////
////	  //Apply soft iron scaling
//	  for(int i=0; i<3;i++)
//	  {
//		  mag_data[i]=(soft_ireon[i][0]*hi_cal[0])+(soft_ireon[i][1]*hi_cal[1])+(soft_ireon[i][2]*hi_cal[2]);
//	  }
//
//	  heading_cal=atan2(mag_data[1],mag_data[0]);
//
//	  	  if(heading_cal<0)
//	  	  {
//	  		heading_cal+=2*M_PI;
//	  	  }
//	  	heading_cal=(heading_cal*180/M_PI)+magnetic_declination;
	  HAL_Delay(10);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HMC5983_CS_Pin */
  GPIO_InitStruct.Pin = HMC5983_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HMC5983_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
