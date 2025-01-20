/*
 * HMC5983.c
 *
 *  Created on: Jan 17, 2025
 *      Author: kaan.sancakdar
 */

#include "HMC5983.h"
#include "main.h"

uint8_t Set_Register_A(uint8_t Set_Temperature_Sensor, uint8_t Set_Sample_Average, uint8_t Set_OutputRate, uint8_t Set_MeasurementMode)
{
	uint8_t data;
	data = (Set_Temperature_Sensor << 7) | (Set_Sample_Average << 5) | (Set_OutputRate << 2) | (Set_MeasurementMode);
	return data;
}

uint8_t Set_Register_B(uint8_t Set_Gain)
{
	uint8_t data;
	data = (Set_Gain << 5);
	return data;
}

uint8_t Set_Mode_Register(uint8_t Set_I2C_HighSpeed, uint8_t Set_Lowest_Power_Mod, uint8_t SPI_Mode_Selection, uint8_t Operating_Mode)
{
	uint8_t data;
	data = (Set_I2C_HighSpeed << 7) | (Set_Lowest_Power_Mod << 5) | (SPI_Mode_Selection << 2) | (Operating_Mode);
	return data;
}

uint8_t HMC5983_Init(SPI_HandleTypeDef *spi, uint8_t Register_A_Address, uint8_t Register_A_Value, uint8_t Register_B_Address, uint8_t Register_B_Value, uint8_t Mode_Register_Address, uint8_t Mode_Register_Value)
{

	uint8_t Init_OK;
	uint8_t Register_A_OK = 0;
	uint8_t Register_B_OK = 0;
	uint8_t Register_Mode_OK = 0;

	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, &Register_A_Address, 1, 100);
	HAL_SPI_Transmit(spi, &Register_A_Value, 1, 100);
	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, &Register_B_Address, 1, 100);
	HAL_SPI_Transmit(spi, &Register_B_Value, 1, 100);
	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, &Mode_Register_Address, 1, 100);
	HAL_SPI_Transmit(spi, &Mode_Register_Value, 1, 100);
	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_SET);

	uint8_t data = 0;
	uint8_t ReadReg = 0x80; //Read Register A
	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, &ReadReg, 1, 100);
	HAL_SPI_Receive(spi, &data, 1, 100);
	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_SET);

	if (data == Register_A_Value)
	{
		Register_A_OK = 0x01;
	}
	else
	{
		Register_A_OK = 0x00;
	}

	ReadReg = 0x81; //Read Register B
	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, &ReadReg, 1, 100);
	HAL_SPI_Receive(spi, &data, 1, 100);
	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_SET);

	if (data == Register_B_Value)
	{
		Register_B_OK = 0x01;
	}
	else
	{
		Register_B_OK = 0x00;
	}

	ReadReg = 0x82; //Read Mode Register
	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, &ReadReg, 1, 100);
	HAL_SPI_Receive(spi, &data, 1, 100);
	HAL_GPIO_WritePin(HMC5983_CS_GPIO_Port, HMC5983_CS_Pin, GPIO_PIN_SET);

	if (data == Mode_Register_Value)
	{
		Register_Mode_OK = 0x01;
	}
	else
	{
		Register_Mode_OK = 0x00;
	}

	Init_OK = (Register_A_OK << 2) | (Register_B_OK << 1) || Register_Mode_OK;
	return Init_OK;
}
