

#include <stdint.h>
#include "main.h"

// HMC5883l - I2C ADDRESS
#define HMC5883l_ADDRESS (0x1E << 1)

// MEMORY MAPPING
/*
Address Location	Name 							Access
---------------------------------------------------
0x00				Configuration Register A		Read/Write
0x01				Configuration Register B		Read/Write
0x02				Mode Register					Read/Write
0x03				Data Output X MSB Register		Read
0x04				Data Output X LSB Register		Read
0x05				Data Output Z MSB Register		Read
0x06				Data Output Z LSB Register		Read
0x07				Data Output Y MSB Register		Read
0x08				Data Output Y LSB Register		Read
0x09				Status Register					Read
0x0A				Identification Register A		Read
0x0B				Identification Register B		Read
0x0C				Identification Register C		Read
0x31				Temperature Output MSB Register	Read
0x32				Temperature Output LSB Register Read
*/

#define HMC5983_CONF_A		0x00
#define HMC5983_CONF_B		0x01
#define HMC5983_MODE		0x02

#define HMC5983_OUT_X_MSB	0x03
#define HMC5983_OUT_X_LSB	0x04
#define HMC5983_OUT_Z_MSB	0x05
#define HMC5983_OUT_Z_LSB	0x06
#define HMC5983_OUT_Y_MSB	0x07
#define HMC5983_OUT_Y_LSB	0x08

#define HMC5983_STATUS		0x09
#define HMC5983_ID_A		0x0A
#define HMC5983_ID_B		0x0B
#define HMC5983_ID_C		0x0C
#define HMC5983_TEMP_OUT_MSB	0x31
#define HMC5983_TEMP_OUT_LSB	0x32

#define HMC5983_REG_A_Value 0x78 //8Avarage, 15Hz, Normal Measurement
#define HMC5983_REG_B_Value 0xA0 //Gain=5
#define HMC5983_MODE_REG_Value 0x00 //Continuous measurement mode

/*
 * Register A Configuration
 *
 * Register A Bit Designation
 *
 * MSB----------------------LSB
 * Set Temperature + Sample Average + OutputRate + Measurement Mode
 *
 * RegisterA =(Set_Temperature_Sensor<<7)|(Set_Sample_Average<<5)|(Set_OutputRate<<2)|(Set_MeasurementMode)
 *
 */

#define SetTemperatureBitLength 1
#define Set_Temperature_Sensor_On 0x01 //Temperature Sensor On
#define Set_Temperature_Sensor_Off 0x00 //Temperature Sensor Off

#define SampleAvarageBitLength 2
#define Sample_Avarage_1 0x00; //1 Average
#define Sample_Avarage_2 0x01; //2 Average
#define Sample_Avarage_4 0x10; //4 Average
#define Sample_Avarage_8 0x11; //8 Average

#define OutputBitLength 3
#define OutpuRate_P75 0x00 //0.75Hz
#define OutpuRate_1P5 0x01 //1.5Hz
#define OutpuRate_3 0x02 //3Hz
#define OutpuRate_7P5 0x03 //7.5Hz
#define OutpuRate_15 0x04 //15Hz(Default)
#define OutpuRate_30 0x05 //30Hz
#define OutpuRate_75 0x06 //75Hz
#define OutpuRate_220 0x07 //220Hz

#define MeasurementModeBitLength 2
#define Measurement_Mode_Normal 0x00 //Normal Measurement Mode
#define Measurement_Mode_PositiveBias 0x01 //Positive Bias Measurement Mode
#define Measurement_Mode_NegativeBias 0x02 //Negative Bias Measurement Mode
#define Measurement_Mode_TemperatureOnly 0x03 //Temperature sensor only


typedef struct
{
	uint8_t Set_Temperature_Sensor;
	uint8_t Set_Sample_Average;
	uint8_t Set_OutputRate;
	uint8_t Set_Measurement_Mode;
}Register_A_;

/*
 * Register B Configuration
 *
 *RegisterB =(Set_Gain<<5)
 */

#define Set_Gain_0P88 0x00 //+-0.88 Gauss
#define Set_Gain_1P3 0x01 //+-1.3 Gauss
#define Set_Gain_1P9 0x02 //+-1.9 Gauss
#define Set_Gain_2P5 0x03 //+-2.5 Gauss
#define Set_Gain_4P0 0x04 //+-4.0 Gauss
#define Set_Gain_4P7 0x05 //+-4.7 Gauss
#define Set_Gain_5P6 0x06 //+-5.6 Gauss
#define Set_Gain_8P1 0x07//+-8.1 Gauss

//Scale value for selected gain
#define Scale_0P88 0.73
#define Scale_1P3 0.92
#define Scale_1P9 1.22
#define Scale_2P5 1.52
#define Scale_4P0 2.27
#define Scale_4P7 2.56
#define Scale_5P6 3.03
#define Scale_8P1 4.35

typedef struct
{
	uint8_t Set_Gain;
	uint8_t Set_Scale;
}Register_B_;


/*
 * Mode Register Configuration
 *
 *
 * ModeRegister= (Set_I2C_HighSpeed<<7)|(Set_Lowest_Power_Mod<<5)|(SPI_Mode_Selection<<2)|(Operating_Mode)
 */

#define Set_I2C_HighSpeed_On 0x01
#define Set_I2C_HighSpeed_Off 0x00

#define Set_Lowest_Power_Mod_On 0x01
#define Set_Lowest_Power_Mod_Off 0x00

#define SPI_Mode_Selection_4Wire 0x00
#define SPI_Mode_Selection_3Wire 0x01

#define Operating_Mode_Continuous_Measurement 0x00
#define Operating_Mode_Single_Measurement 0x01
#define Operating_Mode_IdleMode 0x02

typedef struct
{
	uint8_t Set_I2C_HighSpeed;
	uint8_t Set_Lowest_Power_Mod;
	uint8_t Set_SPI_Mode_Selection;
	uint8_t Set_Operating_Mode;
}Register_Mode_;





uint8_t Set_Register_A(uint8_t Set_Temperature_Sensor, uint8_t Set_Sample_Average, uint8_t Set_OutputRate, uint8_t Set_MeasurementMode);
uint8_t Set_Register_B(uint8_t Set_Gain);
uint8_t Set_Mode_Register(uint8_t Set_I2C_HighSpeed, uint8_t Set_Lowest_Power_Mod, uint8_t SPI_Mode_Selection, uint8_t Operating_Mode);
uint8_t HMC5983_Init(SPI_HandleTypeDef *spi,uint8_t Register_A_Address, uint8_t Register_A_Value, uint8_t Register_B_Address, uint8_t Register_B_Value, uint8_t Mode_Register_Address, uint8_t Mode_Register_Value);
uint8_t HMC5983_Init(SPI_HandleTypeDef *spi, uint8_t Register_A_Address, uint8_t Register_A_Value, uint8_t Register_B_Address, uint8_t Register_B_Value, uint8_t Mode_Register_Address, uint8_t Mode_Register_Value);

