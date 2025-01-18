


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
 * RegisterA =
 *
 */

#define SetTemperatureBitLength 1
#define Set_Temperature_Sensor_On 0x01 //Temperature Sensor On
#define Set_Temperature_Sensor_On 0x00 //Temperature Sensor Off

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
