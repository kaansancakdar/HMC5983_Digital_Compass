# **Objective:**

The goal of this project is to create a digital compass using the HMC5983 3-axis magnetometer, interfacing it with the STM32F401 microcontroller over the SPI interface. The magnetometer's data will be calibrated using the MotionCal Calibration Tool, which helps remove offsets and other errors in the sensor data to improve the compass accuracy.

# **Components Needed:**

## 1.HMC5983 Magnetometer Module:

This is a 3-axis digital magnetometer capable of measuring the magnetic field strength along the X, Y, and Z axes. The HMC5983 communicates over SPI or I2C, and in this project, we will use the SPI interface.
## 2.STM32F401 Microcontroller:

The STM32F401 is an ARM Cortex-M4-based microcontroller with SPI support. We will use STM32CubeIDE or STM32CubeMX for development and configuring the microcontroller.
## 3.SPI Interface Wires:

For connecting the HMC5983 to the STM32F401 microcontroller. The connections will include MISO (Master In Slave Out), MOSI (Master Out Slave In), SCK (Serial Clock), CS (Chip Select), VCC, and GND.
## 4.MotionCal Calibration Tool:

MotionCal is a software tool used to calibrate 3-axis magnetometers by collecting raw data and calculating correction factors like offsets, scaling, and rotation. This tool can improve the accuracy of the compass by eliminating errors due to sensor misalignment and external magnetic fields.

Download and learn how to use this tool:  https://www.pjrc.com/store/prop_shield.html
Example: https://www.youtube.com/watch?v=cGI8mrIanpk

# **Key Steps in the Project:**
## 1. Wiring the Components:
HMC5983 Magnetometer to STM32F401 (SPI Interface):\
    MISO (HMC5983) to MOSI (STM32F401): For data input.\
    MOSI (STM32F401) to MISO (HMC5983): For data output (if required for other sensors).\
    SCK (HMC5983) to SCK (STM32F401): For the clock signal.\
    CS (HMC5983) to a GPIO pin on the STM32F401: To select the chip.\
    VCC (HMC5983) to 3.3V (STM32F401).\
    GND (HMC5983) to GND (STM32F401).\

## 2.Configuring STM32F401 with STM32CubeMX:
Open STM32CubeMX and create a new project for the STM32F401 microcontroller.\
Enable the SPI interface and configure it for communication with the HMC5983.v
Set up the appropriate GPIO pins for CS (Chip Select), SCK, MISO, and MOSI.\
Enable the DMA for SPI communication to handle data transfers efficiently.\
Configure the microcontroller clock settings and peripheral settings.\
Generate the initialization code using STM32CubeMX, then open it in STM32CubeIDE.\
