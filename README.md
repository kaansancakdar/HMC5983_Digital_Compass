Objective:

The goal of this project is to create a digital compass using the HMC5983 3-axis magnetometer, interfacing it with the STM32F401 microcontroller over the SPI interface. The magnetometer's data will be calibrated using the MotionCal Calibration Tool, which helps remove offsets and other errors in the sensor data to improve the compass accuracy.

Components Needed:

HMC5983 Magnetometer Module:

This is a 3-axis digital magnetometer capable of measuring the magnetic field strength along the X, Y, and Z axes. The HMC5983 communicates over SPI or I2C, and in this project, we will use the SPI interface.
STM32F401 Microcontroller:

The STM32F401 is an ARM Cortex-M4-based microcontroller with SPI support. We will use STM32CubeIDE or STM32CubeMX for development and configuring the microcontroller.
SPI Interface Wires:

For connecting the HMC5983 to the STM32F401 microcontroller. The connections will include MISO (Master In Slave Out), MOSI (Master Out Slave In), SCK (Serial Clock), CS (Chip Select), VCC, and GND.
MotionCal Calibration Tool:

MotionCal is a software tool used to calibrate 3-axis magnetometers by collecting raw data and calculating correction factors like offsets, scaling, and rotation. This tool can improve the accuracy of the compass by eliminating errors due to sensor misalignment and external magnetic fields.

