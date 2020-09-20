/**
 * ******************************************************************************
 * @file           : bmp180.h
 * @brief          : Header for bmp180.c file.
 *                   This file contains the common defines of the BMP180 press sensor.
 * *******************************************************************************
 * @author Bilgehan Kargin
 *
 */
#include <stdio.h>
#include "main.h"

/*variables------------------------------------------------------*/
int16_t values[8];								// signed variable to store calibration values for BMP180
uint16_t Val[3];								// unsigned variable to store calibration values for BMP180
#define 	AC1 					values[0]	// label for calibration value
#define 	AC2 					values[1]	// label for calibration value
#define 	AC3 					values[2]	// label for calibration value
#define 	AC4 					Val[0]		// label for calibration value
#define 	AC5 					Val[1]		// label for calibration value
#define 	AC6 					Val[2]		// label for calibration value
#define 	B1 						values[3]	// label for calibration value
#define 	B2 						values[4]	// label for calibration value
#define 	MB 						values[5]	// label for calibration value
#define 	MC 						values[6]	// label for calibration value
#define 	MD 						values[7]	// label for calibration value
#define 	BMP180_RAW_VALUE_ADDR 	0XF6		// address to read raw values from BMP180
#define 	BMP180_CNTL_REG_ADDR 	0xF4		// address to configure BMP180
#define 	ACC_READ_ID_REG_ADDR	0x0F		// address to read ID from accelerometer
#define 	ACC_EN_DEVICE 			0x27		// enable x,y,z axis, enable normal mode, and low sample rate of IIS328DQ
#define 	RECURCISE_READ_EN 		0X80		// command to read succesive data bytes from IIS328DQ
#define 	LOW_POWER 				0x00		// LOW POWER MODE of BMP180 sensor
#define 	STANDARD 				0x01		// STANDARD MODE of BMP180 sensorCommand to read temperature from BMP180
#define 	HIGH_RESOLUTION 		0x02		// HIGH RESOLUTION MODE of BMP180 sensor
#define 	ULTRA_HIGH_RESOLUTION 	0x03		// ULTRA HIGH RESOLUTION MODE of BMP180 sensor
#define 	READ_TEMP_CMD 			0x2E		// Command to read temperature from BMP180
#define 	READ_PRESSURE_CMD 		0x34		// Command to read temperature from BMP180
#define 	PRESSURE 				0x01

/*functions prototypes ---------------------------------------------*/
void WriteRegister(I2C_HandleTypeDef hi2c,uint8_t DevAddress,char address,char content);
void ReadRegister(I2C_HandleTypeDef hi2c, uint8_t DevAddress,uint8_t Address,uint8_t * Buffer,uint8_t length);
int32_t ReadSensor(I2C_HandleTypeDef hi2c,char data,char mode,uint8_t length);
int16_t ReadInt(I2C_HandleTypeDef hi2c,char address);
uint16_t ReadUInt(I2C_HandleTypeDef hi2c, char address);
void ReadBMP180(I2C_HandleTypeDef hi2c, UART_HandleTypeDef huart);
void ReadCalPara(I2C_HandleTypeDef hi2c);
void SetupBMP(I2C_HandleTypeDef hi2c);
