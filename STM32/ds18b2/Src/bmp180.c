/*
 * bmp180.c
 *
 *  Created on: May 8, 2020
 *      Author: Bilgehan Kargin
 */

/* Includes ------------------------------------------------------------------*/
#include "bmp180.h"

uint8_t BMP180_ADDR = 0xEE;		//i2c address for BMP180
float a_x = 0,a_y = 0,a_z = 0;	//Accelerometer data variable
uint64_t timer1=0,timer2=0;		//variable to store time related parameters
int16_t X = 0, Y = 0, Z = 0;	//variables to store raw accelerometer data
uint8_t a_value[6]={0,0,0,0,0,0};	//Buffer to extract values from Accelerometer
int32_t GLOBAL_PRES = 0;

/*Function to read calibration parameters */
void ReadCalPara(I2C_HandleTypeDef hi2c){
    AC1 = ReadInt(hi2c,0xAA);	//Read value for AC1 register
    AC2 = ReadInt(hi2c,0xAC);	//Read value for AC2 register
    AC3 = ReadInt(hi2c,0xAE);	//Read value for AC3 register
    AC4 = ReadUInt(hi2c,0xB0);	//Read value for AC4 register
    AC5 = ReadUInt(hi2c,0xB2);	//Read value for AC5 register
    AC6 = ReadUInt(hi2c,0xB4);	//Read value for AC6 register
    B1 = ReadInt(hi2c,0xB6);		//Read value for B1 register
    B2 = ReadInt(hi2c,0xB8);		//Read value for B2 register
    MB = ReadInt(hi2c,0xBA);		//Read value for MB register
    MC= ReadInt(hi2c,0xBC);		//Read value for MC register
    MD = ReadInt(hi2c,0xBE);		//Read value for MD register
}

/*Read signed integer from I2C bus*/
int16_t ReadInt(I2C_HandleTypeDef hi2c,char address){
	uint8_t temp[2];									// Temporary variable to store raw value
	int16_t result=0;								// variable to be return
	ReadRegister(hi2c,BMP180_ADDR,address,temp,2);		// Read raw value from I2C bus
	result=((temp[0]<<8)|temp[1]);					// convert the raw value to integer
	return result;									// Return integer
}

/*Read unsigned integer from I2C bus*/
uint16_t ReadUInt(I2C_HandleTypeDef hi2c, char address){
	uint8_t temp[2];									// Temporary variable to store raw value
	uint16_t result=0;								// variable to be return
	ReadRegister(hi2c,BMP180_ADDR,address,temp,2);		// Read raw value from I2C bus
	result=((temp[0]<<8)|temp[1]);					// convert the raw value to integer
	return result;									// Return integer
}

/*Read values from BMP180 sensor*/
void ReadBMP180(I2C_HandleTypeDef hi2c, UART_HandleTypeDef huart){
	int32_t X1,X2,B5,ut,p,B6,X3,B3;					// Temporary signed variable to calculate pressure and temperature
	uint32_t B4,B7;									// Temporary unsigned variable to calculate pressure and temperature
	char sampling_mode = LOW_POWER;		// Working mode for BMP180 sensor
	float temperature=0.0;							// variable to carry temperature data
	int32_t UT,UP;									// variable to carry integer value of Temperature and pressure

	char dizi2[50];// it is for press val
	int lenOfDizi2=0;
	int i=0;

	UT=ReadSensor(hi2c,READ_TEMP_CMD,sampling_mode,2);		// Read Temperature and given mode
	if(UT!=0){										// check for valid reading
		X1=(UT-AC6) * AC5>>15;						// calculation to obtain
		X2=(MC<<11)/(X1+MD);						// Temperature from raw
		B5=X1+X2;									// values for more details
		ut=(B5+8)>>4;								// refer section 3.5 in BMP180
		temperature = ut *0.1;						// sensor datasheet.

		UP=ReadSensor(hi2c,READ_PRESSURE_CMD,sampling_mode,3);			// read raw values of pressure
		if(UP!=0){									// check for valid reading
			UP = (UP>>(8-sampling_mode));
			B6=B5-4000;								// calculations to extract pressure
			X1=(B2*((B6*B6)>>12))>>11;				// reading from BMP180 sensor
			X2=(AC2*B6)>>11;						// all the calculations shown
			X3=X1+X2;								// in the datasheet are carry
			B3=((((AC1*4)+X3)<<sampling_mode)+2)/4;// forward in this step
			X1=(AC3*B6)>>13;						// for more details on this
			X2=(B1*((B6*B6)>>12))>>16;				// please refer section 3.5 in
			X3 = (X1 + X2 + 2) >> 2;				// BMP180 sensor datasheet.
			B4=(uint32_t)AC4*(uint32_t)(X3+32768)>>15;					//
			B7=(uint32_t)(UP-B3)*(50000>>sampling_mode);		//
			if (B7 < 0x80000000)					//
				p = (B7 * 2) / B4;					//
			else									//
				p = (B7 / B4) * 2;					//
			X1 = (p >> 8) * (p >> 8);				//
			X1 = (X1 * 3038) >> 16;					//
			X2 = (-7357 * p) >> 16;					//
			GLOBAL_PRES = (p + ((X1 + X2 + 3791) >> 4));		//

			sprintf(dizi2,"press.val=%d%c%c%c%c",(int)GLOBAL_PRES/100, 0xFF, 0xFF, 0xFF,'\n');
			while(dizi2[i]!='\n'){
				lenOfDizi2++;
				++i;
			}
			i=0;
			HAL_UART_Transmit(&huart,dizi2,lenOfDizi2,100);
		}
	}
}

/*Function to Read the raw values from the sensor*/
int32_t ReadSensor(I2C_HandleTypeDef hi2c,char data,char mode,uint8_t length){
	uint8_t response[3],x=0;							//Local variables declaration
	data = data | (mode<<6);						//append mode in control register
	WriteRegister(hi2c,BMP180_ADDR,BMP180_CNTL_REG_ADDR,data);//start pressure sensor in given mode
	do{
		x++;										//increment x to each milliseconds
		HAL_Delay(1);									//delay for 1 milliseconds
		ReadRegister(hi2c,BMP180_ADDR,BMP180_CNTL_REG_ADDR,&response[0],1);//read busy status of sensor
		if(x>=30)									//if no response received with
			return 0;								// 30 milliseconds then return 0.
	}while((response[0]&0x20)!=0x00);				// wait until sensor is busy
	ReadRegister(hi2c,BMP180_ADDR,BMP180_RAW_VALUE_ADDR,response,length);//read the raw data from sensor
	int32_t result= 0 ;
	if(length==2)
		result = ((response[0]<<8) | response[1]);	// convert bytes into signed integer
	else if(length==3)
		result = ((response[0]<<16) | (response[1]<<8) | response[2]);//convert bytes into signed integer
	return result;									//return the result
}

/*function to setup BMP180 sensor*/
void SetupBMP(I2C_HandleTypeDef hi2c){
	uint8_t prxbuf[1];									// local variable
	ReadRegister(hi2c,BMP180_ADDR,0xD0,prxbuf,1);		// read BMP180 sensor ID
//	Debug.printf("BMP ID : %02x \r\n",prxbuf[0]);	// display ID
	ReadCalPara(hi2c);									// read calibration data from the sensor
}

/*function to read RAW values from I2C bus*/
void ReadRegister(I2C_HandleTypeDef hi2c, uint8_t DevAddress,uint8_t Address,uint8_t * Buffer,uint8_t length){
	HAL_I2C_Master_Transmit(&hi2c,DevAddress&0xFE,&Address,1,10);// send device address and register address
	HAL_I2C_Master_Receive(&hi2c,DevAddress|0x01,Buffer,length,10);// read data from sensor
}

/*function to write RAW values on I2C bus*/
void WriteRegister(I2C_HandleTypeDef hi2c,uint8_t DevAddress,char address,char content){
	uint8_t data[2]={address,content};					// local variable to store addres and data to be written
	HAL_I2C_Master_Transmit(&hi2c,DevAddress&0xFE,data,2,10);//	write the address and data to the I2C bus
}
