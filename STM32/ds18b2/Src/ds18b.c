/*
 * ds18b.c
 *
 *  Created on: 3 May 2020
 *      Author: Bilgehan Kargin
 */

/* Includes ------------------------------------------------------------------*/
#include "ds18b.h"


/* Private includes ----------------------------------------------------------*/

/*void delay(uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1) < delay);
}*/
void gpio_set_input (GPIO_InitTypeDef GPIO_InitStruct)
{
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void gpio_set_output (GPIO_InitTypeDef GPIO_InitStruct)
{
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint8_t ds18b20_init (GPIO_InitTypeDef GPIO_InitStruct)
{

	gpio_set_output (GPIO_InitStruct);   // set the pin as output
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	gpio_set_input (GPIO_InitStruct);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)))    // if the pin is low i.e the presence pulse is there
	{
		return 0;
	}

	else
	{
		delay (400);
		return 1;
	}
}

void write (uint8_t data,GPIO_InitTypeDef GPIO_InitStruct)
{
	gpio_set_output (GPIO_InitStruct);   // set as output
	for (int i=0; i<8; i++)
	{
		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			gpio_set_output (GPIO_InitStruct);  // set as output
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the pin LOW
			delay (1);  // wait for  us

			gpio_set_input (GPIO_InitStruct);  // set as input
			delay (60);  // wait for 60 us
		}
		else  // if the bit is low
		{
			// write 0
			gpio_set_output (GPIO_InitStruct);
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the pin LOW
			delay (60);  // wait for 60 us

			gpio_set_input (GPIO_InitStruct);
		}
	}
}

uint8_t read (GPIO_InitTypeDef GPIO_InitStruct)
{
	uint8_t value=0;
	gpio_set_input (GPIO_InitStruct);

	for (int i=0;i<8;i++)
	{
		gpio_set_output (GPIO_InitStruct);   // set as output

		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the data pin LOW
		delay (2);  // wait for 2 us

		gpio_set_input (GPIO_InitStruct);  // set as input
		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us
	}
	return value;
}

void runDS18B(GPIO_InitTypeDef GPIO_InitStruct,UART_HandleTypeDef huart){

		 char ds18bval[50];//it is for temperature val
		 int lenOfds18=0;
		 int i=0;
		/*FOR DS18B TEMP SENSOR */
		 check = ds18b20_init (GPIO_InitStruct);
		 HAL_Delay (1);
		 write (0xCC,GPIO_InitStruct);  // skip ROM
		 write (0x44,GPIO_InitStruct);  // convert t
		 HAL_Delay (800);
		 ds18b20_init (GPIO_InitStruct);
	     HAL_Delay(1);
		 write (0xCC,GPIO_InitStruct);  // skip ROM
		 write (0xBE,GPIO_InitStruct);  // Read Scratchpad
		 temp_l = read(GPIO_InitStruct);
		 temp_h = read(GPIO_InitStruct);
		 temp = (temp_h<<8)|temp_l;
		 temperature_for_ds18b = (float)temp/16;

		 /* FOR DS18B UART TRANSMIT*/
		 sprintf(ds18bval,"temp.val=%d%c%c%c%c",(int)temperature_for_ds18b , 0xFF, 0xFF, 0xFF,'\n');
		 while(ds18bval[i]!='\n'){
			 lenOfds18++;
			 ++i;
		 }
		 i=0;
		 HAL_UART_Transmit(&huart,ds18bval,lenOfds18,100);
}
