/*
 * dht22.c
 *
 *  Created on: May 9, 2020
 *      Author: Bilgehan Kargin
 */

/* Includes ------------------------------------------------------------------*/
#include "dht22.h"

/*variables------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStruct2;
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum, RH, TEMP,TEMP1,TEMP2, RH1,RH2;
uint8_t checkForDHT22 = 0;


void set_gpio_output_DHT22 (void)
{
	/*Configure GPIO pin output: PA4 */
  GPIO_InitStruct2.Pin = GPIO_PIN_4;
  GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct2);
}

void set_gpio_input_DHT22 (void)
{
	/*Configure GPIO pin input: PA4 */
  GPIO_InitStruct2.Pin = GPIO_PIN_4;
  GPIO_InitStruct2.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct2.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct2);
}
void DHT22_start (void)
{
		set_gpio_output_DHT22();  // set the pin as output
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, 0);   // pull the pin low
		delay (18000);   // wait for 18ms
		set_gpio_input_DHT22();   // set as input
}
void check_response (void)
{
	delay (40);
	if (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4))) checkForDHT22 = 1;
	}
	while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)));   // wait for the pin to go low
}

uint8_t read_data_DHT22 (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)));   // wait for the pin to go high
		delay (40);   // wait for 40 us
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)) == 0)   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)));  // wait for the pin to go low
	}
	return i;
}

void runDHT22(UART_HandleTypeDef huart){
		 int i=0;
		 char dht22Temp[50],dht22humidity[50];//it is for temperature val
		 int lenOfdht22Temp=0;
		 int lenOfdht22humidity=0;

		 /*FOR DHT22 SENSOR */
		 DHT22_start ();
		 check_response ();
		 Rh_byte1 = read_data_DHT22 ();
		 Rh_byte2 = read_data_DHT22 ();
		 Temp_byte1 = read_data_DHT22 ();
		 Temp_byte2 = read_data_DHT22 ();
		 sum = read_data_DHT22();
		 if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
		 {
		 	TEMP = ((Temp_byte1<<8)|Temp_byte2);
		 	RH = ((Rh_byte1<<8)|Rh_byte2);
		 }
		 TEMP1 = (Temp_byte1/10);
		 TEMP2 = (Temp_byte1 %10);
		 sprintf(dht22Temp,"temp1.val=%d%d%c%c%c%c",(int)TEMP1,(int)TEMP2, 0xFF, 0xFF, 0xFF,'\n');
		 while(dht22Temp[i]!='\n'){
			 lenOfdht22Temp++;
			++i;
	     }
		 i=0;
		 HAL_UART_Transmit(&huart,dht22Temp,lenOfdht22Temp,100);


		 RH1 = (Rh_byte1 / 10);
		 RH2 = (Rh_byte1 % 10);
		 sprintf(dht22humidity,"humidity.val=%d%d%c%c%c%c",(int)RH1,(int)RH2, 0xFF, 0xFF, 0xFF,'\n');
		 while(dht22humidity[i]!='\n'){
			 lenOfdht22humidity++;
			++i;
		}
		i=0;
		HAL_UART_Transmit(&huart,dht22humidity,lenOfdht22humidity,20);
}
