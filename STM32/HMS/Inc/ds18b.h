/**
 * ******************************************************************************
 * @file           : ds18b.h
 * @brief          : Header for ds18b.c file.
 *                   This file contains the common defines of the Ds18b temp sensor.
 * *******************************************************************************
 * @author Bilgehan Kargin
 *
 */
#include <stdio.h>
#include "main.h"

/*variables------------------------------------------------------*/
uint8_t check;
uint8_t temp_l, temp_h;
uint16_t temp;
float temperature_for_ds18b;

/*functions prototypes ---------------------------------------------*/
void gpio_set_input (GPIO_InitTypeDef GPIO_InitStruct);
void gpio_set_output (GPIO_InitTypeDef GPIO_InitStruct);
uint8_t ds18b20_init (GPIO_InitTypeDef GPIO_InitStruct);
void write (uint8_t data,GPIO_InitTypeDef GPIO_InitStruct);
uint8_t read (GPIO_InitTypeDef GPIO_InitStruct);
void runDS18B(GPIO_InitTypeDef GPIO_InitStruct,UART_HandleTypeDef huart);
/* USER CODE BEGIN EFP */

