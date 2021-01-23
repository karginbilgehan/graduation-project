/**
 * ******************************************************************************
 * @file           : dht22.h
 * @brief          : Header for dht22.c file.
 *                   This file contains the common defines of the Dht22 temp sensor.
 * *******************************************************************************
 * @author Bilgehan Kargin
 *
 */
#include <stdio.h>
#include "main.h"



/*functions prototypes ---------------------------------------------*/
void set_gpio_output_DHT22 (void);
void set_gpio_input_DHT22 (void);
void DHT22_start (void);
void check_response (void);
uint8_t read_data_DHT22 (void);
void runDHT22(UART_HandleTypeDef huart);

/* USER CODE BEGIN EFP */

